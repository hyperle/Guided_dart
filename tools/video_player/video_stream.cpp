#include "tools/video_player/video_stream.hpp"

#include <QDateTime>
#include <QFile>
#include <QImage>
#include <QString>
#include <cerrno>
#include <cstdint>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/error.h>
#include <libswscale/swscale.h>
}

namespace dart::tools::video_player {

QString video_error(int status) {
    char message[AV_ERROR_MAX_STRING_SIZE]{};
    av_strerror(status, message, sizeof(message));
    return QString::fromUtf8(message);
}

class VideoDecoder final {
public:
    ~VideoDecoder() {
        av_packet_free(&packet);
        av_frame_free(&picture);
        avcodec_free_context(&context_);
    }

    bool open(const AVCodecParameters* parameters, QString& error) {
        avcodec_free_context(&context_);
        const auto* codec = avcodec_find_decoder(parameters->codec_id);
        context_ = avcodec_alloc_context3(codec);
        if (!packet)
            packet = av_packet_alloc();
        if (!picture)
            picture = av_frame_alloc();
        if (!codec || !context_ || !packet || !picture) {
            error = QStringLiteral("Video decoder unavailable or insufficient memory");
            return false;
        }
        int status = avcodec_parameters_to_context(context_, parameters);
        if (status >= 0) {
            context_->thread_count = 1;
            context_->err_recognition = AV_EF_EXPLODE;
            status = avcodec_open2(context_, codec, nullptr);
        }
        if (status < 0)
            error = video_error(status);
        return status >= 0;
    }

    int send(const AVPacket* input) { return avcodec_send_packet(context_, input); }
    int receive() {
        av_frame_unref(picture);
        return avcodec_receive_frame(context_, picture);
    }

    AVPacket* packet = nullptr;
    AVFrame* picture = nullptr;

private:
    AVCodecContext* context_ = nullptr;
};

struct VideoStream::Index {
    struct Packet {
        qint64 offset = 0;
        int bytes = 0;
        int flags = 0;
        std::size_t key = 0;
    };
    QString path;
    qint64 bytes = 0;
    qint64 modified = 0;
    AVCodecParameters* parameters = avcodec_parameters_alloc();
    std::vector<Packet> packets;
    std::vector<std::size_t> pictures;

    ~Index() { avcodec_parameters_free(&parameters); }

    static bool independent(const AVPacket& packet, AVCodecID codec) {
        for (int offset = 0; offset + 3 < packet.size; ++offset) {
            const auto* data = packet.data + offset;
            if (data[0] != 0 || data[1] != 0 || data[2] != 1)
                continue;
            if (codec == AV_CODEC_ID_H264 && (data[3] & 31U) == 5U)
                return true;
            const auto kind = (data[3] >> 1U) & 63U;
            if (codec == AV_CODEC_ID_HEVC && kind >= 16U && kind <= 20U)
                return true;
        }
        return false;
    }
};

VideoStream::VideoStream()
    : index_(std::make_unique<Index>()) {}
VideoStream::~VideoStream() = default;

bool VideoStream::supports(const QString& suffix) {
    const auto lower = suffix.toLower();
    return lower == "h264" || lower == "h265" || lower == "hevc";
}

bool VideoStream::append(
    const QFileInfo& file, const QString& relative_name, std::vector<Frame>& frames,
    QString& error) {
    auto video = std::shared_ptr<VideoStream>(new VideoStream());
    auto& index = *video->index_;
    index.path = file.absoluteFilePath();
    index.bytes = file.size();
    index.modified = file.lastModified().toMSecsSinceEpoch();
    const auto fail = [&](const QString& reason) {
        error = index.path + ": " + reason;
        return false;
    };
    struct Input {
        AVFormatContext* context = nullptr;
        ~Input() { avformat_close_input(&context); }
    } input;
    const auto* format = av_find_input_format(file.suffix().toLower() == "h264" ? "h264" : "hevc");
    const auto path = QFile::encodeName(index.path);
    int status = avformat_open_input(&input.context, path.constData(), format, nullptr);
    if (status < 0)
        return fail(video_error(status));
    status = avformat_find_stream_info(input.context, nullptr);
    if (status < 0)
        return fail(video_error(status));
    const int stream = av_find_best_stream(input.context, AVMEDIA_TYPE_VIDEO, -1, -1, nullptr, 0);
    if (stream < 0)
        return fail(video_error(stream));
    if (!index.parameters)
        return fail(QStringLiteral("Insufficient memory for video parameters"));
    status = avcodec_parameters_copy(index.parameters, input.context->streams[stream]->codecpar);
    if (status < 0)
        return fail(video_error(status));
    VideoDecoder decoder;
    if (!decoder.open(index.parameters, error))
        return fail(error);
    const auto receive = [&]() {
        int result = 0;
        while ((result = decoder.receive()) >= 0) {
            const auto* picture = decoder.picture;
            if (picture->pts < 0 || static_cast<std::uint64_t>(picture->pts) >= index.packets.size()
                || picture->decode_error_flags != 0 || (picture->flags & AV_FRAME_FLAG_CORRUPT))
                return AVERROR_INVALIDDATA;
            index.pictures.push_back(static_cast<std::size_t>(picture->pts));
        }
        return result == AVERROR(EAGAIN) || result == AVERROR_EOF ? 0 : result;
    };
    std::size_t key = 0;
    while ((status = av_read_frame(input.context, decoder.packet)) >= 0) {
        auto* packet = decoder.packet;
        if (packet->stream_index != stream) {
            av_packet_unref(packet);
            continue;
        }
        if (packet->pos < 0 || packet->size <= 0 || packet->pos > index.bytes - packet->size)
            return fail(QStringLiteral("Invalid video packet offset/size"));
        if ((packet->flags & AV_PKT_FLAG_KEY)
            && Index::independent(*packet, index.parameters->codec_id))
            key = index.packets.size();
        packet->pts = static_cast<std::int64_t>(index.packets.size());
        packet->dts = AV_NOPTS_VALUE;
        index.packets.push_back({packet->pos, packet->size, packet->flags, key});
        status = decoder.send(packet);
        av_packet_unref(packet);
        if (status < 0 || (status = receive()) < 0)
            return fail(video_error(status));
    }
    if (status != AVERROR_EOF)
        return fail(video_error(status));
    status = decoder.send(nullptr);
    if (status < 0 || (status = receive()) < 0)
        return fail(video_error(status));
    if (index.pictures.empty())
        return fail(QStringLiteral("No decodable video frames"));
    for (std::size_t picture = 0; picture < index.pictures.size(); ++picture) {
        const auto& packet = index.packets[index.pictures[picture]];
        Frame frame;
        frame.image_index = picture + 1;
        frame.path = index.path;
        frame.relative_name = relative_name;
        frame.file_offset = packet.offset;
        frame.file_bytes = packet.bytes;
        frame.video = video;
        frame.video_frame_index = picture;
        frames.push_back(std::move(frame));
    }
    return true;
}

struct VideoImageReader::State {
    std::shared_ptr<VideoStream> video;
    QFile file;
    VideoDecoder decoder;
    SwsContext* scaler = nullptr;
    std::size_t next_packet = 0;
    quint64 image_index = 0;
    bool flushed = false;
    QImage image;

    ~State() { sws_freeContext(scaler); }
};

VideoImageReader::VideoImageReader() = default;
VideoImageReader::~VideoImageReader() = default;

QImage VideoImageReader::read(const Frame& frame, QString& error) {
    const auto fail = [&](const QString& reason) -> QImage {
        error = frame.path + ": " + reason;
        state_.reset();
        return {};
    };
    if (!frame.video)
        return fail(QStringLiteral("Missing video index"));
    const auto& index = *frame.video->index_;
    const QFileInfo current(index.path);
    if (current.size() != index.bytes
        || current.lastModified().toMSecsSinceEpoch() != index.modified)
        return fail(QStringLiteral("Video file changed; reload the recording"));
    if (frame.video_frame_index >= index.pictures.size())
        return fail(QStringLiteral("Invalid video frame index"));
    const auto wanted = index.pictures[frame.video_frame_index];
    const auto key = index.packets[wanted].key;
    if (!state_ || state_->video != frame.video || state_->image_index > frame.image_index
        || state_->next_packet < key) {
        state_ = std::make_unique<State>();
        state_->video = frame.video;
        state_->file.setFileName(index.path);
        if (!state_->file.open(QIODevice::ReadOnly))
            return fail(state_->file.errorString());
        if (state_->file.size() != index.bytes)
            return fail(QStringLiteral("Video file size changed; reload the recording"));
        if (!state_->decoder.open(index.parameters, error))
            return fail(error);
        state_->next_packet = key;
    }
    auto& state = *state_;
    if (state.image_index == frame.image_index)
        return state.image;
    while (true) {
        int status = state.decoder.receive();
        if (status >= 0) {
            const auto* picture = state.decoder.picture;
            if (picture->pts != static_cast<std::int64_t>(wanted))
                continue;
            if (picture->decode_error_flags != 0 || (picture->flags & AV_FRAME_FLAG_CORRUPT))
                return fail(QStringLiteral("Corrupt video frame"));
            QImage image(picture->width, picture->height, QImage::Format_RGB888);
            if (image.isNull())
                return fail(QStringLiteral("Insufficient memory for video image"));
            state.scaler = sws_getCachedContext(
                state.scaler, picture->width, picture->height,
                static_cast<AVPixelFormat>(picture->format), picture->width, picture->height,
                AV_PIX_FMT_RGB24, SWS_BILINEAR, nullptr, nullptr, nullptr);
            if (!state.scaler)
                return fail(QStringLiteral("Cannot convert video pixel format"));
            const auto* coefficients = sws_getCoefficients(static_cast<int>(picture->colorspace));
            status = sws_setColorspaceDetails(
                state.scaler, coefficients, picture->color_range == AVCOL_RANGE_JPEG, coefficients,
                1, 0, 1 << 16, 1 << 16);
            if (status < 0)
                return fail(video_error(status));
            std::uint8_t* pixels[4]{image.bits()};
            if (image.bytesPerLine() > std::numeric_limits<int>::max())
                return fail(QStringLiteral("Video image stride is too large"));
            const int strides[4]{static_cast<int>(image.bytesPerLine())};
            status = sws_scale(
                state.scaler, picture->data, picture->linesize, 0, picture->height, pixels,
                strides);
            if (status != picture->height)
                return fail(QStringLiteral("Cannot convert video frame"));
            state.image_index = frame.image_index;
            state.image = image;
            return image;
        }
        if (status != AVERROR(EAGAIN))
            return fail(
                status == AVERROR_EOF ? QStringLiteral("Video frame not found")
                                      : video_error(status));
        if (state.next_packet == index.packets.size()) {
            if (state.flushed)
                return fail(QStringLiteral("Video decoder did not finish"));
            status = state.decoder.send(nullptr);
            state.flushed = true;
        } else {
            const auto& entry = index.packets[state.next_packet];
            auto* packet = state.decoder.packet;
            av_packet_unref(packet);
            status = av_new_packet(packet, entry.bytes);
            if (status < 0)
                return fail(video_error(status));
            if (!state.file.seek(entry.offset)
                || state.file.read(reinterpret_cast<char*>(packet->data), entry.bytes)
                       != entry.bytes)
                return fail(QStringLiteral("Incomplete video packet"));
            packet->pts = static_cast<std::int64_t>(state.next_packet++);
            packet->dts = AV_NOPTS_VALUE;
            packet->pos = entry.offset;
            packet->flags = entry.flags;
            status = state.decoder.send(packet);
            av_packet_unref(packet);
        }
        if (status < 0)
            return fail(video_error(status));
    }
}

} // namespace dart::tools::video_player
