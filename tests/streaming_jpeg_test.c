#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <jpeglib.h>

#include "streaming/jpeg.h"

static void round_trip(unsigned quality, unsigned output_width, unsigned output_height) {
    enum { width = 37, height = 29, stride = 43 };
    uint8_t input[stride * height];
    memset(input, 0xee, sizeof(input));
    for (unsigned y = 0; y < height; ++y)
        for (unsigned x = 0; x < width; ++x)
            input[y * stride + x] = (uint8_t)((x * 5 + y * 3) % 256);
    dart_jpeg_encoder_t encoder;
    assert(dart_jpeg_init(&encoder, quality) == DART_STATUS_OK);
    uint8_t output[32768];
    size_t written = 100;
    assert(
        dart_jpeg_encode(
            &encoder, input, sizeof(input), width, height, stride, output_width, output_height,
            output, sizeof(output), &written)
        == 0);
    assert(written > 100 && written < sizeof(output));
    struct jpeg_decompress_struct decoder;
    struct jpeg_error_mgr errors;
    decoder.err = jpeg_std_error(&errors);
    jpeg_create_decompress(&decoder);
    jpeg_mem_src(&decoder, output, written);
    assert(jpeg_read_header(&decoder, TRUE) == JPEG_HEADER_OK);
    assert(jpeg_start_decompress(&decoder));
    assert(
        decoder.output_width == output_width && decoder.output_height == output_height
        && decoder.output_components == 1);
    unsigned total_error = 0;
    while (decoder.output_scanline < decoder.output_height) {
        const unsigned y = decoder.output_scanline;
        uint8_t row[width];
        JSAMPROW rows[] = {row};
        assert(jpeg_read_scanlines(&decoder, rows, 1) == 1);
        for (unsigned x = 0; x < output_width; ++x) {
            const unsigned sx = x * width / output_width;
            const unsigned sy = y * height / output_height;
            total_error += (unsigned)abs((int)row[x] - input[sy * stride + sx]);
        }
    }
    if (quality == 100)
        assert(total_error < output_width * output_height * 2);
    assert(jpeg_finish_decompress(&decoder));
    jpeg_destroy_decompress(&decoder);
    memset(output, 0x5a, sizeof(output));
    assert(
        dart_jpeg_encode(
            &encoder, input, sizeof(input), width, height, stride, output_width, output_height,
            output + 1, 20, &written)
        == DART_STATUS_OUT_OF_RANGE);
    assert(written == 0 && output[0] == 0x5a && output[21] == 0x5a);
    assert(
        dart_jpeg_encode(
            &encoder, input, width - 1, width, height, stride, output_width, output_height, output,
            sizeof(output), &written)
        == DART_STATUS_INVALID_ARGUMENT);
    assert(written == 0);
}
int main(void) {
    dart_jpeg_encoder_t encoder;
    assert(dart_jpeg_init(&encoder, 0) == DART_STATUS_INVALID_ARGUMENT);
    assert(dart_jpeg_init(&encoder, 101) == DART_STATUS_INVALID_ARGUMENT);
    for (unsigned quality = 1; quality <= 100; ++quality) {
        round_trip(quality, 37, 29);
        round_trip(quality, 19, 15);
        round_trip(quality, 1, 1);
    }
    return 0;
}
