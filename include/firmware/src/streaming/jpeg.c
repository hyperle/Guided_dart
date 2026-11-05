#include "streaming/jpeg.h"

#include <math.h>
#include <string.h>

static const uint8_t zigzag[64] = {0,  1,  8,  16, 9,  2,  3,  10, 17, 24, 32, 25, 18, 11, 4,  5,
                                   12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13, 6,  7,  14, 21, 28,
                                   35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
                                   58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63};
static const uint8_t base_quantization[64] = {
    16, 11, 10, 16, 24,  40,  51,  61,  12, 12, 14, 19, 26,  58,  60,  55,
    14, 13, 16, 24, 40,  57,  69,  56,  14, 17, 22, 29, 51,  87,  80,  62,
    18, 22, 37, 56, 68,  109, 103, 77,  24, 35, 55, 64, 81,  104, 113, 92,
    49, 64, 78, 87, 103, 121, 120, 101, 72, 92, 95, 98, 112, 100, 103, 99};
/* T.81 Annex K luminance Huffman tables, length counts followed by symbols. */
static const uint8_t dc_counts[16] = {0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0};
static const uint8_t dc_symbols[12] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
static const uint8_t ac_counts[16] = {0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 125};
static const uint8_t ac_symbols[162] = {
    0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12, 0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61,
    0x07, 0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08, 0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52,
    0xd1, 0xf0, 0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x25,
    0x26, 0x27, 0x28, 0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x43, 0x44, 0x45,
    0x46, 0x47, 0x48, 0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64,
    0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x83,
    0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99,
    0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6,
    0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3,
    0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8,
    0xe9, 0xea, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa};

static void build_codes(dart_jpeg_code_t* table, const uint8_t* counts, const uint8_t* symbols) {
    unsigned code = 0, index = 0;
    for (unsigned length = 1; length <= 16; ++length) {
        for (unsigned i = 0; i < counts[length - 1]; ++i) {
            table[symbols[index++]] = (dart_jpeg_code_t){(uint16_t)code++, (uint8_t)length};
        }
        code <<= 1;
    }
}

dart_status_t dart_jpeg_init(dart_jpeg_encoder_t* encoder, uint32_t quality) {
    if (encoder == NULL || quality < 1 || quality > 100)
        return DART_STATUS_INVALID_ARGUMENT;
    memset(encoder, 0, sizeof(*encoder));
    const uint32_t scale = quality < 50 ? 5000 / quality : 200 - 2 * quality;
    for (unsigned i = 0; i < 64; ++i) {
        uint32_t value = (base_quantization[i] * scale + 50) / 100;
        encoder->quantization[i] = (uint8_t)(value < 1 ? 1 : value > 255 ? 255 : value);
    }
    for (unsigned u = 0; u < 8; ++u)
        for (unsigned x = 0; x < 8; ++x)
            encoder->basis[u][x] = 0.5F * (u == 0 ? 0.7071067812F : 1.0F)
                                 * cosf((float)((2 * x + 1) * u) * 0.19634954085F);
    build_codes(encoder->dc, dc_counts, dc_symbols);
    build_codes(encoder->ac, ac_counts, ac_symbols);
    encoder->initialized = 1;
    return DART_STATUS_OK;
}

typedef struct writer {
    uint8_t* output;
    size_t capacity, used;
    uint32_t bits;
    unsigned count;
    int overflow;
} writer_t;

static void byte(writer_t* out, uint8_t value) {
    if (out->used < out->capacity)
        out->output[out->used++] = value;
    else
        out->overflow = 1;
}
static void word(writer_t* out, uint16_t value) {
    byte(out, (uint8_t)(value >> 8));
    byte(out, (uint8_t)value);
}
static void marker(writer_t* out, uint8_t value) {
    byte(out, 0xff);
    byte(out, value);
}
static void bits(writer_t* out, unsigned value, unsigned count) {
    out->bits = (out->bits << count) | (value & ((1U << count) - 1));
    out->count += count;
    while (out->count >= 8) {
        out->count -= 8;
        const uint8_t value_byte = (uint8_t)(out->bits >> out->count);
        byte(out, value_byte);
        if (value_byte == 0xff)
            byte(out, 0);
    }
}
static void huffman(writer_t* out, const dart_jpeg_code_t* table, unsigned symbol) {
    bits(out, table[symbol].bits, table[symbol].length);
}
static unsigned magnitude(int value) {
    unsigned count = 0, absolute = (unsigned)(value < 0 ? -value : value);
    while (absolute != 0) {
        ++count;
        absolute >>= 1;
    }
    return count;
}
static void coefficient(writer_t* out, int value, unsigned count) {
    bits(out, (unsigned)(value < 0 ? value - 1 : value), count);
}
static void table_header(
    writer_t* out, uint8_t id, const uint8_t* counts, const uint8_t* symbols, unsigned size) {
    marker(out, 0xc4);
    word(out, (uint16_t)(19 + size));
    byte(out, id);
    for (unsigned i = 0; i < 16; ++i)
        byte(out, counts[i]);
    for (unsigned i = 0; i < size; ++i)
        byte(out, symbols[i]);
}

static void encode_block(
    writer_t* out, const dart_jpeg_encoder_t* encoder, const uint8_t* pixels, uint32_t width,
    uint32_t height, uint32_t stride, uint32_t output_width, uint32_t output_height, uint32_t bx,
    uint32_t by, int* previous_dc) {
    float row_dct[8][8];
    int quantized[64];
    for (unsigned y = 0; y < 8; ++y) {
        const uint32_t oy = by + y < output_height ? by + y : output_height - 1;
        const uint32_t sy = (uint32_t)((uint64_t)oy * height / output_height);
        float row[8];
        for (unsigned x = 0; x < 8; ++x) {
            const uint32_t ox = bx + x < output_width ? bx + x : output_width - 1;
            const uint32_t sx = (uint32_t)((uint64_t)ox * width / output_width);
            row[x] = (float)pixels[(size_t)sy * stride + sx] - 128.0F;
        }
        for (unsigned u = 0; u < 8; ++u) {
            float sum = 0;
            for (unsigned x = 0; x < 8; ++x)
                sum += row[x] * encoder->basis[u][x];
            row_dct[y][u] = sum;
        }
    }
    for (unsigned v = 0; v < 8; ++v)
        for (unsigned u = 0; u < 8; ++u) {
            float sum = 0;
            for (unsigned y = 0; y < 8; ++y)
                sum += row_dct[y][u] * encoder->basis[v][y];
            const unsigned index = v * 8 + u;
            const int value = (int)lroundf(sum / encoder->quantization[index]);
            quantized[index] = value < -1023 ? -1023 : value > 1023 ? 1023 : value;
        }
    const int difference = quantized[0] - *previous_dc;
    *previous_dc = quantized[0];
    const unsigned dc_size = magnitude(difference);
    huffman(out, encoder->dc, dc_size);
    coefficient(out, difference, dc_size);
    unsigned zeroes = 0;
    for (unsigned i = 1; i < 64; ++i) {
        const int value = quantized[zigzag[i]];
        if (value == 0) {
            ++zeroes;
            continue;
        }
        while (zeroes >= 16) {
            huffman(out, encoder->ac, 0xf0);
            zeroes -= 16;
        }
        const unsigned size = magnitude(value);
        huffman(out, encoder->ac, zeroes * 16 + size);
        coefficient(out, value, size);
        zeroes = 0;
    }
    if (zeroes != 0)
        huffman(out, encoder->ac, 0);
}

dart_status_t dart_jpeg_encode(
    const dart_jpeg_encoder_t* encoder, const uint8_t* pixels, size_t pixels_size, uint32_t width,
    uint32_t height, uint32_t stride, uint32_t output_width, uint32_t output_height,
    uint8_t* output, size_t capacity, size_t* written) {
    if (written == NULL)
        return DART_STATUS_INVALID_ARGUMENT;
    *written = 0;
    if (encoder == NULL || !encoder->initialized || pixels == NULL || output == NULL || width == 0
        || height == 0 || stride < width || output_width == 0 || output_height == 0
        || output_width > width || output_height > height || output_width > 65535
        || output_height > 65535 || (uint64_t)(height - 1) * stride + width > pixels_size)
        return DART_STATUS_INVALID_ARGUMENT;
    writer_t out = {.output = output, .capacity = capacity};
    marker(&out, 0xd8);
    marker(&out, 0xdb);
    word(&out, 67);
    byte(&out, 0);
    for (unsigned i = 0; i < 64; ++i)
        byte(&out, encoder->quantization[zigzag[i]]);
    marker(&out, 0xc0);
    word(&out, 11);
    byte(&out, 8);
    word(&out, (uint16_t)output_height);
    word(&out, (uint16_t)output_width);
    byte(&out, 1);
    byte(&out, 1);
    byte(&out, 0x11);
    byte(&out, 0);
    table_header(&out, 0x00, dc_counts, dc_symbols, sizeof(dc_symbols));
    table_header(&out, 0x10, ac_counts, ac_symbols, sizeof(ac_symbols));
    marker(&out, 0xda);
    word(&out, 8);
    byte(&out, 1);
    byte(&out, 1);
    byte(&out, 0);
    byte(&out, 0);
    byte(&out, 63);
    byte(&out, 0);
    int previous_dc = 0;
    for (uint32_t y = 0; y < output_height && !out.overflow; y += 8)
        for (uint32_t x = 0; x < output_width && !out.overflow; x += 8)
            encode_block(
                &out, encoder, pixels, width, height, stride, output_width, output_height, x, y,
                &previous_dc);
    if (out.count != 0)
        bits(&out, (1U << (8 - out.count)) - 1, 8 - out.count);
    marker(&out, 0xd9);
    if (out.overflow)
        return DART_STATUS_OUT_OF_RANGE;
    *written = out.used;
    return DART_STATUS_OK;
}
