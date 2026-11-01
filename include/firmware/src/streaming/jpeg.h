#ifndef DART_STREAMING_JPEG_H
#define DART_STREAMING_JPEG_H

#include <stddef.h>
#include <stdint.h>

#include "status.h"

#ifdef __cplusplus
extern "C" {
#endif

/* CPU-only baseline grayscale JPEG. The caller owns the workspace and output;
 * initialization computes tables once, encoding never allocates or uses DMA.
 * Input remains borrowed until encode returns; output is valid up to *written.
 */
typedef struct dart_jpeg_code {
    uint16_t bits;
    uint8_t length;
} dart_jpeg_code_t;

typedef struct dart_jpeg_encoder {
    float basis[8][8];
    uint8_t quantization[64];
    dart_jpeg_code_t dc[256];
    dart_jpeg_code_t ac[256];
    uint32_t initialized;
} dart_jpeg_encoder_t;

dart_status_t dart_jpeg_init(dart_jpeg_encoder_t* encoder, uint32_t quality);
dart_status_t dart_jpeg_encode(
    const dart_jpeg_encoder_t* encoder, const uint8_t* pixels, size_t pixels_size, uint32_t width,
    uint32_t height, uint32_t stride, uint32_t output_width, uint32_t output_height,
    uint8_t* output, size_t capacity, size_t* written);

#ifdef __cplusplus
}
#endif
#endif
