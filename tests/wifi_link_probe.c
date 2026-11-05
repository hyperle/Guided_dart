#define _POSIX_C_SOURCE 200809L
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "streaming/http.h"
#include "streaming/jpeg.h"

/* Host integration probe uses the same C encoder and uploader as dart_app.
 * This is not a camera/HAL simulation. Never deployed to the board.
 */
static int cancelled(void* context) {
    (void)context;
    return 0;
}
int main(int argc, char** argv) {
    if (argc != 5)
        return 2;
    dart_http_client_t client = {.socket = -1};
    int result = dart_http_open(
        &client, "127.0.0.1", (uint16_t)strtoul(argv[1], NULL, 10), 1000, 1000, cancelled, NULL);
    if (result != 0) {
        fprintf(stderr, "open errno=%d\n", result);
        return 1;
    }
    const unsigned count = (unsigned)strtoul(argv[3], NULL, 10);
    const unsigned width = 320, height = 240;
    uint8_t* pixels = malloc(width * height);
    uint8_t* jpeg = malloc(width * height * 4);
    if (pixels == NULL || jpeg == NULL)
        return 2;
    dart_jpeg_encoder_t encoder;
    (void)dart_jpeg_init(&encoder, 70);
    for (unsigned frame = 0; frame < count && result == 0; ++frame) {
        for (unsigned y = 0; y < height; ++y)
            for (unsigned x = 0; x < width; ++x)
                pixels[y * width + x] = (uint8_t)(x + y + frame * 3);
        size_t size = 0;
        if (dart_jpeg_encode(
                &encoder, pixels, width * height, width, height, width, width, height, jpeg,
                width * height * 4, &size)
            != 0)
            return 2;
        if (frame == 0 && argv[4][0] != '-') {
            FILE* file = fopen(argv[4], "wb");
            if (file == NULL || fwrite(jpeg, 1, size, file) != size || fclose(file) != 0)
                return 2;
        }
        result = dart_http_upload(&client, argv[2], jpeg, size, frame, frame * 66667);
        struct timespec pause = {.tv_nsec = 66666667};
        (void)nanosleep(&pause, NULL);
    }
    dart_http_close(&client);
    free(pixels);
    free(jpeg);
    if (result != 0)
        fprintf(stderr, "upload errno=%d\n", result);
    return result == 0 ? 0 : 1;
}
