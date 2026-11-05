#ifndef DART_STREAMING_HTTP_H
#define DART_STREAMING_HTTP_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int (*dart_http_cancelled_t)(void* context);
typedef struct dart_http_client {
    int socket;
    uint32_t timeout_ms;
    dart_http_cancelled_t cancelled;
    void* context;
    char host[16];
    uint16_t port;
} dart_http_client_t;

/* Initialize socket=-1. Only the sender thread accesses this object. All calls
 * use monotonic deadlines and cancellation polling. Returns errno, or zero.
 * JPEG memory is borrowed through upload; a successful call consumed the full
 * response and confirms receipt, not persistence or successful video encoding.
 */
int dart_http_open(
    dart_http_client_t* client, const char* host, uint16_t port, uint32_t connect_timeout_ms,
    uint32_t io_timeout_ms, dart_http_cancelled_t cancelled, void* context);
int dart_http_upload(
    dart_http_client_t* client, const char* token, const uint8_t* jpeg, size_t size,
    uint64_t sequence, uint64_t received_us);
void dart_http_close(dart_http_client_t* client);

#ifdef __cplusplus
}
#endif
#endif
