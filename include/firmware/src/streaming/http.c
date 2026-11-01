#define _POSIX_C_SOURCE 200809L
#include "streaming/http.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

static int64_t now_ms(void) {
    struct timespec now;
    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0)
        return -1;
    return (int64_t)now.tv_sec * 1000 + now.tv_nsec / 1000000;
}
static int ready(dart_http_client_t* client, short events, int64_t deadline) {
    for (;;) {
        if (client->cancelled != NULL && client->cancelled(client->context))
            return ECANCELED;
        const int64_t now = now_ms();
        if (now < 0)
            return EIO;
        const int64_t remaining = deadline - now;
        if (remaining <= 0)
            return ETIMEDOUT;
        struct pollfd descriptor = {.fd = client->socket, .events = events};
        const int rc = poll(&descriptor, 1, remaining < 100 ? (int)remaining : 100);
        if (rc < 0 && errno != EINTR)
            return errno;
        if (rc > 0) {
            if (descriptor.revents & events)
                return 0;
            if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL))
                return ECONNRESET;
        }
    }
}
void dart_http_close(dart_http_client_t* client) {
    if (client->socket >= 0)
        close(client->socket);
    client->socket = -1;
}
int dart_http_open(
    dart_http_client_t* client, const char* host, uint16_t port, uint32_t connect_timeout_ms,
    uint32_t io_timeout_ms, dart_http_cancelled_t cancelled, void* context) {
    if (client == NULL || host == NULL || strlen(host) >= sizeof(client->host) || port == 0
        || connect_timeout_ms == 0 || io_timeout_ms == 0)
        return EINVAL;
    dart_http_close(client);
    client->timeout_ms = io_timeout_ms;
    client->cancelled = cancelled;
    client->context = context;
    client->port = port;
    strcpy(client->host, host);
    struct sockaddr_in address = {.sin_family = AF_INET, .sin_port = htons(port)};
    if (inet_pton(AF_INET, host, &address.sin_addr) != 1)
        return EINVAL;
    client->socket = socket(AF_INET, SOCK_STREAM, 0);
    if (client->socket < 0)
        return errno;
    const int flags = fcntl(client->socket, F_GETFL, 0);
    const int no_delay = 1;
    int result = 0;
    if (flags < 0) {
        result = errno;
    } else if (
        setsockopt(client->socket, IPPROTO_TCP, TCP_NODELAY, &no_delay, sizeof(no_delay)) != 0) {
        result = errno;
    } else if (fcntl(client->socket, F_SETFL, flags | O_NONBLOCK) < 0) {
        result = errno;
    } else if (
        connect(client->socket, (struct sockaddr*)&address, sizeof(address)) < 0
        && errno != EISCONN) {
        if (errno != EINPROGRESS && errno != EALREADY && errno != EAGAIN) {
            result = errno;
        } else {
            result = ready(client, POLLOUT, now_ms() + connect_timeout_ms);
            if (result == 0) {
                int error = 0;
                socklen_t length = sizeof(error);
                if (getsockopt(client->socket, SOL_SOCKET, SO_ERROR, &error, &length) != 0)
                    result = errno;
                else
                    result = error;
            }
        }
    }
    if (result != 0)
        dart_http_close(client);
    return result;
}
static int send_all(dart_http_client_t* client, const void* data, size_t length, int64_t deadline) {
    const uint8_t* cursor = data;
    while (length != 0) {
        int result = ready(client, POLLOUT, deadline);
        if (result != 0)
            return result;
        const size_t chunk = length < 1024 ? length : 1024;
        const ssize_t sent = send(client->socket, cursor, chunk, MSG_NOSIGNAL);
        if (sent < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
                continue;
            return errno;
        }
        if (sent == 0)
            return ECONNRESET;
        cursor += sent;
        length -= (size_t)sent;
    }
    return 0;
}
static int
    receive(dart_http_client_t* client, void* data, size_t size, int64_t deadline, size_t* got) {
    for (;;) {
        const int result = ready(client, POLLIN, deadline);
        if (result != 0)
            return result;
        const ssize_t received = recv(client->socket, data, size, 0);
        if (received < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
                continue;
            return errno;
        }
        if (received == 0)
            return ECONNRESET;
        *got = (size_t)received;
        return 0;
    }
}
static int response(dart_http_client_t* client, int64_t deadline) {
    char buffer[4096];
    size_t used = 0, header_bytes = 0;
    while (header_bytes == 0) {
        if (used == sizeof(buffer) - 1)
            return EMSGSIZE;
        size_t got;
        const int result =
            receive(client, buffer + used, sizeof(buffer) - 1 - used, deadline, &got);
        if (result != 0)
            return result;
        used += got;
        buffer[used] = '\0';
        char* end = strstr(buffer, "\r\n\r\n");
        if (end != NULL)
            header_bytes = (size_t)(end - buffer) + 4;
    }
    unsigned status = 0;
    if (sscanf(buffer, "HTTP/1.1 %u", &status) != 1 || status < 200 || status >= 300)
        return EPROTO;
    size_t content_length = 0;
    int has_length = 0, close_connection = 0;
    char* line = strstr(buffer, "\r\n") + 2;
    while (line < buffer + header_bytes - 2) {
        char* end = strstr(line, "\r\n");
        *end = '\0';
        if (strncasecmp(line, "Content-Length:", 15) == 0) {
            char* tail;
            errno = 0;
            const unsigned long length = strtoul(line + 15, &tail, 10);
            if (has_length || errno != 0 || tail == line + 15 || *tail != '\0' || length > 4096)
                return EPROTO;
            content_length = length;
            has_length = 1;
        } else if (strncasecmp(line, "Transfer-Encoding:", 18) == 0) {
            return EPROTO;
        } else if (strcasecmp(line, "Connection: close") == 0) {
            close_connection = 1;
        }
        line = end + 2;
    }
    if ((!has_length && status != 204) || (status == 204 && content_length != 0)
        || used - header_bytes > content_length)
        return EPROTO;
    size_t remaining = content_length - (used - header_bytes);
    while (remaining != 0) {
        size_t got;
        const int result = receive(
            client, buffer, remaining < sizeof(buffer) ? remaining : sizeof(buffer), deadline,
            &got);
        if (result != 0)
            return result;
        remaining -= got;
    }
    if (close_connection)
        dart_http_close(client);
    return 0;
}
int dart_http_upload(
    dart_http_client_t* client, const char* token, const uint8_t* jpeg, size_t size,
    uint64_t sequence, uint64_t received_us) {
    if (client == NULL || client->socket < 0 || token == NULL || jpeg == NULL || size == 0)
        return EINVAL;
    if (strlen(token) > 128 || token[0] == '\0' || strpbrk(token, "\r\n") != NULL)
        return EINVAL;
    char header[768];
    const int length = snprintf(
        header, sizeof(header),
        "POST /api/frame HTTP/1.1\r\nHost: %s:%u\r\n"
        "Authorization: Bearer %s\r\nContent-Type: image/jpeg\r\nContent-Length: %zu\r\n"
        "X-Frame-Sequence: %" PRIu64 "\r\nX-Timestamp-Us: %" PRIu64
        "\r\nConnection: keep-alive\r\n\r\n",
        client->host, client->port, token, size, sequence, received_us);
    if (length < 0 || (size_t)length >= sizeof(header))
        return EINVAL;
    const int64_t deadline = now_ms() + client->timeout_ms;
    int result = send_all(client, header, (size_t)length, deadline);
    if (result == 0)
        result = send_all(client, jpeg, size, deadline);
    if (result == 0)
        result = response(client, deadline);
    if (result != 0)
        dart_http_close(client);
    return result;
}
