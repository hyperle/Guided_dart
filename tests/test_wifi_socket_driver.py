"""Check the SDK socket fix against real POSIX nonblocking/EOF semantics."""

from pathlib import Path
import os
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SDK = Path(os.environ.get("K230_SDK_DIR", "/mnt/mydata/kRTOSSDK"))
SOURCE = SDK / "src/canmv/port/network/network_rt_smart.c"
PATCH = ROOT / "tools/wifi/network-socket-fix.patch"


@unittest.skipUnless(SOURCE.is_file() and shutil.which("cc") and shutil.which("patch"),
                     "K230 SDK, patch and native C compiler required")
class SocketDriverTest(unittest.TestCase):
    def test_nonblocking_empty_read_is_eagain_partial_data_is_returned_and_eof_terminates(self):
        with tempfile.TemporaryDirectory() as directory:
            work = Path(directory)
            target = work / SOURCE.name
            shutil.copy2(SOURCE, target)
            subprocess.run(["patch", "--batch", "--fuzz=0", "-p1", "-i", str(PATCH)],
                           cwd=work, check=True, capture_output=True)
            text = target.read_text()
            start = text.index("STATIC mp_uint_t network_rt_wlan_socket_recv(struct _mod_network_socket_obj_t *_socket, byte *buf, mp_uint_t len, int *_errno)\n{")
            end = text.index("STATIC mp_uint_t network_rt_wlan_socket_sendto(", start)
            callback = text[start:end]
            harness = r'''
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#define STATIC static
#define debug_printf(...) ((void)0)
typedef unsigned char byte;
typedef size_t mp_uint_t;
struct _mod_network_socket_obj_t { int fileno; int timeout; };
static int network_rt_wlan_socket_listening(struct _mod_network_socket_obj_t *s, int *e) { return 0; }
static int network_rt_wlan_socket_poll(struct _mod_network_socket_obj_t *s, uint32_t f, int *e) { return 0; }
#define MP_ENOTCONN ENOTCONN
'''+callback+r'''
int main(void) {
    int pair[2];
    if (socketpair(AF_UNIX, SOCK_STREAM, 0, pair)) return 1;
    if (fcntl(pair[0], F_SETFL, O_NONBLOCK)) return 2;
    struct _mod_network_socket_obj_t socket = { pair[0], 0 };
    byte buffer[512];
    int error = 0;
    mp_uint_t size = network_rt_wlan_socket_recv(&socket, buffer, sizeof buffer, &error);
    if (size != (mp_uint_t)-1 || (error != EAGAIN && error != EWOULDBLOCK)) return 3;
    if (write(pair[1], "ok", 2) != 2) return 4;
    size = network_rt_wlan_socket_recv(&socket, buffer, sizeof buffer, &error);
    if (size != 2 || memcmp(buffer, "ok", 2)) return 5;
    close(pair[1]);
    socket.timeout = -1;
    size = network_rt_wlan_socket_recv(&socket, buffer, sizeof buffer, &error);
    close(pair[0]);
    return size == 0 ? 0 : 6;
}
'''
            path = work / "socket_check.c"
            path.write_text(harness)
            binary = work / "socket_check"
            subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Wno-unused-parameter",
                            str(path), "-o", str(binary)], check=True, capture_output=True)
            subprocess.run([str(binary)], check=True, capture_output=True, timeout=3)


if __name__ == "__main__":
    unittest.main()
