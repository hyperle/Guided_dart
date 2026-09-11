/*
 * LushanPi RT-Smart boot launcher.
 *
 * The official smart_ipc.elf owns VICAP/VENC and provides the H.264 RTSP
 * stream. This launcher only brings up WLAN, waits for DHCP, then replaces
 * itself with smart_ipc.elf. Telnet, when enabled by the firmware, remains
 * the remote control entry point.
 */
#include "hal_netmgmt.h"

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>

namespace {
constexpr char kDefaultSsid[] = "CHANGE_ME_SSID";
constexpr char kDefaultPassword[] = "CHANGE_ME_PASSWORD";
constexpr char kSmartIpc[] =
    "/sdcard/app/examples/integrated_poc/smart_ipc/smart_ipc.elf";

void usage(const char *name)
{
    std::printf("Usage: %s [ssid] [password] [smart_ipc options...]\n", name);
    std::printf("Defaults are compiled into the image; change them before build.\n");
}

bool wait_for_sta(int timeout_seconds)
{
    for (int i = 0; i < timeout_seconds; ++i) {
        int connected = 0;
        if (netmgmt_wlan_sta_isconnected(&connected) == 0 && connected)
            return true;
        sleep(1);
    }
    return false;
}

} // namespace

int main(int argc, char **argv)
{
    if (argc > 1 && (!std::strcmp(argv[1], "-h") || !std::strcmp(argv[1], "--help"))) {
        usage(argv[0]);
        return 0;
    }

    char ssid[RT_WLAN_SSID_MAX_LENGTH + 1];
    char password[RT_WLAN_PASSWORD_MAX_LENGTH + 1];
    std::snprintf(ssid, sizeof(ssid), "%s", argc > 1 ? argv[1] : kDefaultSsid);
    std::snprintf(password, sizeof(password), "%s",
                  argc > 2 ? argv[2] : kDefaultPassword);

    std::printf("lushan_stream: selecting WLAN radio\n");
    if (netmgmt_wlan_select_device(NETMGMT_WLAN_DEVICE_AUTO,
                                   RT_NET_DEV_WLAN_STA) != 0) {
        std::fprintf(stderr, "WLAN radio is unavailable\n");
        return 2;
    }
    netmgmt_wlan_sta_set_auto_reconnect(1);

    std::printf("lushan_stream: connecting to SSID '%s'\n", ssid);
    if (netmgmt_wlan_sta_connect_with_ssid(ssid, password) != 0) {
        std::fprintf(stderr, "WLAN connect request failed\n");
        return 3;
    }
    if (!wait_for_sta(30)) {
        std::fprintf(stderr, "WLAN association timed out\n");
        return 4;
    }

    if (netmgmt_utils_set_ifconfig_dhcp(RT_NET_DEV_WLAN_STA) != 0) {
        std::fprintf(stderr, "DHCP request failed\n");
        return 5;
    }
    sleep(2);

    struct ifconfig_t cfg = {};
    if (netmgmt_utils_get_ifconfig(RT_NET_DEV_WLAN_STA, &cfg) == 0)
        std::printf("lushan_stream: WLAN IP 0x%08x\n", cfg.ip.addr);

    /* Keep the official encoder/RTSP implementation as the media owner. */
    const char *ipc = argc > 3 ? argv[3] : kSmartIpc;
    char *const default_args[] = {
        const_cast<char *>(ipc),
        const_cast<char *>("-G"), const_cast<char *>("1"),
        const_cast<char *>("-M"), const_cast<char *>("0"),
        const_cast<char *>("-t"), const_cast<char *>("h264"),
        const_cast<char *>("-E"), const_cast<char *>("0"),
        const_cast<char *>("-F"), const_cast<char *>("0"),
        const_cast<char *>("-D"), const_cast<char *>("0"), nullptr};

    std::printf("lushan_stream: starting RTSP encoder %s\n", ipc);
    execv(ipc, default_args);
    std::fprintf(stderr, "execv(%s) failed: %s\n", ipc, std::strerror(errno));
    return 6;
}
