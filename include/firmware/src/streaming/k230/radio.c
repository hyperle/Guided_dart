#define _POSIX_C_SOURCE 200809L
#include "streaming/k230/radio.h"

#include <errno.h>
#include <string.h>
#include <time.h>

#include "hal_netmgmt.h"

static int matches(char* ssid) {
    if (ssid[0] == '\0')
        return 0;
    struct rt_wlan_info_t info = {0};
    if (netmgmt_wlan_sta_get_ap_info(&info) != 0)
        return EIO;
    const size_t length = strlen(ssid);
    return info.ssid.len == (int)length && memcmp(info.ssid.val, ssid, length) == 0 ? 0 : EBUSY;
}

int dart_wifi_ensure(char* ssid, char* password, dart_http_cancelled_t cancelled, void* context) {
    if (cancelled(context))
        return ECANCELED;
    int connected = 0;
    if (netmgmt_wlan_sta_isconnected(&connected) != 0)
        return ENODEV;
    if (!connected) {
        if (ssid[0] == '\0')
            return ENOTCONN;
        if (netmgmt_wlan_select_device(NETMGMT_WLAN_DEVICE_SDIO, RT_NET_DEV_WLAN_STA) != 0)
            return ENODEV;
        // Selection may find an existing connection. Do not replace another
        // consumer's association, or change shared default-route/reconnect policy.
        if (cancelled(context))
            return ECANCELED;
        if (netmgmt_wlan_sta_isconnected(&connected) != 0)
            return EIO;
        if (!connected) {
            if (netmgmt_wlan_sta_connect_with_ssid(ssid, password) != 0)
                return ECONNREFUSED;
            if (cancelled(context))
                return ECANCELED;
            if (netmgmt_utils_set_ifconfig_dhcp(RT_NET_DEV_WLAN_STA) != 0)
                return EIO;
        }
    }
    struct timespec begin;
    if (clock_gettime(CLOCK_MONOTONIC, &begin) != 0)
        return EIO;
    for (;;) {
        if (cancelled(context))
            return ECANCELED;
        struct ifconfig_t config = {0};
        connected = 0;
        if (netmgmt_wlan_sta_isconnected(&connected) == 0 && connected) {
            const int result = matches(ssid);
            if (result != 0)
                return result;
            if (netmgmt_utils_get_ifconfig(RT_NET_DEV_WLAN_STA, &config) == 0 && config.ip.addr != 0
                && config.ip.addr != UINT32_MAX)
                return 0;
        }
        struct timespec now;
        if (clock_gettime(CLOCK_MONOTONIC, &now) != 0)
            return EIO;
        if (now.tv_sec - begin.tv_sec >= 20)
            return ETIMEDOUT;
        struct timespec pause = {.tv_nsec = 100000000};
        (void)nanosleep(&pause, NULL);
    }
}
