#include <assert.h>
#include <errno.h>
#include <string.h>

#include "hal_netmgmt.h"
#include "streaming/k230/radio.h"

static int connected, selects, connects, dhcp, cancelled_now, cancel_on_connect, select_connected;
static char current_ssid[33];

int netmgmt_wlan_sta_isconnected(int* status) {
    *status = connected;
    return 0;
}
int netmgmt_wlan_select_device(enum netmgmt_wlan_device device, enum rt_netif_t itf) {
    assert(device == NETMGMT_WLAN_DEVICE_SDIO && itf == RT_NET_DEV_WLAN_STA);
    ++selects;
    connected = select_connected;
    return 0;
}
int netmgmt_wlan_sta_get_ap_info(struct rt_wlan_info_t* info) {
    memset(info, 0, sizeof(*info));
    info->ssid.len = strlen(current_ssid);
    memcpy(info->ssid.val, current_ssid, info->ssid.len);
    return 0;
}
int netmgmt_wlan_sta_connect_with_ssid(char* ssid, char* password) {
    assert(strcmp(password, "test-password") == 0);
    ++connects;
    connected = 1;
    cancelled_now = cancel_on_connect;
    strcpy(current_ssid, ssid);
    return 0;
}
int netmgmt_utils_set_ifconfig_dhcp(enum rt_netif_t itf) {
    assert(itf == RT_NET_DEV_WLAN_STA);
    ++dhcp;
    return 0;
}
int netmgmt_utils_get_ifconfig(enum rt_netif_t itf, struct ifconfig_t* config) {
    assert(itf == RT_NET_DEV_WLAN_STA);
    memset(config, 0, sizeof(*config));
    config->ip.addr = 0x01010101;
    return 0;
}
static int cancelled(void* context) {
    (void)context;
    return cancelled_now;
}
static void reset(void) {
    connected = selects = connects = dhcp = cancelled_now = cancel_on_connect = select_connected =
        0;
    strcpy(current_ssid, "existing");
}
int main(void) {
    char ssid[] = "existing", password[] = "test-password", empty[] = "", other[] = "other";
    reset();
    connected = 1;
    assert(dart_wifi_ensure(ssid, password, cancelled, NULL) == 0);
    assert(selects == 0 && connects == 0 && dhcp == 0);
    assert(dart_wifi_ensure(empty, password, cancelled, NULL) == 0);
    assert(selects == 0 && connects == 0 && dhcp == 0);
    assert(dart_wifi_ensure(other, password, cancelled, NULL) == EBUSY);
    assert(connected && strcmp(current_ssid, "existing") == 0);
    assert(selects == 0 && connects == 0 && dhcp == 0);
    reset();
    assert(dart_wifi_ensure(empty, password, cancelled, NULL) == ENOTCONN);
    assert(selects == 0 && connects == 0 && dhcp == 0);
    reset();
    assert(dart_wifi_ensure(ssid, password, cancelled, NULL) == 0);
    assert(selects == 1 && connects == 1 && dhcp == 1);
    reset();
    select_connected = 1;
    assert(dart_wifi_ensure(other, password, cancelled, NULL) == EBUSY);
    assert(selects == 1 && connects == 0 && dhcp == 0);
    reset();
    cancelled_now = 1;
    assert(dart_wifi_ensure(ssid, password, cancelled, NULL) == ECANCELED);
    assert(selects == 0 && connects == 0);
    reset();
    cancel_on_connect = 1;
    assert(dart_wifi_ensure(ssid, password, cancelled, NULL) == ECANCELED);
    assert(selects == 1 && connects == 1 && dhcp == 0);
    // No implementations of disconnect, static-IP reset, default route or
    // auto-reconnect mutation are supplied: an added call also fails linkage.
    return 0;
}
