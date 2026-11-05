#ifndef DART_STREAMING_K230_RADIO_H
#define DART_STREAMING_K230_RADIO_H

#include "streaming/http.h"

#ifdef __cplusplus
extern "C" {
#endif
/* Reuse the current STA association; choose SDIO only for a new association.
 * A different existing SSID returns EBUSY. Never disconnect another consumer,
 * change the shared default route/reconnect policy, or power down the radio.
 * Empty SSID requires an existing association. Pinmux belongs to the BSP.
 */
int dart_wifi_ensure(char* ssid, char* password, dart_http_cancelled_t cancelled, void* context);
#ifdef __cplusplus
}
#endif
#endif
