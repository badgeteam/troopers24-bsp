#pragma once

#include <stdbool.h>

#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "esp_wpa2.h"

// Secure option.
#define WIFI_MCH2022_SSID     "MCH2022"
#define WIFI_MCH2022_USER     "mch"
#define WIFI_MCH2022_IDENT    "mch"
#define WIFI_MCH2022_PASSWORD "mch"
#define WIFI_MCH2022_AUTH     WIFI_AUTH_WPA2_ENTERPRISE
#define WIFI_MCH2022_PHASE2   ESP_EAP_TTLS_PHASE2_PAP

/*
// Insecure option.
#define WIFI_MCH2022_SSID     "MCH2022-insecure"
#define WIFI_MCH2022_USER     ""
#define WIFI_MCH2022_IDENT    ""
#define WIFI_MCH2022_PASSWORD ""
#define WIFI_MCH2022_AUTH     WIFI_AUTH_OPEN
#define WIFI_MCH2022_PHASE2   0
*/

bool wifi_connect_to_stored();
void wifi_set_defaults();
void wifi_disconnect_and_disable();
