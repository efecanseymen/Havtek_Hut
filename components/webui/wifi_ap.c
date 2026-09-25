/*
 * wifi_ap.c -- SoftAP. Sabit IP 192.168.4.1 (IDF varsayilani).
 */

#include "webui.h"
#include "config.h"

#include <string.h>
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_log.h"

static const char *TAG = "wifi";

static void olay(void *arg, esp_event_base_t taban, int32_t id, void *veri)
{
    (void)arg; (void)taban;

    if (id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *e = veri;
        ESP_LOGI(TAG, "istemci baglandi, aid=%d", e->aid);
    } else if (id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *e = veri;
        ESP_LOGI(TAG, "istemci ayrildi, aid=%d", e->aid);
    }
}

esp_err_t wifi_ap_baslat(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                        ESP_EVENT_ANY_ID, &olay, NULL, NULL));

    wifi_config_t cfg = { 0 };
    strncpy((char *)cfg.ap.ssid, HUT_AP_SSID, sizeof(cfg.ap.ssid) - 1);
    strncpy((char *)cfg.ap.password, HUT_AP_PAROLA, sizeof(cfg.ap.password) - 1);
    cfg.ap.ssid_len       = strlen(HUT_AP_SSID);
    cfg.ap.channel        = HUT_AP_KANAL;
    cfg.ap.max_connection = HUT_AP_MAKS_IST;
    cfg.ap.authmode       = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &cfg));

    /* Guc tasarrufu kapali: telemetri akisinda gecikme istemiyoruz. */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "AP hazir: SSID=%s parola=%s -> http://192.168.4.1",
             HUT_AP_SSID, HUT_AP_PAROLA);
    return ESP_OK;
}
