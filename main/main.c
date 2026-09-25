/*
 * main.c -- HAVTEK Hareketli Uydu Terminali, v0.1
 *
 * Bu surumun isi stabilizasyonu "bitirmek" degil, onu tasarlamak icin gereken
 * olcumleri toplamak:
 *   - motorlari guvenle surmek (jog, aciya git, rampa, limit)
 *   - IMU'yu okumak ve 100 Hz'de loglamak
 *   - TEK eksende deneysel stabilizasyonu denemek (ayri panel)
 *   - her seyi tarayiciya akitip CSV olarak indirmek
 *
 * Acilis sirasi onemli: NVS -> ayarlar -> kuyruklar -> gercek zaman gorevi
 * (donanimi o aciyor) -> WiFi/arayuz -> seri komut satiri.
 */

#include "app_priv.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_system.h"

#include "config.h"
#include "hut_state.h"
#include "webui.h"

static const char *TAG = "havtek";

void app_main(void)
{
    ESP_LOGI(TAG, "HAVTEK HUT v%s baslatiliyor", HUT_SURUM);

    esp_err_t hata = nvs_flash_init();
    if (hata == ESP_ERR_NVS_NO_FREE_PAGES || hata == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        hata = nvs_flash_init();
    }
    ESP_ERROR_CHECK(hata);

    hut_cfg_yukle(&g_cfg);
    hut_state_baslat();

    /* Donanim Core 0'da: I2C hattinin tek sahibi bu gorev. */
    rt_gorev_baslat();

    /* Arayuz Core 1'de: WiFi gecikmesi kontrol dongusune bulasmasin. */
    ESP_ERROR_CHECK(wifi_ap_baslat());
    ESP_ERROR_CHECK(webui_baslat());

    cli_gorev_baslat();

    ESP_LOGI(TAG, "hazir. WiFi: %s / %s -> http://192.168.4.1",
             HUT_AP_SSID, HUT_AP_PAROLA);
}
