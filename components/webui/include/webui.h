#pragma once
/*
 * webui.h -- SoftAP + HTTP + WebSocket arayuzu.
 *
 * Mimari: kart kendi WiFi agini yayinlar (router/internet gerekmez), arayuz
 * dosyalari flash'a gomulu olarak servis edilir, kontrol ve telemetri tek bir
 * WebSocket uzerinden akar.
 *
 * Gorev ayrimi onemli: bu tarafta HICBIR donanim islemi yok. Gelen komutlar
 * kuyruga birakilir, gercek zaman gorevi uygular. Boylece WiFi gecikmesi
 * kontrol dongusune bulasmaz.
 */

#include "esp_err.h"

esp_err_t wifi_ap_baslat(void);
esp_err_t webui_baslat(void);
