#pragma once
/*
 * i2c_hub.h -- I2C veri yollarinin sahibi.
 *
 * Iki hat destekleniyor:
 *   hat 0 : ana hat (IMU + manyetometre + birinci motor surucu)
 *   hat 1 : istege bagli ikinci hat (yalnizca ikinci motor surucu)
 *
 * Neden iki hat: iki M20 de fabrikadan 0x16 adresiyle geliyor ve ayni hatta
 * takilamiyor. Ikinci hat, lehim yapmadan iki motoru birlikte kullanmayi
 * saglar. Adimlar iki hatta bolundugu icin zamanlama da rahatlar.
 *
 * Neden tek yerden: ayni pinlerde iki kez i2c_new_master_bus cagirmak
 * catisiyor. Bus'lar burada bir kez aciliyor, herkes buradan cihaz ekliyor.
 *
 * Ayrica: bir STM8 takilip SCL'yi LOW'da birakirsa o hat tamamen kilitleniyor
 * ve yazilim reset'i kurtarmiyor (GPS'te yasandi). i2c_hub_kurtar() en azindan
 * veri yolunu sifirlamayi deniyor.
 */

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#define I2C_HUB_MAKS_HAT    2
#define I2C_HUB_MAKS_CIHAZ  16

typedef struct {
    uint8_t adres[I2C_HUB_MAKS_CIHAZ];
    uint8_t sayi;
} i2c_tarama_t;

/* Hat seviyesini olcer, veri yolunu kurar. Her hat icin bir kez cagrilir.
   sda/scl negatifse hat kurulmaz ve ESP_ERR_INVALID_ARG doner. */
esp_err_t i2c_hub_baslat(int hat, int sda_gpio, int scl_gpio);

bool i2c_hub_hat_var(int hat);
i2c_master_bus_handle_t i2c_hub_bus(int hat);

/* 0x08..0x77 arasini tarar. Yavas (~100 ms), gercek zaman dongusunde cagirma. */
void i2c_hub_tara(int hat, i2c_tarama_t *sonuc);

bool i2c_hub_var_mi(int hat, uint8_t adres);

/* Basit kayit tabanli cihazlar (IMU, manyetometre) icin yardimcilar.
   M20 kendi protokolunu kullandigi icin bunlari KULLANMAZ. */
esp_err_t i2c_hub_cihaz_ekle(int hat, uint8_t adres, uint32_t hiz_hz,
                             i2c_master_dev_handle_t *cikis);
esp_err_t i2c_hub_kayit_yaz(i2c_master_dev_handle_t dev, uint8_t kayit,
                            uint8_t deger);
esp_err_t i2c_hub_kayit_oku(i2c_master_dev_handle_t dev, uint8_t kayit,
                            uint8_t *tampon, size_t uzunluk);

/* Hat kilitlenmesinden sonra son care. Basarisiz olursa tek cozum guc kesmek. */
esp_err_t i2c_hub_kurtar(int hat);
