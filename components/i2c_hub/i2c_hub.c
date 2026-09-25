/*
 * i2c_hub.c -- aciklama icin i2c_hub.h.
 */

#include "i2c_hub.h"

#include <string.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

static const char *TAG = "i2c";

#define PROBE_TIMEOUT_MS  50
#define ISLEM_TIMEOUT_MS  100

static i2c_master_bus_handle_t s_bus[I2C_HUB_MAKS_HAT];
static int s_sda[I2C_HUB_MAKS_HAT] = { -1, -1 };
static int s_scl[I2C_HUB_MAKS_HAT] = { -1, -1 };

static bool hat_gecerli(int hat)
{
    return hat >= 0 && hat < I2C_HUB_MAKS_HAT && s_bus[hat] != NULL;
}

/*
 * Veri yolu kurulmadan ONCE hat seviyesini olc.
 *
 * Bu kontrol bring-up'in en pahali dersinden kaldi: ilk M20 kartinin SCL
 * hatti kart icinde GND'ye kisaydi ve gunlerce kodda hata arandi. Saglikli
 * hatta pull-up iki hatti da HIGH'a ceker.
 */
static void hat_seviyesi_kontrol(int hat, int sda_gpio, int scl_gpio)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << sda_gpio) | (1ULL << scl_gpio),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };

    if (gpio_config(&io) != ESP_OK) {
        return;
    }
    esp_rom_delay_us(200);

    int sda = gpio_get_level(sda_gpio);
    int scl = gpio_get_level(scl_gpio);

    if (sda && scl) {
        return;
    }
    ESP_LOGE(TAG, "hat %d SEVIYESI HATALI: SDA=%d SCL=%d (ikisi de 1 olmali)",
             hat, sda, scl);
    ESP_LOGE(TAG, "bu bir yazilim sorunu DEGIL. LOW okunan hat ya kopuk ya da");
    ESP_LOGE(TAG, "kart icinde GND'ye kisa. Multimetreyle olcun; bir cihaz");
    ESP_LOGE(TAG, "hatti kilitlemisse tek cozum gucu tamamen kesmek.");
}

esp_err_t i2c_hub_baslat(int hat, int sda_gpio, int scl_gpio)
{
    if (hat < 0 || hat >= I2C_HUB_MAKS_HAT) {
        return ESP_ERR_INVALID_ARG;
    }
    if (sda_gpio < 0 || scl_gpio < 0) {
        ESP_LOGI(TAG, "hat %d kapali (pin tanimli degil)", hat);
        return ESP_ERR_INVALID_ARG;
    }
    if (s_bus[hat]) {
        return ESP_OK;
    }

    s_sda[hat] = sda_gpio;
    s_scl[hat] = scl_gpio;

    hat_seviyesi_kontrol(hat, sda_gpio, scl_gpio);

    i2c_master_bus_config_t cfg = {
        .i2c_port          = -1,        /* IDF bos portu kendi secsin */
        .sda_io_num        = sda_gpio,
        .scl_io_num        = scl_gpio,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = true,
        },
    };

    esp_err_t hata = i2c_new_master_bus(&cfg, &s_bus[hat]);
    if (hata != ESP_OK) {
        ESP_LOGE(TAG, "hat %d kurulamadi: %s", hat, esp_err_to_name(hata));
        s_bus[hat] = NULL;
        return hata;
    }
    ESP_LOGI(TAG, "hat %d hazir (SDA=GPIO%d SCL=GPIO%d)", hat, sda_gpio, scl_gpio);
    return ESP_OK;
}

bool i2c_hub_hat_var(int hat)
{
    return hat_gecerli(hat);
}

i2c_master_bus_handle_t i2c_hub_bus(int hat)
{
    return hat_gecerli(hat) ? s_bus[hat] : NULL;
}

void i2c_hub_tara(int hat, i2c_tarama_t *sonuc)
{
    memset(sonuc, 0, sizeof(*sonuc));
    if (!hat_gecerli(hat)) {
        return;
    }

    for (uint8_t adres = 0x08; adres <= 0x77; adres++) {
        if (i2c_master_probe(s_bus[hat], adres, PROBE_TIMEOUT_MS) != ESP_OK) {
            continue;
        }
        if (sonuc->sayi < I2C_HUB_MAKS_CIHAZ) {
            sonuc->adres[sonuc->sayi++] = adres;
        }
        ESP_LOGI(TAG, "hat %d: cihaz 0x%02X", hat, adres);
    }
    if (sonuc->sayi == 0) {
        ESP_LOGW(TAG, "hat %d bos -- kablo, besleme ve GND'yi kontrol et", hat);
    }
}

bool i2c_hub_var_mi(int hat, uint8_t adres)
{
    if (!hat_gecerli(hat)) {
        return false;
    }
    return i2c_master_probe(s_bus[hat], adres, PROBE_TIMEOUT_MS) == ESP_OK;
}

esp_err_t i2c_hub_cihaz_ekle(int hat, uint8_t adres, uint32_t hiz_hz,
                             i2c_master_dev_handle_t *cikis)
{
    if (!hat_gecerli(hat)) {
        return ESP_ERR_INVALID_STATE;
    }
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = adres,
        .scl_speed_hz    = hiz_hz,
    };
    return i2c_master_bus_add_device(s_bus[hat], &cfg, cikis);
}

esp_err_t i2c_hub_kayit_yaz(i2c_master_dev_handle_t dev, uint8_t kayit,
                            uint8_t deger)
{
    uint8_t p[2] = { kayit, deger };
    return i2c_master_transmit(dev, p, 2, ISLEM_TIMEOUT_MS);
}

esp_err_t i2c_hub_kayit_oku(i2c_master_dev_handle_t dev, uint8_t kayit,
                            uint8_t *tampon, size_t uzunluk)
{
    /* ST sensorleri repeated START'i destekliyor; M20'nin aksine burada
       transmit_receive kullanmak guvenli ve daha hizli. */
    return i2c_master_transmit_receive(dev, &kayit, 1, tampon, uzunluk,
                                       ISLEM_TIMEOUT_MS);
}

esp_err_t i2c_hub_kurtar(int hat)
{
    if (!hat_gecerli(hat)) {
        return ESP_ERR_INVALID_STATE;
    }
    ESP_LOGW(TAG, "hat %d sifirlaniyor", hat);
    return i2c_master_bus_reset(s_bus[hat]);
}
