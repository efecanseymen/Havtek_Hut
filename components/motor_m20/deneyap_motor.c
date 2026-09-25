/*
 * deneyap_motor.c -- M20 protokol katmani. Aciklama icin deneyap_motor.h.
 *
 * efedeneme projesindeki dogrulanmis surumden alindi; tek fark veri yolunu
 * artik i2c_hub aciyor.
 */

#include "deneyap_motor.h"
#include "i2c_hub.h"

#include <string.h>
#include "esp_log.h"
#include "esp_rom_sys.h"

static const char *TAG = "m20";

#define I2C_TIMEOUT_MS  100

/* Mod gecis beklemesi. 2 ms donanimda dogrulandi (1 ms calismiyor).
   vTaskDelay degil mesgul bekleme: tick frekansindan bagimsiz olsun ve
   yalnizca hareket basinda 6 ms tutsun. */
#define MOD_BEKLEME_US  2000

enum {
    KOMUT_DRIVE_MOTOR1   = 0x00,
    KOMUT_DRIVE_MOTOR2   = 0x01,
    KOMUT_STEP_MOTOR     = 0x02,
    KOMUT_PWM_AYARI      = 0x03,
    KOMUT_STANDBY        = 0x04,
    KOMUT_MOD            = 0x05,
    KOMUT_ADRES_DEGISTIR = 0x06,
    KOMUT_FW_SURUMU      = 0x07,
    KOMUT_MOTOR_HATASI   = 0x08,
};

#define MOD_STEP  0x00

static esp_err_t paket_gonder(m20_t *m, uint8_t komut,
                              const uint8_t *veri, uint8_t boyut)
{
    uint8_t tampon[2 + 4];

    if (boyut > 4) {
        return ESP_ERR_INVALID_SIZE;
    }
    tampon[0] = komut;
    tampon[1] = boyut;
    if (boyut > 0) {
        memcpy(&tampon[2], veri, boyut);
    }
    return i2c_master_transmit(m->dev, tampon, (size_t)(2 + boyut),
                               I2C_TIMEOUT_MS);
}

static esp_err_t paket_oku(m20_t *m, uint8_t komut, uint8_t *cikis, size_t n)
{
    esp_err_t hata = i2c_master_transmit(m->dev, &komut, 1, I2C_TIMEOUT_MS);
    if (hata != ESP_OK) {
        return hata;
    }
    return i2c_master_receive(m->dev, cikis, n, I2C_TIMEOUT_MS);
}

esp_err_t m20_standby(m20_t *m, bool uyanik)
{
    /* Isim yaniltici: 0x01 = UYANIK, 0x00 = uyku (donanimda dogrulandi). */
    uint8_t v = uyanik ? 0x01 : 0x00;
    return paket_gonder(m, KOMUT_STANDBY, &v, 1);
}

static esp_err_t pwm_frekansi(m20_t *m, uint32_t hz)
{
    uint8_t v[4] = {
        (uint8_t)(hz), (uint8_t)(hz >> 8),
        (uint8_t)(hz >> 16), (uint8_t)(hz >> 24),
    };
    return paket_gonder(m, KOMUT_PWM_AYARI, v, 4);
}

esp_err_t m20_step_moduna_gec(m20_t *m)
{
    uint8_t mod = MOD_STEP;
    esp_err_t hata;

    hata = m20_standby(m, false);
    if (hata != ESP_OK) return hata;
    esp_rom_delay_us(MOD_BEKLEME_US);

    hata = paket_gonder(m, KOMUT_MOD, &mod, 1);
    if (hata != ESP_OK) return hata;
    esp_rom_delay_us(MOD_BEKLEME_US);

    hata = m20_standby(m, true);
    if (hata != ESP_OK) return hata;
    esp_rom_delay_us(MOD_BEKLEME_US);

    return ESP_OK;
}

esp_err_t m20_step_yaz(m20_t *m, bool in1a, bool in1b, bool in2a, bool in2b)
{
    uint8_t v[4] = { in1a, in1b, in2a, in2b };
    return paket_gonder(m, KOMUT_STEP_MOTOR, v, 4);
}

esp_err_t m20_hata_oku(m20_t *m, uint8_t *hata_bayragi)
{
    uint8_t v = 0;
    esp_err_t hata = paket_oku(m, KOMUT_MOTOR_HATASI, &v, 1);

    if (hata == ESP_OK && hata_bayragi) {
        *hata_bayragi = v;
    }
    return hata;
}

esp_err_t m20_baslat(m20_t *m, int hat, uint8_t adres)
{
    esp_err_t hata;

    memset(m, 0, sizeof(*m));
    m->adres = adres;
    m->hat   = hat;

    if (!i2c_hub_var_mi(hat, adres)) {
        ESP_LOGE(TAG, "hat %d, 0x%02X adresinde cevap yok", hat, adres);
        return ESP_ERR_NOT_FOUND;
    }

    hata = i2c_hub_cihaz_ekle(hat, adres, M20_I2C_HZ, &m->dev);
    if (hata != ESP_OK) {
        return hata;
    }

    /* Vendor begin() sirasi: PWM -> mod -> cikislari sifirla. */
    hata = pwm_frekansi(m, M20_PWM_HZ);
    if (hata != ESP_OK) return hata;

    hata = m20_step_moduna_gec(m);
    if (hata != ESP_OK) return hata;

    /* STEP modunda cikisi sifirlamanin dogru yolu STEP komutu; vendor burada
       DC komutu gonderiyor, calisan Arduino surumu ise step_yaz kullaniyor. */
    (void)m20_step_yaz(m, false, false, false, false);

    m->hazir = true;
    ESP_LOGI(TAG, "surucu hazir (hat %d, 0x%02X, STEP modu)", hat, adres);
    return ESP_OK;
}

void m20_bitir(m20_t *m)
{
    if (m->dev) {
        (void)m20_step_yaz(m, false, false, false, false);
        i2c_master_bus_rm_device(m->dev);
        m->dev = NULL;
    }
    m->hazir = false;
}
