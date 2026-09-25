/*
 * imu.c -- aciklama icin imu.h.
 */

#include "imu.h"
#include "i2c_hub.h"
#include "pins.h"

#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "imu";

#define IMU_I2C_HZ  400000

/* --- LSM6DS ailesi kayitlari --- */
#define LSM_WHO_AM_I   0x0F
#define LSM_CTRL1_XL   0x10
#define LSM_CTRL2_G    0x11
#define LSM_CTRL3_C    0x12
#define LSM_OUTX_L_G   0x22   /* 12 bayt: gyro XYZ + ivme XYZ */

#define WHO_LSM6DSM    0x6A
#define WHO_LSM6DSL    0x6A
#define WHO_LSM6DSO    0x6C
#define WHO_LSM6DSR    0x6B
#define WHO_LSM6DSV    0x70

/* --- LIS2MDL --- */
#define LIS2MDL_WHO        0x4F
#define LIS2MDL_WHO_DEGER  0x40
#define LIS2MDL_CFG_A      0x60
#define LIS2MDL_OUTX_L     0x68

/* --- MMC5603NJ (Memsic). Deneyap 9 eksen kartinda 0x30'da gorulen cihaz. --- */
#define MMC_URUN_ID        0x39
#define MMC_URUN_DEGER     0x10
#define MMC_XOUT0          0x00   /* 6 bayt, MSB once, ISARETSIZ (merkez 32768) */
#define MMC_ODR            0x1A
#define MMC_CTRL0          0x1B
#define MMC_CTRL1          0x1C
#define MMC_CTRL2          0x1D

/* --- LIS3MDL --- */
#define LIS3MDL_WHO        0x0F
#define LIS3MDL_WHO_DEGER  0x3D
#define LIS3MDL_CTRL1      0x20
#define LIS3MDL_CTRL2      0x21
#define LIS3MDL_CTRL3      0x22
#define LIS3MDL_CTRL4      0x23
#define LIS3MDL_OUTX_L     0x28

typedef enum { MAG_YOK = 0, MAG_LIS2MDL, MAG_LIS3MDL, MAG_MMC5603 } mag_tip_t;

static imu_bilgi_t s_bilgi;
static mag_tip_t   s_mag_tip;
static i2c_master_dev_handle_t s_dev;
static i2c_master_dev_handle_t s_mag_dev;

static int16_t le16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/* ------------------------------------------------------------ ivme + gyro */

static esp_err_t lsm_yapilandir(void)
{
    esp_err_t hata;

    /* CTRL3_C: BDU (veri butunlugu) + otomatik adres artirma. Bu olmadan
       12 baytlik blok okumasi yarim guncellenmis veri dondurebilir. */
    hata = i2c_hub_kayit_yaz(s_dev, LSM_CTRL3_C, 0x44);
    if (hata != ESP_OK) return hata;

    /* CTRL1_XL = 0x48 : 104 Hz, +-4g.
       Kontrol dongumuz 100 Hz; sensoru 104 Hz'e almak en yakin ust komsu. */
    hata = i2c_hub_kayit_yaz(s_dev, LSM_CTRL1_XL, 0x48);
    if (hata != ESP_OK) return hata;

    /* CTRL2_G = 0x44 : 104 Hz, +-500 dps.
       Platform hizlari cok dusuk (~5 derece/s); dar olcek = daha iyi cozunurluk. */
    hata = i2c_hub_kayit_yaz(s_dev, LSM_CTRL2_G, 0x44);
    if (hata != ESP_OK) return hata;

    s_bilgi.ivme_olcek = 0.122f / 1000.0f;   /* +-4g  : 0.122 mg/LSB  */
    s_bilgi.gyro_olcek = 17.50f / 1000.0f;   /* 500dps: 17.5 mdps/LSB */
    return ESP_OK;
}

static bool lsm_tani(uint8_t adres)
{
    i2c_master_dev_handle_t dev;
    uint8_t who = 0;

    if (!i2c_hub_var_mi(0, adres)) {
        return false;
    }
    if (i2c_hub_cihaz_ekle(0, adres, IMU_I2C_HZ, &dev) != ESP_OK) {
        return false;
    }
    if (i2c_hub_kayit_oku(dev, LSM_WHO_AM_I, &who, 1) != ESP_OK) {
        i2c_master_bus_rm_device(dev);
        return false;
    }

    s_dev = dev;
    s_bilgi.adres    = adres;
    s_bilgi.who_am_i = who;

    switch (who) {
    case WHO_LSM6DSM: strcpy(s_bilgi.tip, "LSM6DSM/DSL"); break;
    case WHO_LSM6DSO: strcpy(s_bilgi.tip, "LSM6DSO");     break;
    case WHO_LSM6DSR: strcpy(s_bilgi.tip, "LSM6DSR");     break;
    case WHO_LSM6DSV: strcpy(s_bilgi.tip, "LSM6DSV");     break;
    default:          strcpy(s_bilgi.tip, "BILINMEYEN");  break;
    }

    ESP_LOGI(TAG, "0x%02X: WHO_AM_I=0x%02X (%s)", adres, who, s_bilgi.tip);

    if (strcmp(s_bilgi.tip, "BILINMEYEN") == 0) {
        ESP_LOGW(TAG, "bu cip taninmadi. Kayit haritasi farkli olabilir --");
        ESP_LOGW(TAG, "WHO_AM_I degerini not alip surucuyu ona gore guncelleyin.");
    }
    return true;
}

/* ------------------------------------------------------------ manyetometre */

/* MMC5603: surekli olcum modu. Sirasi onemli -- once ODR, sonra Cmm_freq_en,
   en son Cmm_en; ters sirada cip tek atim modunda kaliyor. */
static bool mmc_kur(i2c_master_dev_handle_t dev)
{
    bool ok = true;

    ok &= i2c_hub_kayit_yaz(dev, MMC_ODR,   100) == ESP_OK;  /* 100 Hz       */
    ok &= i2c_hub_kayit_yaz(dev, MMC_CTRL0, 0xA0) == ESP_OK;  /* cmm_freq+asr */
    ok &= i2c_hub_kayit_yaz(dev, MMC_CTRL1, 0x00) == ESP_OK;  /* en genis BW  */
    ok &= i2c_hub_kayit_yaz(dev, MMC_CTRL2, 0x10) == ESP_OK;  /* cmm_en       */
    return ok;
}

static bool mag_tani(void)
{
    const uint8_t adaylar[] = { ADR_MAG_MMC5603, ADR_MAG_LIS2MDL,
                                ADR_MAG_LIS3MDL_A };

    for (size_t i = 0; i < sizeof(adaylar); i++) {
        uint8_t adres = adaylar[i];
        i2c_master_dev_handle_t dev;
        uint8_t who = 0;

        if (!i2c_hub_var_mi(0, adres)) {
            continue;
        }
        if (i2c_hub_cihaz_ekle(0, adres, IMU_I2C_HZ, &dev) != ESP_OK) {
            continue;
        }

        /* MMC5603: urun kimligi 0x39 = 0x10 */
        if (i2c_hub_kayit_oku(dev, MMC_URUN_ID, &who, 1) == ESP_OK &&
            who == MMC_URUN_DEGER && mmc_kur(dev)) {
            s_mag_dev  = dev;
            s_mag_tip  = MAG_MMC5603;
            s_bilgi.mag_var   = true;
            s_bilgi.mag_adres = adres;
            s_bilgi.mag_who   = who;
            /* 16 bit modda 0.0625 mG/LSB = 0.00625 uT/LSB */
            s_bilgi.mag_olcek = 0.00625f;
            strcpy(s_bilgi.mag_tip, "MMC5603");
            ESP_LOGI(TAG, "manyetometre: MMC5603 @0x%02X", adres);
            return true;
        }

        /* LIS2MDL: WHO 0x4F = 0x40 */
        if (i2c_hub_kayit_oku(dev, LIS2MDL_WHO, &who, 1) == ESP_OK &&
            who == LIS2MDL_WHO_DEGER) {
            /* 0x8C: sicaklik telafisi acik, 100 Hz, surekli mod */
            if (i2c_hub_kayit_yaz(dev, LIS2MDL_CFG_A, 0x8C) == ESP_OK) {
                s_mag_dev  = dev;
                s_mag_tip  = MAG_LIS2MDL;
                s_bilgi.mag_var   = true;
                s_bilgi.mag_adres = adres;
                s_bilgi.mag_who   = who;
                s_bilgi.mag_olcek = 0.15f;      /* 1.5 mgauss = 0.15 uT/LSB */
                strcpy(s_bilgi.mag_tip, "LIS2MDL");
                ESP_LOGI(TAG, "manyetometre: LIS2MDL @0x%02X", adres);
                return true;
            }
        }

        /* LIS3MDL: WHO 0x0F = 0x3D */
        if (i2c_hub_kayit_oku(dev, LIS3MDL_WHO, &who, 1) == ESP_OK &&
            who == LIS3MDL_WHO_DEGER) {
            bool ok = true;
            ok &= i2c_hub_kayit_yaz(dev, LIS3MDL_CTRL1, 0x70) == ESP_OK; /* UHP, 10Hz */
            ok &= i2c_hub_kayit_yaz(dev, LIS3MDL_CTRL2, 0x00) == ESP_OK; /* +-4 gauss */
            ok &= i2c_hub_kayit_yaz(dev, LIS3MDL_CTRL3, 0x00) == ESP_OK; /* surekli  */
            ok &= i2c_hub_kayit_yaz(dev, LIS3MDL_CTRL4, 0x0C) == ESP_OK; /* Z UHP    */
            if (ok) {
                s_mag_dev  = dev;
                s_mag_tip  = MAG_LIS3MDL;
                s_bilgi.mag_var   = true;
                s_bilgi.mag_adres = adres;
                s_bilgi.mag_who   = who;
                s_bilgi.mag_olcek = 100.0f / 6842.0f;  /* +-4 gauss -> uT */
                strcpy(s_bilgi.mag_tip, "LIS3MDL");
                ESP_LOGI(TAG, "manyetometre: LIS3MDL @0x%02X", adres);
                return true;
            }
        }

        ESP_LOGW(TAG, "0x%02X cihaz var ama manyetometre olarak taninmadi", adres);
        i2c_master_bus_rm_device(dev);
    }

    ESP_LOGW(TAG, "manyetometre bulunamadi -- kuzey ofsetini elle girin");
    return false;
}

/* ------------------------------------------------------------------ genel */

esp_err_t imu_baslat(void)
{
    memset(&s_bilgi, 0, sizeof(s_bilgi));
    s_mag_tip = MAG_YOK;

    if (!lsm_tani(ADR_IMU_A) && !lsm_tani(ADR_IMU_B)) {
        ESP_LOGE(TAG, "IMU bulunamadi (0x%02X / 0x%02X)", ADR_IMU_A, ADR_IMU_B);
        return ESP_ERR_NOT_FOUND;
    }

    esp_err_t hata = lsm_yapilandir();
    if (hata != ESP_OK) {
        ESP_LOGE(TAG, "IMU yapilandirilamadi: %s", esp_err_to_name(hata));
        return hata;
    }

    s_bilgi.var = true;
    (void)mag_tani();
    return ESP_OK;
}

const imu_bilgi_t *imu_bilgi(void)
{
    return &s_bilgi;
}

static esp_err_t mag_oku(float *cikis)
{
    uint8_t ham[6];
    uint8_t kayit;

    if (s_mag_tip == MAG_LIS2MDL) {
        kayit = LIS2MDL_OUTX_L;
    } else if (s_mag_tip == MAG_LIS3MDL) {
        kayit = LIS3MDL_OUTX_L | 0x80;   /* otomatik adres artirma */
    } else if (s_mag_tip == MAG_MMC5603) {
        kayit = MMC_XOUT0;
    } else {
        return ESP_ERR_NOT_SUPPORTED;
    }

    esp_err_t hata = i2c_hub_kayit_oku(s_mag_dev, kayit, ham, 6);
    if (hata != ESP_OK) {
        return hata;
    }

    if (s_mag_tip == MAG_MMC5603) {
        /* ST cipleri little-endian ve isaretli; MMC big-endian ve ISARETSIZ,
           sifir noktasi 32768. Iki formati karistirmak sessizce sacma aci
           uretir, o yuzden ayri yol. */
        for (int i = 0; i < 3; i++) {
            uint16_t ham16 = (uint16_t)((ham[i * 2] << 8) | ham[i * 2 + 1]);
            cikis[i] = ((float)ham16 - 32768.0f) * s_bilgi.mag_olcek;
        }
    } else {
        for (int i = 0; i < 3; i++) {
            cikis[i] = (float)le16(&ham[i * 2]) * s_bilgi.mag_olcek;
        }
    }
    return ESP_OK;
}

esp_err_t imu_oku(imu_orneklem_t *o)
{
    uint8_t ham[12];

    memset(o, 0, sizeof(*o));
    o->t_us = esp_timer_get_time();

    if (!s_bilgi.var) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Tek blok: gyro XYZ + ivme XYZ. Iki ayri okuma yapmak hem hatti iki kat
       mesgul eder hem de iki ornegi farkli anlara denk getirir. */
    esp_err_t hata = i2c_hub_kayit_oku(s_dev, LSM_OUTX_L_G, ham, sizeof(ham));
    if (hata != ESP_OK) {
        return hata;
    }

    for (int i = 0; i < 3; i++) {
        o->gyro[i] = (float)le16(&ham[i * 2])       * s_bilgi.gyro_olcek;
        o->ivme[i] = (float)le16(&ham[6 + i * 2])   * s_bilgi.ivme_olcek;
    }

    if (s_bilgi.mag_var && mag_oku(o->mag) == ESP_OK) {
        o->mag_var = true;
    }

    o->gecerli = true;
    return ESP_OK;
}
