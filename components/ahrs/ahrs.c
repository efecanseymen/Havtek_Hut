/*
 * ahrs.c -- aciklama icin ahrs.h.
 */

#include "ahrs.h"
#include "config.h"

#include <math.h>
#include <string.h>
#include "esp_log.h"

static const char *TAG = "ahrs";

#define RAD2DER  57.29577951f

/* Gyro bias kalibrasyonu icin ornek sayisi: 100 Hz'de ~3 saniye. */
#define KALIBRE_ORNEK  300

static void kalman_baslat(kalman_eksen_t *k)
{
    memset(k, 0, sizeof(*k));
    k->P[0][0] = 1.0f;
    k->P[1][1] = 1.0f;
}

/*
 * Klasik iki durumlu Kalman: durum = (aci, gyro sapmasi).
 * Tahmin adimi gyro'yu integre eder, duzeltme adimi ivmeolcerden gelen aciyi
 * kullanir. Q_aci ve Q_bias modele, R_olcum sensore ne kadar guvendigimizi
 * soyler (KTR'de anlatilan Q ve R).
 */
static float kalman_guncelle(kalman_eksen_t *k, float olculen_aci,
                             float gyro_hizi, float dt)
{
    float Q_aci  = g_cfg.q_aci;
    float Q_bias = g_cfg.q_bias;
    float R      = g_cfg.r_olcum;

    /* --- tahmin --- */
    float hiz = gyro_hizi - k->bias;
    k->aci += dt * hiz;

    k->P[0][0] += dt * (dt * k->P[1][1] - k->P[0][1] - k->P[1][0] + Q_aci);
    k->P[0][1] -= dt * k->P[1][1];
    k->P[1][0] -= dt * k->P[1][1];
    k->P[1][1] += Q_bias * dt;

    /* --- duzeltme --- */
    float S = k->P[0][0] + R;
    float K0 = k->P[0][0] / S;
    float K1 = k->P[1][0] / S;

    float y = olculen_aci - k->aci;
    k->aci  += K0 * y;
    k->bias += K1 * y;

    float P00 = k->P[0][0], P01 = k->P[0][1];
    k->P[0][0] -= K0 * P00;
    k->P[0][1] -= K0 * P01;
    k->P[1][0] -= K1 * P00;
    k->P[1][1] -= K1 * P01;

    return k->aci;
}

void ahrs_baslat(ahrs_t *a)
{
    memset(a, 0, sizeof(*a));
    kalman_baslat(&a->roll_k);
    kalman_baslat(&a->pitch_k);
    a->ilk    = true;
    a->pusula = NAN;
}

void ahrs_kalibrasyon_basla(ahrs_t *a)
{
    a->kalibre_suruyor = true;
    a->kalibre_n       = 0;
    a->kalibre_toplam[0] = a->kalibre_toplam[1] = a->kalibre_toplam[2] = 0.0f;
    ESP_LOGI(TAG, "gyro kalibrasyonu basladi -- sistem HAREKETSIZ kalmali");
}

bool ahrs_kalibrasyon_bitti_mi(ahrs_t *a)
{
    if (a->kalibre_suruyor || a->kalibre_n < KALIBRE_ORNEK) {
        return false;
    }
    return true;
}

void ahrs_yaw_ayarla(ahrs_t *a, float derece)
{
    a->yaw = derece;
}

void ahrs_guncelle(ahrs_t *a, imu_orneklem_t *o, float dt_s)
{
    if (!o->gecerli) {
        return;
    }
    if (dt_s <= 0.0f || dt_s > 0.5f) {
        dt_s = 0.01f;         /* dilim kacmissa makul bir degere dus */
    }

    /* --- kalibrasyon: ham gyro'nun ortalamasi bias'tir --- */
    if (a->kalibre_suruyor) {
        for (int i = 0; i < 3; i++) {
            a->kalibre_toplam[i] += o->gyro[i];
        }
        a->kalibre_n++;

        if (a->kalibre_n >= KALIBRE_ORNEK) {
            for (int i = 0; i < 3; i++) {
                g_cfg.gyro_bias[i] = a->kalibre_toplam[i] / (float)a->kalibre_n;
            }
            g_cfg.gyro_kalibre = true;
            a->kalibre_suruyor = false;
            hut_cfg_kirlet();
            ESP_LOGI(TAG, "gyro bias: %.4f %.4f %.4f derece/s",
                     g_cfg.gyro_bias[0], g_cfg.gyro_bias[1], g_cfg.gyro_bias[2]);
        }
    }

    /* --- bias duzeltmesi --- */
    if (g_cfg.gyro_kalibre) {
        for (int i = 0; i < 3; i++) {
            o->gyro[i] -= g_cfg.gyro_bias[i];
        }
    }

    /* --- ivmeolcerden aci ---
       Sensor eksen yonelimi karta gore degisebilir; T-2 testinde isaretleri
       dogrulayip gerekirse burayi duzeltecegiz. */
    float ax = o->ivme[0], ay = o->ivme[1], az = o->ivme[2];
    float roll_olcum  = atan2f(ay, sqrtf(ax * ax + az * az)) * RAD2DER;
    float pitch_olcum = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD2DER;

    if (a->ilk) {
        a->roll_k.aci  = roll_olcum;
        a->pitch_k.aci = pitch_olcum;
        a->ilk = false;
    }

    a->roll  = kalman_guncelle(&a->roll_k,  roll_olcum,  o->gyro[0], dt_s);
    a->pitch = kalman_guncelle(&a->pitch_k, pitch_olcum, o->gyro[1], dt_s);

    /* --- yaw: sadece gyro. Kayiyor; olcmek istedigimiz sey de bu. --- */
    a->yaw += (o->gyro[2]) * dt_s;
    if (a->yaw >  180.0f) a->yaw -= 360.0f;
    if (a->yaw < -180.0f) a->yaw += 360.0f;

    /* --- pusula: sadece bilgi amacli, kontrole girmiyor --- */
    if (o->mag_var) {
        float mx = (o->mag[0] - g_cfg.mag_ofset[0]) * g_cfg.mag_olcek[0];
        float my = (o->mag[1] - g_cfg.mag_ofset[1]) * g_cfg.mag_olcek[1];
        float aci = atan2f(-my, mx) * RAD2DER + g_cfg.kuzey_ofset;

        if (aci < 0.0f)   aci += 360.0f;
        if (aci >= 360.0f) aci -= 360.0f;
        a->pusula = aci;
    }
}

void ahrs_durum(const ahrs_t *a, durus_t *d)
{
    d->roll    = a->roll;
    d->pitch   = a->pitch;
    d->yaw     = a->yaw;
    d->pusula  = a->pusula;
    d->kalibre = g_cfg.gyro_kalibre;
}
