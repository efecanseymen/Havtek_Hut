/*
 * kontrol.c -- aciklama icin kontrol.h.
 */

#include "kontrol.h"
#include "config.h"
#include "eksen.h"

#include <math.h>
#include <string.h>

void kontrol_sifirla(kontrol_t *k)
{
    memset(k, 0, sizeof(*k));
}

float kontrol_son_kp(const kontrol_t *k)
{
    return k->son_kp;
}

float kontrol_hesapla(kontrol_t *k, float hata, float dt_s, float reduktor)
{
    float mutlak = fabsf(hata);
    float kp, cikti_der, cikti_sps, tavan;

    /* Olu bant: bir adimdan kucuk hatayi kovalamak titreme uretir. */
    if (mutlak < g_cfg.olu_bant) {
        /* Entegrali yavasca soner birak: hedefte beklerken birikmesin. */
        k->entegral *= 0.98f;
        k->son_kp    = 0.0f;
        k->son_cikti = 0.0f;
        return 0.0f;
    }

    if (g_cfg.fuzzy) {
        float olcek = g_cfg.hata_olcek > 0.01f ? g_cfg.hata_olcek : 1.0f;
        float oran  = mutlak / olcek;

        if (oran > 1.0f) {
            oran = 1.0f;
        }
        kp = g_cfg.kp_min + (g_cfg.kp_maks - g_cfg.kp_min) * oran;
    } else {
        kp = g_cfg.kp_maks;      /* karsilastirma icin sabit kazanc */
    }

    k->entegral += hata * dt_s;
    if (k->entegral >  g_cfg.entegral_sinir) k->entegral =  g_cfg.entegral_sinir;
    if (k->entegral < -g_cfg.entegral_sinir) k->entegral = -g_cfg.entegral_sinir;

    /* derece/s cinsinden istenen hiz */
    cikti_der = kp * hata + g_cfg.ki * k->entegral;

    /* derece/s -> adim/s */
    if (reduktor <= 0.0f) {
        reduktor = 1.0f;
    }
    cikti_sps = cikti_der / (EKSEN_MOTOR_DERECE_ADIM / reduktor);

    tavan = g_cfg.maks_sps;
    if (cikti_sps >  tavan) {
        cikti_sps =  tavan;
        /* Doyumdayken entegrali buyutmeye devam etmek windup demek. */
        k->entegral -= hata * dt_s;
    }
    if (cikti_sps < -tavan) {
        cikti_sps = -tavan;
        k->entegral -= hata * dt_s;
    }

    k->son_kp    = kp;
    k->son_cikti = cikti_sps;
    return cikti_sps;
}
