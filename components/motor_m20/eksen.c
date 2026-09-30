/*
 * eksen.c -- aciklama icin eksen.h.
 */

#include "eksen.h"
#include "config.h"

#include <math.h>
#include <string.h>
#include "esp_log.h"

static const char *TAG = "eksen";

/*
 * Donanimda dogrulanmis TEK gecerli tablo: 8 durumlu yarim adim.
 * Satirlar (IN1A, IN1B, IN2A, IN2B).
 *
 * Bipolar mantikla turetilen 4 durumlu tam adim tablosu denendi ve YANLIS
 * cikti (motor titreyerek duzensiz donuyor) -- STM8 dort biti unipolar tarzi
 * esliyor. Bu tabloyu degistirmeyin.
 */
static const uint8_t DIZI[8][4] = {
    { 1, 0, 0, 0 },
    { 1, 1, 0, 0 },
    { 0, 1, 0, 0 },
    { 0, 1, 1, 0 },
    { 0, 0, 1, 0 },
    { 0, 0, 1, 1 },
    { 0, 0, 0, 1 },
    { 1, 0, 0, 1 },
};

#define DIZI_UZUNLUK  8

/* Art arda bu kadar I2C hatasindan sonra ekseni ariza say. */
#define I2C_HATA_ESIGI  20

esp_err_t eksen_baslat(eksen_t *e, int hat, uint8_t adres, uint8_t rol,
                       int8_t yon)
{
    memset(e, 0, sizeof(*e));
    e->hat   = hat;
    e->adres = adres;
    e->rol   = rol;
    e->yon   = (yon < 0) ? -1 : 1;

    esp_err_t hata = m20_baslat(&e->surucu, hat, adres);
    if (hata != ESP_OK) {
        return hata;
    }
    e->var = true;
    return ESP_OK;
}

void eksen_bitir(eksen_t *e)
{
    if (e->var) {
        m20_bitir(&e->surucu);
        e->var = false;
    }
}

static float sps_kirp(float sps)
{
    float tavan = g_cfg.maks_sps;

    if (tavan > EKSEN_MAKS_SPS_DONANIM) {
        tavan = EKSEN_MAKS_SPS_DONANIM;
    }
    if (sps >  tavan) return  tavan;
    if (sps < -tavan) return -tavan;
    return sps;
}

void eksen_hiz(eksen_t *e, float sps)
{
    e->git_aktif = false;
    e->hedef_sps = sps_kirp(sps);
}

void eksen_git(eksen_t *e, int32_t hedef_adim)
{
    e->git_aktif = true;
    e->git_hedef = hedef_adim;
}

void eksen_dur(eksen_t *e)
{
    e->git_aktif = false;
    e->hedef_sps = 0.0f;
}

void eksen_acil_dur(eksen_t *e)
{
    e->git_aktif = false;
    e->hedef_sps = 0.0f;
    e->anlik_sps = 0.0f;
}

void eksen_referans(eksen_t *e)
{
    e->adim      = 0;
    e->git_aktif = false;
}

esp_err_t eksen_serbest(eksen_t *e)
{
    if (!e->var) {
        return ESP_ERR_INVALID_STATE;
    }
    eksen_acil_dur(e);
    e->bobin_serbest = true;
    return m20_step_yaz(&e->surucu, false, false, false, false);
}

float eksen_derece(const eksen_t *e, float reduktor)
{
    if (reduktor <= 0.0f) {
        reduktor = 1.0f;
    }
    return (float)e->adim * (EKSEN_MOTOR_DERECE_ADIM / reduktor);
}

int32_t eksen_derece_adim(float derece, float reduktor)
{
    if (reduktor <= 0.0f) {
        reduktor = 1.0f;
    }
    return (int32_t)lroundf(derece / (EKSEN_MOTOR_DERECE_ADIM / reduktor));
}

/* Yazilim limiti: bu yonde bir adim daha atilabilir mi? */
static bool limit_izin_verir(const eksen_t *e, int8_t yon_isaret)
{
    if (!g_cfg.limit_aktif) {
        return true;
    }
    float reduktor = g_cfg.reduktor[e->rol < ROL_SAYISI ? e->rol : 0];
    float derece   = eksen_derece(e, reduktor);
    float adim_der = EKSEN_MOTOR_DERECE_ADIM / reduktor;
    float sonraki  = derece + (float)yon_isaret * adim_der * (float)e->yon;

    if (e->rol == ROL_AZ) {
        return fabsf(sonraki) <= g_cfg.az_limit;
    }
    return sonraki >= g_cfg.el_min && sonraki <= g_cfg.el_maks;
}

/*
 * GIT modunda hedefe carpmadan durabilmek icin gereken hiz tavani.
 * Sabit ivmede v = sqrt(2*a*kalan_adim).
 */
static float git_hedef_hizi(const eksen_t *e)
{
    int32_t kalan = e->git_hedef - e->adim;
    float   mutlak;

    if (kalan == 0) {
        return 0.0f;
    }
    mutlak = sqrtf(2.0f * g_cfg.ivme * (float)labs(kalan));
    if (mutlak > g_cfg.maks_sps) {
        mutlak = g_cfg.maks_sps;
    }
    /* Son adimlarda hiz sifira cok yaklasip hareket durmasin. */
    if (mutlak < g_cfg.baslangic_sps) {
        mutlak = g_cfg.baslangic_sps;
    }
    return (kalan > 0) ? mutlak : -mutlak;
}

static esp_err_t durum_yaz(eksen_t *e)
{
    return m20_step_yaz(&e->surucu,
                        DIZI[e->dizi][0], DIZI[e->dizi][1],
                        DIZI[e->dizi][2], DIZI[e->dizi][3]);
}

bool eksen_servis(eksen_t *e, int64_t simdi_us, float dt_s)
{
    float hedef, fark, mutlak;
    int8_t isaret;

    if (!e->var) {
        return false;
    }

    hedef = e->git_aktif ? sps_kirp(git_hedef_hizi(e)) : e->hedef_sps;

    /* GIT modunda hedefe varinca isi biter. */
    if (e->git_aktif && e->adim == e->git_hedef) {
        e->git_aktif = false;
        e->hedef_sps = 0.0f;
        e->anlik_sps = 0.0f;
        return false;
    }

    /* Ivme rampasi. Step motorda hiz sicramasi = adim kacirma; kacirdigini
       da fark edemiyoruz, o yuzden rampa istege bagli degil. */
    fark = hedef - e->anlik_sps;
    float adim_degisim = g_cfg.ivme * dt_s;

    if (fabsf(fark) <= adim_degisim) {
        e->anlik_sps = hedef;
    } else {
        e->anlik_sps += (fark > 0.0f) ? adim_degisim : -adim_degisim;
    }

    /*
     * Duran motordan kalkis: sifirdan rampalamak yerine "cekme hizi"ndan
     * basla, yoksa ilk adimin periyodu saniyeler surer.
     *
     * DIKKAT: bu zemin ISTENEN hizi asmamali. Onceki surum kosulsuz
     * baslangic_sps'e yukseltiyordu ve sonucu su oluyordu: 60'in altindaki
     * hicbir hiz komutu calismiyor, "10 adim/s" isteyince motor 60'ta
     * doniyordu. Yavas hassas hareket (limit ogretme, ince hizalama) bu
     * yuzden imkansizdi.
     */
    if (fabsf(e->anlik_sps) < 0.5f && fabsf(hedef) > 0.01f) {
        float kalkis = g_cfg.baslangic_sps;

        if (kalkis > fabsf(hedef)) {
            kalkis = fabsf(hedef);      /* istenen hizi asma */
        }
        e->anlik_sps  = (hedef > 0.0f) ? kalkis : -kalkis;
        e->sonraki_us = simdi_us;       /* ilk adimi bekletme */
    }

    mutlak = fabsf(e->anlik_sps);
    if (mutlak < 1.0f) {
        e->anlik_sps  = 0.0f;
        e->sonraki_us = simdi_us;
        return false;
    }

    if (simdi_us < e->sonraki_us) {
        return false;
    }

    isaret = (e->anlik_sps > 0.0f) ? 1 : -1;

    if (!limit_izin_verir(e, isaret)) {
        e->limit_vurdu = true;
        e->anlik_sps   = 0.0f;
        e->hedef_sps   = 0.0f;
        e->git_aktif   = false;
        return false;
    }
    e->limit_vurdu = false;

    /* Surucu araya uykuya kacmis olabilir: duran motor yeniden kalkarken
       mod kurulumunu tekrarla (3 paket, ~6 ms). Her adimda degil, sadece
       bobinler serbest birakilmissa. */
    if (e->bobin_serbest) {
        if (m20_step_moduna_gec(&e->surucu) == ESP_OK) {
            e->bobin_serbest = false;
        }
    }

    /* Tabloda bir ileri/geri. Motor yonu ile aci yonu ayri: e->yon isareti
       burada uygulaniyor, konum sayaci HER ZAMAN aci yonunde artiyor. */
    int8_t motor_isaret = (int8_t)(isaret * e->yon);
    if (motor_isaret > 0) {
        e->dizi = (uint8_t)((e->dizi + 1) % DIZI_UZUNLUK);
    } else {
        e->dizi = (uint8_t)((e->dizi + DIZI_UZUNLUK - 1) % DIZI_UZUNLUK);
    }

    esp_err_t hata = durum_yaz(e);
    if (hata != ESP_OK) {
        e->i2c_hata++;
        if (e->i2c_hata == I2C_HATA_ESIGI) {
            ESP_LOGE(TAG, "0x%02X: art arda %d I2C hatasi", e->adres,
                     I2C_HATA_ESIGI);
        }
        /* Hata alinan adimi saymiyoruz: bobin durumu degismedi. */
        e->dizi = (uint8_t)((e->dizi + DIZI_UZUNLUK - motor_isaret) % DIZI_UZUNLUK);
        e->sonraki_us = simdi_us + 2000;
        return false;
    }

    e->i2c_hata = 0;
    e->adim    += isaret;

    uint32_t periyot_us = (uint32_t)(1000000.0f / mutlak);
    if (periyot_us < 2000u) {
        periyot_us = 2000u;      /* 500 adim/s donanim tavani */
    }
    e->sonraki_us = simdi_us + periyot_us;
    return true;
}
