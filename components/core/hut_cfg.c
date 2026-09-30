/*
 * hut_cfg.c -- parametrelerin varsayilanlari, NVS'e yazilmasi, isimle
 * degistirilmesi.
 */

#include "config.h"
#include "pins.h"

#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

static const char *TAG = "cfg";

#define CFG_SIHIR   0x48555431u    /* "HUT1" */
#define CFG_SURUM   5   /* v3: olcum_ekseni eklendi */
#define NVS_ALAN    "havtek"
#define NVS_ANAHTAR "cfg"

hut_cfg_t g_cfg;

static volatile bool s_kirli;

/* Sema degisince eski kayit sessizce yanlis yorumlanmasin diye basit bir
   ozet. Kriptografik olmasina gerek yok, sadece bozulmayi yakaliyor. */
static uint32_t saglama_hesapla(const hut_cfg_t *c)
{
    const uint8_t *p = (const uint8_t *)c;
    size_t n = offsetof(hut_cfg_t, saglama);
    uint32_t h = 2166136261u;

    for (size_t i = 0; i < n; i++) {
        h ^= p[i];
        h *= 16777619u;
    }
    return h;
}

void hut_cfg_varsayilan(hut_cfg_t *c)
{
    memset(c, 0, sizeof(*c));
    c->sihir = CFG_SIHIR;
    c->surum = CFG_SURUM;

    /* Varsayilan: hat 0'in ilk surucusu yatay, digerleri dikey. Kullanici
       arayuzden degistirir, "Eksen Tani" ise olcerek bulur. */
    for (int i = 0; i < HUT_MAKS_SLOT; i++) {
        c->rol_map[i]    = (i == 0) ? ROL_AZ : ROL_EL;
        c->yon_map[i]    = 1;
        c->rol_atandi[i] = false;
    }

    c->reduktor[ROL_AZ] = VARS_REDUKTOR;
    c->reduktor[ROL_EL] = VARS_REDUKTOR;
    c->maks_sps    = VARS_MAKS_SPS;
    c->ivme        = VARS_IVME;
    c->jog_sps       = VARS_JOG_SPS;
    c->jog_dps       = VARS_JOG_DPS;
    c->baslangic_sps = VARS_BASLANGIC_SPS;
    c->tutma_torku = true;

    c->fuzzy           = true;
    c->kp_min          = VARS_KP_MIN;
    c->kp_maks         = VARS_KP_MAKS;
    c->ki              = VARS_KI;
    c->hata_olcek      = VARS_HATA_OLCEK;
    c->olu_bant        = VARS_OLU_BANT;
    c->entegral_sinir  = VARS_ENTEGRAL_SINIR;
    c->stab_pencere    = VARS_STAB_PENCERE;
    c->stab_hata_sinir = VARS_STAB_HATA_SINIR;
    c->kilit_sinir_ms  = VARS_KILIT_SINIR_MS;
    c->mudahale_test   = VARS_MUDAHALE_TEST;

    c->olcum_ekseni[ROL_AZ] = OLCUM_YAW;
    c->olcum_ekseni[ROL_EL] = OLCUM_PITCH;

    c->q_aci   = VARS_Q_ACI;
    c->q_bias  = VARS_Q_BIAS;
    c->r_olcum = VARS_R_OLCUM;

    c->az_limit    = VARS_AZ_LIMIT;
    c->el_min      = VARS_EL_MIN;
    c->el_maks     = VARS_EL_MAKS;
    /* v0.1'de limitler KAPALI: tezgahta tek motorla deneme yapilacak ve
       referans noktasi henuz anlamli degil. Arayuzden acilabilir. */
    c->limit_aktif = false;

    /* Korumalar varsayilan olarak ACIK; ana anahtar kapali. Yani kutudan
       cikan davranis guvenli taraf. */
    c->guvenlik_kapali = false;
    c->hata_kapali     = false;
    c->pencere_aktif   = true;
    c->sapma_aktif     = true;
    c->ters_yon_aktif  = true;

    for (int i = 0; i < 3; i++) {
        c->mag_olcek[i] = 1.0f;
    }
    c->kuzey_ofset = 0.0f;
}

bool hut_cfg_yukle(hut_cfg_t *c)
{
    nvs_handle_t h;
    hut_cfg_t    gecici;
    size_t       boyut = sizeof(gecici);

    hut_cfg_varsayilan(c);

    if (nvs_open(NVS_ALAN, NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGI(TAG, "kayitli ayar yok, varsayilanlar kullaniliyor");
        return false;
    }

    esp_err_t hata = nvs_get_blob(h, NVS_ANAHTAR, &gecici, &boyut);
    nvs_close(h);

    if (hata != ESP_OK || boyut != sizeof(gecici)) {
        ESP_LOGW(TAG, "ayar okunamadi (%s), varsayilanlar", esp_err_to_name(hata));
        return false;
    }
    if (gecici.sihir != CFG_SIHIR || gecici.surum != CFG_SURUM) {
        ESP_LOGW(TAG, "ayar semasi eski/yabanci, varsayilanlar");
        return false;
    }
    if (gecici.saglama != saglama_hesapla(&gecici)) {
        ESP_LOGW(TAG, "ayar saglamasi tutmadi, varsayilanlar");
        return false;
    }

    *c = gecici;
    ESP_LOGI(TAG, "ayarlar NVS'ten yuklendi");
    return true;
}

bool hut_cfg_kaydet(const hut_cfg_t *c)
{
    nvs_handle_t h;
    hut_cfg_t    gecici = *c;

    gecici.sihir   = CFG_SIHIR;
    gecici.surum   = CFG_SURUM;
    gecici.saglama = saglama_hesapla(&gecici);

    if (nvs_open(NVS_ALAN, NVS_READWRITE, &h) != ESP_OK) {
        return false;
    }

    esp_err_t hata = nvs_set_blob(h, NVS_ANAHTAR, &gecici, sizeof(gecici));
    if (hata == ESP_OK) {
        hata = nvs_commit(h);
    }
    nvs_close(h);

    if (hata != ESP_OK) {
        ESP_LOGE(TAG, "ayar yazilamadi: %s", esp_err_to_name(hata));
        return false;
    }
    ESP_LOGI(TAG, "ayarlar kaydedildi");
    return true;
}

void hut_cfg_kirlet(void)   { s_kirli = true;  }
bool hut_cfg_kirli_mi(void) { return s_kirli;  }
void hut_cfg_temizle(void)  { s_kirli = false; }

/* ------------------------------------------------------- isimle parametre */

typedef struct {
    const char *ad;
    size_t      ofset;
    float       alt, ust;
} param_tanim_t;

#define P(ad_, alan_, alt_, ust_) \
    { ad_, offsetof(hut_cfg_t, alan_), alt_, ust_ }

static const param_tanim_t PARAMLAR[] = {
    P("maks_sps",        maks_sps,        1.0f,   500.0f),
    P("ivme",            ivme,            10.0f,  5000.0f),
    P("jog_sps",         jog_sps,         1.0f,   500.0f),
    P("jog_dps",         jog_dps,         0.1f,   200.0f),
    P("baslangic_sps",   baslangic_sps,   1.0f,   500.0f),
    P("kp_min",          kp_min,          0.0f,   200.0f),
    P("kp_maks",         kp_maks,         0.0f,   500.0f),
    P("ki",              ki,              0.0f,   100.0f),
    P("hata_olcek",      hata_olcek,      0.1f,   90.0f),
    P("olu_bant",        olu_bant,        0.0f,   5.0f),
    P("entegral_sinir",  entegral_sinir,  0.0f,   500.0f),
    P("stab_pencere",    stab_pencere,    1.0f,   180.0f),
    P("stab_hata_sinir", stab_hata_sinir, 1.0f,   180.0f),
    P("mudahale_test",   mudahale_test,   1.0f,   90.0f),
    P("q_aci",           q_aci,           0.0f,   1.0f),
    P("q_bias",          q_bias,          0.0f,   1.0f),
    P("r_olcum",         r_olcum,         0.0001f, 10.0f),
    P("az_limit",        az_limit,        1.0f,   720.0f),
    P("el_min",          el_min,          -180.0f, 180.0f),
    P("el_maks",         el_maks,         -180.0f, 180.0f),
    P("reduktor_az",     reduktor[ROL_AZ], 0.1f,  100.0f),
    P("reduktor_el",     reduktor[ROL_EL], 0.1f,  100.0f),
    P("kuzey_ofset",     kuzey_ofset,     -360.0f, 360.0f),
};

bool hut_cfg_param_ayarla(hut_cfg_t *c, const char *anahtar, float deger)
{
    for (size_t i = 0; i < sizeof(PARAMLAR) / sizeof(PARAMLAR[0]); i++) {
        if (strcmp(PARAMLAR[i].ad, anahtar) != 0) {
            continue;
        }
        if (isnan(deger)) {
            return false;
        }
        if (deger < PARAMLAR[i].alt) deger = PARAMLAR[i].alt;
        if (deger > PARAMLAR[i].ust) deger = PARAMLAR[i].ust;

        *(float *)((uint8_t *)c + PARAMLAR[i].ofset) = deger;
        hut_cfg_kirlet();
        return true;
    }

    /* Float olmayan birkac anahtar elle. */
    if (strcmp(anahtar, "kilit_sinir_ms") == 0) {
        if (deger < 500.0f)   deger = 500.0f;
        if (deger > 60000.0f) deger = 60000.0f;
        c->kilit_sinir_ms = (uint32_t)deger;
        hut_cfg_kirlet();
        return true;
    }
    if (strcmp(anahtar, "fuzzy") == 0) {
        c->fuzzy = (deger != 0.0f);
        hut_cfg_kirlet();
        return true;
    }
    if (strcmp(anahtar, "limit_aktif") == 0) {
        c->limit_aktif = (deger != 0.0f);
        hut_cfg_kirlet();
        return true;
    }
    if (strcmp(anahtar, "hata_kapali") == 0) {
        c->hata_kapali = (deger != 0.0f);
        hut_cfg_kirlet();
        return true;
    }
    if (strcmp(anahtar, "guvenlik_kapali") == 0) {
        c->guvenlik_kapali = (deger != 0.0f);
        hut_cfg_kirlet();
        return true;
    }
    if (strcmp(anahtar, "pencere_aktif") == 0) {
        c->pencere_aktif = (deger != 0.0f);
        hut_cfg_kirlet();
        return true;
    }
    if (strcmp(anahtar, "sapma_aktif") == 0) {
        c->sapma_aktif = (deger != 0.0f);
        hut_cfg_kirlet();
        return true;
    }
    if (strcmp(anahtar, "ters_yon_aktif") == 0) {
        c->ters_yon_aktif = (deger != 0.0f);
        hut_cfg_kirlet();
        return true;
    }
    if (strcmp(anahtar, "olcum_az") == 0) {
        c->olcum_ekseni[ROL_AZ] = (uint8_t)deger;
        hut_cfg_kirlet();
        return true;
    }
    if (strcmp(anahtar, "olcum_el") == 0) {
        c->olcum_ekseni[ROL_EL] = (uint8_t)deger;
        hut_cfg_kirlet();
        return true;
    }
    if (strcmp(anahtar, "tutma_torku") == 0) {
        c->tutma_torku = (deger != 0.0f);
        hut_cfg_kirlet();
        return true;
    }
    return false;
}

int hut_cfg_json(const hut_cfg_t *c, char *cikis, int boyut)
{
    return snprintf(cikis, boyut,
        "{\"surum\":\"%s\",\"maks_sps\":%.1f,\"ivme\":%.1f,\"jog_sps\":%.1f,"
        "\"baslangic_sps\":%.1f,\"jog_dps\":%.2f,"
        "\"fuzzy\":%d,\"kp_min\":%.2f,\"kp_maks\":%.2f,\"ki\":%.3f,"
        "\"hata_olcek\":%.2f,\"olu_bant\":%.2f,\"entegral_sinir\":%.1f,"
        "\"stab_pencere\":%.1f,\"stab_hata_sinir\":%.1f,"
        "\"kilit_sinir_ms\":%lu,\"mudahale_test\":%.1f,"
        "\"q_aci\":%.4f,\"q_bias\":%.4f,\"r_olcum\":%.4f,"
        "\"az_limit\":%.1f,\"el_min\":%.1f,\"el_maks\":%.1f,\"limit_aktif\":%d,"
        "\"guvenlik_kapali\":%d,\"pencere_aktif\":%d,\"sapma_aktif\":%d,"
        "\"ters_yon_aktif\":%d,\"hata_kapali\":%d,"
        "\"reduktor_az\":%.3f,\"reduktor_el\":%.3f,\"tutma_torku\":%d,"
        "\"kuzey_ofset\":%.1f,\"gyro_kalibre\":%d,\"mag_kalibre\":%d,"
        "\"olcum_az\":%d,\"olcum_el\":%d}",
        HUT_SURUM, c->maks_sps, c->ivme, c->jog_sps, c->baslangic_sps, c->jog_dps,
        c->fuzzy ? 1 : 0, c->kp_min, c->kp_maks, c->ki,
        c->hata_olcek, c->olu_bant, c->entegral_sinir,
        c->stab_pencere, c->stab_hata_sinir,
        (unsigned long)c->kilit_sinir_ms, c->mudahale_test,
        c->q_aci, c->q_bias, c->r_olcum,
        c->az_limit, c->el_min, c->el_maks, c->limit_aktif ? 1 : 0,
        c->guvenlik_kapali ? 1 : 0, c->pencere_aktif ? 1 : 0,
        c->sapma_aktif ? 1 : 0, c->ters_yon_aktif ? 1 : 0,
        c->hata_kapali ? 1 : 0,
        c->reduktor[ROL_AZ], c->reduktor[ROL_EL], c->tutma_torku ? 1 : 0,
        c->kuzey_ofset, c->gyro_kalibre ? 1 : 0, c->mag_kalibre ? 1 : 0,
        c->olcum_ekseni[ROL_AZ], c->olcum_ekseni[ROL_EL]);
}
