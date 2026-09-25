#pragma once
/*
 * config.h -- calisma zamani parametreleri ve varsayilanlari.
 *
 * Kural: hicbir ayar koda gomulu sabit olarak kullanilmaz. Hepsi bu yapida
 * durur, arayuzden degistirilir, NVS'e yazilir. 5V'tan 12V'a gecince tek
 * yapilacak sey degerleri guncellemek olsun diye boyle.
 */

#include "system_types.h"

/* ------------------------------------------------------------------- WiFi */

#define HUT_AP_SSID      "HAVTEK-HUT"
#define HUT_AP_PAROLA    "havtek2026"      /* en az 8 karakter olmali */
#define HUT_AP_KANAL     6
#define HUT_AP_MAKS_IST  4

/* --------------------------------------------------------------- mekanik */

/* M20 yarim adim tablosu: 400 adim = 1 tur -> 0.9 derece/adim (donanimda
   sayildi). Redüktör orani bunu antene indirger. */
#define MOTOR_DERECE_ADIM   0.9f

/* ------------------------------------------------------------ varsayilanlar */

#define VARS_REDUKTOR       3.6f     /* 10:36 disli -> 0.25 derece/adim      */
#define VARS_MAKS_SPS       300.0f   /* I2C tavani 500; 300 guvenli baslangic*/
#define VARS_IVME           600.0f   /* adim/s^2                             */
#define VARS_JOG_SPS        150.0f
#define VARS_BASLANGIC_SPS  60.0f    /* duran motorun kalkis (cekme) hizi    */

#define VARS_KP_MIN         2.0f     /* kucuk hatada yumusak                 */
#define VARS_KP_MAKS        12.0f    /* buyuk hatada agresif                 */
#define VARS_KI             0.5f
#define VARS_HATA_OLCEK     5.0f     /* Kp'nin doyuma gittigi hata (derece)  */
#define VARS_OLU_BANT       0.25f    /* bir adim: icinde hareket yok         */
#define VARS_ENTEGRAL_SINIR 50.0f

#define VARS_STAB_PENCERE   20.0f    /* referanstan +-20 derece calisma alani*/
#define VARS_STAB_HATA_SINIR 30.0f   /* asilirsa dur + hata                  */

#define VARS_AZ_LIMIT       360.0f   /* +-360 derece                         */
#define VARS_EL_MIN         0.0f
#define VARS_EL_MAKS        90.0f

/* Kalman (KTR'deki Q ve R). Deger secimi T-1/T-6 testlerinden gelecek. */
#define VARS_Q_ACI          0.001f
#define VARS_Q_BIAS         0.003f
#define VARS_R_OLCUM        0.03f

typedef struct {
    uint32_t sihir;             /* gecerlilik imzasi                        */
    uint16_t surum;             /* sema surumu                              */

    /* (hat, adres) -> rol ve yon. Slot = hat * 4 + (adres - ADR_MOTOR_ILK). */
    uint8_t  rol_map[HUT_MAKS_SLOT];
    int8_t   yon_map[HUT_MAKS_SLOT];
    bool     rol_atandi[HUT_MAKS_SLOT];

    float    reduktor[ROL_SAYISI];
    float    maks_sps;
    float    ivme;
    float    jog_sps;
    float    baslangic_sps;     /* rampanin basladigi hiz                   */
    bool     tutma_torku;       /* hareket bitince bobinler enerjili kalsin? */

    /* kontrolcu */
    bool     fuzzy;             /* false = sabit kazancli PI                */
    float    kp_min, kp_maks, ki, hata_olcek;
    float    olu_bant, entegral_sinir;
    float    stab_pencere, stab_hata_sinir;

    /* filtre */
    float    q_aci, q_bias, r_olcum;

    /* limitler (derece) */
    float    az_limit, el_min, el_maks;
    bool     limit_aktif;

    /* kalibrasyon */
    float    gyro_bias[3];
    bool     gyro_kalibre;
    float    mag_ofset[3];
    float    mag_olcek[3];
    bool     mag_kalibre;
    float    kuzey_ofset;       /* terminalin 0 ekseni kuzeyden kac derece  */

    uint32_t saglama;           /* en sonda: uzerindeki her seyin ozeti     */
} hut_cfg_t;

/* Varsayilanlari doldurur (NVS bos veya bozuksa). */
void hut_cfg_varsayilan(hut_cfg_t *c);

/* NVS'ten yukler; yoksa/bozuksa varsayilana doner ve false dondurur. */
bool hut_cfg_yukle(hut_cfg_t *c);

/* NVS'e yazar. Cagrilmasi pahali (flash), gercek zaman gorevinden CAGIRMA. */
bool hut_cfg_kaydet(const hut_cfg_t *c);

/* Isimle tek parametre degistirir. Arayuzdeki "param" komutu bunu kullanir.
   Bulunamayan anahtar icin false doner. */
bool hut_cfg_param_ayarla(hut_cfg_t *c, const char *anahtar, float deger);

/* Tum parametreleri JSON nesnesi olarak yazar (arayuz baslangicta okur). */
int hut_cfg_json(const hut_cfg_t *c, char *cikis, int boyut);

/* Uygulama boyunca tek ornek. Gercek zaman gorevi okur, servis gorevi yazar. */
extern hut_cfg_t g_cfg;

/* Bir sonraki NVS yazimini tetikler (servis gorevi periyodik bakar). */
void hut_cfg_kirlet(void);
bool hut_cfg_kirli_mi(void);
void hut_cfg_temizle(void);
