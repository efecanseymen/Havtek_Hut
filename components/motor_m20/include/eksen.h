#pragma once
/*
 * eksen.h -- bir ekseni suren blokemeyen katman.
 *
 * Neden blokemeyen: efedeneme'deki step_motor_adim() hareket bitene kadar
 * bloke ediyordu. Kontrol dongusu 100 Hz'de karar verecekse adim uretimi
 * dongunun icinde bloke edemez. Burada her cagrida EN FAZLA BIR adim atiliyor;
 * cagiran 1 ms'lik izgarada donuyor.
 *
 * Sorumluluk siniri: hedef hiz (adim/s, isaretli) disaridan verilir. Bu katman
 * ivme rampasini, adim zamanlamasini, bobin dizisini, konum sayacini ve
 * yazilim limitlerini yonetir. Aci hesabi ve kontrolcu burada DEGIL.
 */

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "deneyap_motor.h"

/* Yarim adim tablosu: 400 adim = 1 tur (donanimda sayildi) -> 0.9 derece. */
#define EKSEN_MOTOR_DERECE_ADIM  0.9f

/* I2C tavani: adim basina ~1.26 ms paket + STM8 isleme. */
#define EKSEN_MAKS_SPS_DONANIM   500.0f

typedef struct {
    m20_t   surucu;
    bool    var;
    int     hat;          /* hangi I2C hattinda                           */
    uint8_t adres;
    uint8_t rol;          /* ROL_AZ / ROL_EL                              */
    int8_t  yon;          /* motorun + yonu acinin + yonu mu (+1/-1)      */

    uint8_t dizi;         /* 8 durumluk yarim adim tablosunda neredeyiz   */
    int32_t adim;         /* referanstan beri net adim                    */

    float   hedef_sps;    /* istenen hiz, isaretli                        */
    float   anlik_sps;    /* rampadan gecmis gercek hiz                   */
    int64_t sonraki_us;   /* bir sonraki adimin zamani                    */

    bool    git_aktif;    /* belirli bir adima gidiliyor mu               */
    int32_t git_hedef;

    bool    limit_vurdu;
    bool    bobin_serbest;
    uint32_t i2c_hata;    /* art arda hata sayaci                         */
} eksen_t;

/* Surucuyu acar ve ekseni sifirlar. */
esp_err_t eksen_baslat(eksen_t *e, int hat, uint8_t adres, uint8_t rol,
                       int8_t yon);
void      eksen_bitir(eksen_t *e);

/* Hedef hiz (adim/s, isaretli). Rampa bu hiza kendi ulasir. */
void eksen_hiz(eksen_t *e, float sps);

/* Belirli bir adima git: mesafeye gore yavaslar ve hedefte durur. */
void eksen_git(eksen_t *e, int32_t hedef_adim);

/* Rampayla dur. */
void eksen_dur(eksen_t *e);

/* Aninda dur (acil durdurma). Adim kacirabilir; sadece acil durumda. */
void eksen_acil_dur(eksen_t *e);

/*
 * 1 ms izgarada cagrilir. En fazla bir adim atar.
 *   simdi_us : esp_timer_get_time()
 *   dt_s     : son cagridan beri gecen sure
 * Doner: adim atildiysa true.
 */
bool eksen_servis(eksen_t *e, int64_t simdi_us, float dt_s);

/* Konumu sifir kabul et (referanslama). Motoru hareket ettirmez. */
void eksen_referans(eksen_t *e);

/* Bobinleri birak: tutma torku ve isinma gider, konum bilgisi guvenilmez olur. */
esp_err_t eksen_serbest(eksen_t *e);

/* Adim <-> derece. Redüktör orani cfg'den gelir. */
float eksen_derece(const eksen_t *e, float reduktor);
int32_t eksen_derece_adim(float derece, float reduktor);
