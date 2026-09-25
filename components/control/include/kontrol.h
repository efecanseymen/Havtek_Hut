#pragma once
/*
 * kontrol.h -- Fuzzy-PI (ve karsilastirma icin sabit kazancli PI).
 *
 * Girdi : aci hatasi (derece)
 * Cikti : adim/s cinsinden ISARETLI hiz komutu
 *
 * Neden hiz: her 10 ms'de "su kadar derece git" demek motoru surekli dur-kalk
 * ettirir ve titretir. Ust katman aci hedefini belirler, kontrolcu hiz uretir,
 * eksen katmani rampayla uygular.
 *
 * Fuzzy kismi: kazanc hatanin buyuklugune gore kayiyor.
 *   |hata| buyuk  -> kp_maks'a yakin  (agresif toparlama)
 *   |hata| kucuk  -> kp_min'e yakin   (asim ve titreme yok)
 * Uyelik fonksiyonu yerine dogrusal gecis kullaniliyor: bu donanimda kural
 * tablosunun getirdigi ek karmasikligi hakli cikaracak bir fark yok, ve
 * parametre sayisi az oldugu icin tezgahta ayarlamasi kolay.
 *
 * Olu bant: mekanik cozunurluk 0.25 derece. Bundan kucuk hatayi duzeltmeye
 * calismak motorun hedefte bir ileri bir geri titremesi demektir.
 */

#include <stdbool.h>

typedef struct {
    float entegral;
    float son_kp;
    float son_cikti;
} kontrol_t;

void kontrol_sifirla(kontrol_t *k);

/* Hata (derece) -> hiz komutu (adim/s). reduktor, dereceyi adima cevirir. */
float kontrol_hesapla(kontrol_t *k, float hata, float dt_s, float reduktor);

float kontrol_son_kp(const kontrol_t *k);
