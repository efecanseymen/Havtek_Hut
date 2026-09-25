#pragma once
/*
 * hut_state.h -- gorevler arasi veri alisverisi.
 *
 * Uc kanal var, hepsi tek yonlu:
 *   komut  : web/CLI  -> gercek zaman gorevi   (kuyruk)
 *   durum  : gercek zaman gorevi -> herkes     (kilitli anlik goruntu)
 *   log    : gercek zaman gorevi -> telemetri  (halka tampon)
 *
 * Paylasilan global degisken yok; gercek zaman gorevi hicbir yerde bloke
 * olmuyor.
 */

#include "system_types.h"

void hut_state_baslat(void);

/* --- komut kuyrugu --- */
bool hut_komut_gonder(const komut_t *k);          /* web/CLI tarafi        */
bool hut_komut_al(komut_t *k);                    /* gercek zaman gorevi   */

/* --- durum anlik goruntusu --- */
void hut_durum_yaz(const sistem_durum_t *d);      /* gercek zaman gorevi   */
void hut_durum_oku(sistem_durum_t *d);            /* herkes                */

/* --- log halka tamponu --- */
void hut_log_yaz(const log_orneklem_t *o);        /* gercek zaman gorevi   */
int  hut_log_oku(log_orneklem_t *dizi, int maks); /* telemetri gorevi      */
void hut_log_temizle(void);
uint32_t hut_log_dusen(void);   /* tampon dolu diye atilan orneklem sayisi */
