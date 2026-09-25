/*
 * cli.c -- USB seri uzerinden yedek komut satiri.
 *
 * Sartname kablolu VE kablosuz erisim istiyor; ayrica WiFi ya da arayuz
 * coktugunde motoru durdurabilecek ikinci bir yol olmasi guvenlik meselesi.
 *
 * Komutlar (satir sonu ile):
 *   tara | durum | dur | estop | hatasil
 *   jog <az|el> <+|-> [sps]     jog <az|el> stop
 *   git <az|el> <derece>        ref <az|el>     serbest <az|el>
 *   gyro                        mag <on|off>
 *   stab <on|off> [az|el]       kilit
 *   log <on|off>                param <ad> <deger>
 */

#include "app_priv.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "hut_state.h"
#include "config.h"

static const char *TAG = "cli";

static int rol_coz(const char *s)
{
    if (!s) return -1;
    if (strcmp(s, "az") == 0) return ROL_AZ;
    if (strcmp(s, "el") == 0) return ROL_EL;
    return -1;
}

static void durum_bas(void)
{
    sistem_durum_t d;
    hut_durum_oku(&d);

    printf("mod=%d hata=%d surucu=%d imu=%d\n",
           (int)d.mod, (int)d.hata, d.surucu_sayisi, d.imu_var ? 1 : 0);
    printf("roll=%.2f pitch=%.2f yaw=%.2f kalibre=%d\n",
           d.durus.roll, d.durus.pitch, d.durus.yaw, d.durus.kalibre ? 1 : 0);
    for (int i = 0; i < d.surucu_sayisi; i++) {
        printf("  0x%02X rol=%s yon=%d adim=%ld aci=%.2f hiz=%.1f i2c_hata=%lu\n",
               d.eksen[i].adres, d.eksen[i].rol == ROL_AZ ? "az" : "el",
               d.eksen[i].yon, (long)d.eksen[i].adim, d.eksen[i].derece,
               d.eksen[i].sps, (unsigned long)d.eksen[i].i2c_hata);
    }
    printf("stab=%d hedef=%.2f olculen=%.2f hata=%.2f\n",
           d.stab_acik ? 1 : 0, d.stab_hedef, d.stab_olculen, d.stab_hata);
    printf("dongu=%lu us asim=%lu log=%d satir=%lu\n",
           (unsigned long)d.dongu_us, (unsigned long)d.asim,
           d.log_acik ? 1 : 0, (unsigned long)d.log_satir);
}

static void satir_isle(char *satir)
{
    char *kelime[5] = { 0 };
    int   n = 0;

    for (char *p = strtok(satir, " \t\r\n"); p && n < 5;
         p = strtok(NULL, " \t\r\n")) {
        kelime[n++] = p;
    }
    if (n == 0) {
        return;
    }

    komut_t k;
    memset(&k, 0, sizeof(k));
    bool gonder = true;

    if (strcmp(kelime[0], "tara") == 0) {
        k.tip = KOMUT_TARA;
    } else if (strcmp(kelime[0], "durum") == 0) {
        durum_bas();
        gonder = false;
    } else if (strcmp(kelime[0], "dur") == 0) {
        k.tip = KOMUT_DUR;
    } else if (strcmp(kelime[0], "estop") == 0) {
        k.tip = KOMUT_ACIL_DUR;
    } else if (strcmp(kelime[0], "hatasil") == 0) {
        k.tip = KOMUT_HATA_SIL;
    } else if (strcmp(kelime[0], "jog") == 0 && n >= 3) {
        if (strcmp(kelime[2], "stop") == 0) {
            k.tip = KOMUT_JOG_DUR;
            k.a   = rol_coz(kelime[1]);
        } else {
            k.tip = KOMUT_JOG;
            k.a   = rol_coz(kelime[1]);
            k.b   = (kelime[2][0] == '-') ? -1 : 1;
            k.c   = (n >= 4) ? strtof(kelime[3], NULL) : 0.0f;
        }
    } else if (strcmp(kelime[0], "git") == 0 && n >= 3) {
        k.tip = KOMUT_GIT;
        k.a   = rol_coz(kelime[1]);
        k.c   = strtof(kelime[2], NULL);
    } else if (strcmp(kelime[0], "ref") == 0 && n >= 2) {
        k.tip = KOMUT_REFERANS;
        k.a   = rol_coz(kelime[1]);
    } else if (strcmp(kelime[0], "serbest") == 0 && n >= 2) {
        k.tip = KOMUT_SERBEST;
        k.a   = rol_coz(kelime[1]);
    } else if (strcmp(kelime[0], "gyro") == 0) {
        k.tip = KOMUT_GYRO_KALIBRE;
    } else if (strcmp(kelime[0], "mag") == 0 && n >= 2) {
        k.tip = KOMUT_MAG_KALIBRE;
        k.a   = (strcmp(kelime[1], "on") == 0);
    } else if (strcmp(kelime[0], "stab") == 0 && n >= 2) {
        k.tip = KOMUT_STAB;
        k.a   = (strcmp(kelime[1], "on") == 0);
        k.b   = (n >= 3) ? rol_coz(kelime[2]) : -1;
    } else if (strcmp(kelime[0], "kilit") == 0) {
        k.tip = KOMUT_STAB_KILIT;
    } else if (strcmp(kelime[0], "log") == 0 && n >= 2) {
        k.tip = KOMUT_LOG;
        k.a   = (strcmp(kelime[1], "on") == 0);
    } else if (strcmp(kelime[0], "param") == 0 && n >= 3) {
        k.tip = KOMUT_PARAM;
        strncpy(k.anahtar, kelime[1], sizeof(k.anahtar) - 1);
        k.c = strtof(kelime[2], NULL);
    } else {
        printf("bilinmeyen komut: %s\n", kelime[0]);
        gonder = false;
    }

    if (gonder) {
        hut_komut_gonder(&k);
        printf("ok\n");
    }
}

static void cli_gorev(void *arg)
{
    (void)arg;
    char satir[96];

    ESP_LOGI(TAG, "seri komut satiri hazir ('durum' yazip deneyin)");

    for (;;) {
        if (fgets(satir, sizeof(satir), stdin) != NULL) {
            satir_isle(satir);
        } else {
            /* stdin bloklamiyor: bos donusleri bekleyerek yumusat. */
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

void cli_gorev_baslat(void)
{
    xTaskCreatePinnedToCore(cli_gorev, "cli", 4096, NULL, 3, NULL, 1);
}
