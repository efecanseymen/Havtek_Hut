#pragma once
/*
 * app_priv.h -- main bileseni icindeki gorevler arasi bildirimler.
 */

#include "esp_err.h"

/* Gercek zaman gorevi: I2C hattinin TEK sahibi. Core 0'da kosar. */
void rt_gorev_baslat(void);

/* USB seri uzerinden yedek komut satiri. WiFi coktugunde tek kurtarici. */
void cli_gorev_baslat(void);
