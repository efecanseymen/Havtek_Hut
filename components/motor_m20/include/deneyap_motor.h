#pragma once
/*
 * deneyap_motor.h -- Deneyap Cift Kanalli Motor Surucu (M20) protokol katmani.
 *
 * Kart uzerinde iki cip var:
 *   Toshiba TC78H660FTG : asil H-koprusu (2 kanal, 2.5-16V, kanal basina 2A)
 *   ST STM8S003F3       : I2C kolesi. Biz bununla konusuyoruz.
 *
 * ESP32'den STEP/DIR palsi URETILMIYOR. Her adim icin STM8'e bir I2C paketi
 * gidiyor (~1.26 ms @50 kHz) ve bobin durumlarini o suruyor. Pratik tavan
 * ~500 adim/s ve zamanlama jitter'li.
 *
 * Protokol (vendor kutuphanesinden sokuldu, 2026-09-01'de donanimda dogrulandi)
 *   Yazma : [komut][veriBoyutu][veri0..veriN]
 *   Okuma : once [komut] yaz + STOP, sonra ayri islemde oku. Repeated START
 *           DEGIL -- STM8 firmware'inin destekledigi belgeli degil.
 *
 * Bu surumde veri yolunu i2c_hub aciyor; burada sadece cihaz ekleniyor.
 */

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

/* ADR1/ADR2 lehim kopruleriyle degisir. Fabrika cikisi 0x16. */
#define M20_ADRES_VARSAYILAN  0x16
#define M20_ADRES_ALT1        0x17    /* ADR1 kisa */
#define M20_ADRES_ALT2        0x18    /* ADR2 kisa */
#define M20_ADRES_ALT3        0x19    /* ikisi de  */

#define M20_I2C_HZ            50000   /* vendor DYM disi kartlarda 50 kHz */
#define M20_PWM_HZ            500     /* vendor begin() varsayilani       */

typedef struct {
    i2c_master_dev_handle_t dev;
    int                     hat;      /* hangi I2C hatti (0 veya 1) */
    uint8_t                 adres;
    bool                    hazir;
} m20_t;

/* Hazir bir veri yoluna cihazi ekler, yoklar ve STEP moduna alir. */
esp_err_t m20_baslat(m20_t *m, int hat, uint8_t adres);
void      m20_bitir(m20_t *m);

/* TC78H660FTG hata bayragi (asiri akim / termal). 0 = sorun yok. */
esp_err_t m20_hata_oku(m20_t *m, uint8_t *hata);

esp_err_t m20_standby(m20_t *m, bool uyanik);

/*
 * Mod degisimi. TC78H660FTG modu, standby birakildiktan SONRAKI MODE pin
 * durumundan orneklenir; bu yuzden sira ve beklemeler onemli:
 *   standby(uyku) -> 2 ms -> mod -> 2 ms -> standby(uyanik) -> 2 ms
 * Bekleme 2 ms OLMALI; 1 ms ile STM8 modu her zaman orneklemiyor ve motor
 * hic donmuyor. Hareket oncesi tekrar cagirmak, surucunun uykuya kacmasina
 * karsi ucuz bir sigorta.
 */
esp_err_t m20_step_moduna_gec(m20_t *m);

/*
 * STEP modunda dort bobin girisini birlikte yazar. Sirali adim uretmek
 * eksen katmaninin isi. Dordu de false: bobinler serbest (tutma yok, isinma
 * yok).
 */
esp_err_t m20_step_yaz(m20_t *m, bool in1a, bool in1b, bool in2a, bool in2b);
