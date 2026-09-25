#pragma once
/*
 * imu.h -- Deneyap 9 Eksen Ataletsel Olcum Birimi surucusu.
 *
 * ONEMLI: karttaki cipler henuz donanimda DOGRULANMADI. KTR'de LSM6DSM
 * yaziyor; manyetometrenin hangi cip oldugu belirsiz. Bu yuzden surucu
 * "tarayip tani" mantigiyla yazildi:
 *   - 0x6A / 0x6B adresinde WHO_AM_I okur, LSM6DSM/DSL/DSO ailesini tanir.
 *   - Manyetometre icin LIS2MDL (0x1E) ve LIS3MDL (0x1C/0x1E) dener.
 *   - Tanimadigini bulursa adresi ve WHO_AM_I degerini arayuze basar; o zaman
 *     dogru sureyi buna gore ekleriz.
 *
 * Hiz: IMU 400 kHz'de konusuyor. M20 ayni hatta 50 kHz'de; IDF i2c_master
 * saati cihaz basina ayarladigi icin bu sorun degil.
 */

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "system_types.h"

typedef struct {
    bool     var;
    uint8_t  adres;
    uint8_t  who_am_i;
    char     tip[12];          /* "LSM6DSM" vb.                            */

    bool     mag_var;
    uint8_t  mag_adres;
    uint8_t  mag_who;
    char     mag_tip[12];

    float    ivme_olcek;       /* LSB -> g                                  */
    float    gyro_olcek;       /* LSB -> derece/s                           */
    float    mag_olcek;        /* LSB -> uT                                 */
} imu_bilgi_t;

/* Hatti tarar, bulduğu cipi yapilandirir. IMU yoksa ESP_ERR_NOT_FOUND. */
esp_err_t imu_baslat(void);

const imu_bilgi_t *imu_bilgi(void);

/*
 * Tek olcum. gyro/ivme HAM fiziksel birimde doner (bias duzeltmesi
 * UYGULANMAZ; onu ahrs katmani yapar). Manyetometre varsa okunur.
 */
esp_err_t imu_oku(imu_orneklem_t *o);
