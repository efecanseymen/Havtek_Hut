#pragma once
/*
 * pins.h -- donanim baglantilari tek yerde.
 *
 * Deneyap Kart v2 (ESP32-S3). I2C hatti JST SH konnektorunden cikiyor;
 * konnektor sirasi SCL, SDA, 3V3, GND.
 */

/*
 * I2C hatti 0 -- ana hat. IMU, manyetometre ve birinci motor surucu burada.
 * Hiz cihaz basina ayarlaniyor (IDF i2c_master surucusu her islemde saati
 * yeniden kuruyor), yani M20 50 kHz'de IMU 400 kHz'de ayni hatta yasiyor.
 */
#define PIN_I2C_SDA        47      /* D10 */
#define PIN_I2C_SCL        21      /* D11 */

/*
 * I2C hatti 1 -- ikinci motor surucu icin.
 *
 * Iki M20 de fabrikadan 0x16 adresiyle geliyor ve ayni hatta takilamiyor.
 * Lehim koprusuyle birini 0x17 yapmak kalici cozum; ama ikinci surucuyu
 * AYRI bir hatta almak lehim gerektirmiyor ve adimlar iki hat arasinda
 * paylasildigi icin I2C yukunu de yariya boluyor.
 *
 * -1 yazarsaniz ikinci hat hic kurulmaz.
 */
#define PIN_I2C2_SDA       40
#define PIN_I2C2_SCL       39

/* Lazer modulu. Henuz baglanmadi: -1 = ozellik kapali. */
#define PIN_LAZER          -1

/* Fiziksel Baslat/Kapat tusu. Henuz baglanmadi: -1 = ozellik kapali.
   Bagladiginizda pini yazin; aktif-LOW varsayiliyor (dahili pull-up). */
#define PIN_BUTON          -1

/* ----------------------------------------------------------- I2C adresleri */

/* M20 motor surucu. Fabrika cikisi 0x16; ADR1 lehim koprusu 0x17 yapar. */
#define ADR_MOTOR_ILK      0x16
#define ADR_MOTOR_SON      0x19

/* LSM6DSM / LSM6DSL ivme+gyro. SDO pinine gore 0x6A veya 0x6B. */
#define ADR_IMU_A          0x6A
#define ADR_IMU_B          0x6B

/* Manyetometre. 2026-09-20 taramasinda kartta 0x30 gorunuyor -> MMC5603NJ.
   Digerleri yedek aday olarak duruyor (LIS2MDL 0x1E, LIS3MDL 0x1C/0x1E). */
#define ADR_MAG_MMC5603    0x30
#define ADR_MAG_LIS2MDL    0x1E
#define ADR_MAG_LIS3MDL_A  0x1C
#define ADR_MAG_LIS3MDL_B  0x1E
