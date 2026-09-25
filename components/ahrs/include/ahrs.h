#pragma once
/*
 * ahrs.h -- ham IMU verisinden aci kestirimi.
 *
 * Roll ve pitch: iki durumlu (aci + gyro sapmasi) Kalman filtresi. Yercekimi
 * mutlak referans oldugu icin bu iki aci KAYMAZ.
 *
 * Yaw: yercekimi yaw hakkinda bir sey soylemez, geriye sadece gyro integrali
 * kalir ve bu KAYAR. Kalibre edilmis bias'tan sonra bile 5 dakikada birkac
 * derece birikebilir; v0.1'in isi tam olarak bu kaymayi olcmek. Manyetometre
 * varsa yaw'i duzeltmek yerine yalnizca ACILIS kuzey referansi icin
 * kullaniyoruz: motor miknatislari ve celik kafes yuzunden hareket halindeki
 * manyetometre olcumune guvenmiyoruz.
 */

#include <stdint.h>
#include <stdbool.h>
#include "system_types.h"

typedef struct {
    float aci;          /* derece      */
    float bias;         /* derece/s    */
    float P[2][2];      /* hata kovaryansi */
} kalman_eksen_t;

typedef struct {
    kalman_eksen_t roll_k, pitch_k;
    float roll, pitch, yaw;
    float pusula;
    bool  ilk;

    /* gyro bias kalibrasyonu */
    bool     kalibre_suruyor;
    uint32_t kalibre_n;
    float    kalibre_toplam[3];
} ahrs_t;

void ahrs_baslat(ahrs_t *a);

/* Her IMU orneginde cagrilir. o->gyro icinden bias burada cikarilir. */
void ahrs_guncelle(ahrs_t *a, imu_orneklem_t *o, float dt_s);

/* Gyro bias kalibrasyonunu baslatir: sistem HAREKETSIZ olmali. */
void ahrs_kalibrasyon_basla(ahrs_t *a);

/* Yeterli ornek toplandiysa bias'i g_cfg'ye yazar ve true doner. */
bool ahrs_kalibrasyon_bitti_mi(ahrs_t *a);

/* Yaw'i verilen degere kurar (referanslama / kilitleme sonrasi). */
void ahrs_yaw_ayarla(ahrs_t *a, float derece);

void ahrs_durum(const ahrs_t *a, durus_t *d);
