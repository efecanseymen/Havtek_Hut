#pragma once
/*
 * system_types.h -- moduller arasi ortak tipler.
 *
 * Kural: moduller birbirini global degiskenle degil, bu tiplerle ve kuyrukla
 * konusur. Boylece her modul tek basina test edilebilir kalir.
 */

#include <stdint.h>
#include <stdbool.h>

#define HUT_SURUM  "0.2.0"

/* Ayni anda izlenen en fazla eksen sayisi. */
#define HUT_MAKS_SURUCU  4

/* Ayar tablosunda yer ayrilan slot: 2 I2C hatti x 3 adres (0x16..0x18).
   Iki surucu de 0x16'da olabilecegi icin anahtar sadece adres DEGIL,
   (hat, adres) ikilisi. Boyut 8: slot_no() hat*4+(adres-0x16) kullanir. */
#define HUT_MAKS_SLOT    8

/* ------------------------------------------------------------------ eksenler */

typedef enum {
    ROL_AZ = 0,        /* yatay eksen  */
    ROL_EL = 1,        /* dikey eksen  */
    ROL_SAYISI
} eksen_rol_t;

/* ------------------------------------------------------------------ modlar */

/*
 * KULLANICI MODU -- sartnamenin istedigi iki mod. Operatorun sectigi sey bu.
 *   MANUEL   : arayuzden girilen aci/jog komutlari uygulanir, stabilizasyon yok
 *   OTOMATIK : hedef kilitli, sensorle surekli duzeltme yapilir
 *
 * Asagidaki sistem_modu_t ise sistemin O AN ne yaptigini soyleyen ic durum
 * (jog mu ediyor, aciya mi gidiyor, hata mi var). Ikisi ayri kavram: operator
 * OTOMATIK'te iken bile sistem gecici olarak "mudahale" durumunda olabilir.
 */
typedef enum {
    KMOD_MANUEL = 0,
    KMOD_OTOMATIK = 1
} kullanici_modu_t;

typedef enum {
    MOD_BOSTA = 0,     /* motor duruyor, bobinler enerjili                 */
    MOD_JOG,           /* kullanici basili tuttugu surece donuyor          */
    MOD_GIT,           /* belirli bir aciya/adima gidiyor                  */
    MOD_STAB,          /* tek eksenli stabilizasyon (deneysel)             */
    MOD_KALIBRE,       /* gyro veya manyetometre kalibrasyonu suruyor      */
    MOD_HATA           /* sistem kendi hatasini yakaladi, mudahale bekler  */
} sistem_modu_t;

typedef enum {
    HATA_YOK = 0,
    HATA_I2C,              /* surucu/IMU I2C'de art arda cevap vermedi     */
    HATA_IMU_YOK,          /* IMU bulunamadi ama gereken bir mod istendi   */
    HATA_IMU_ZAMAN_ASIMI,  /* IMU verisi 100 ms'den uzun suredir bayat     */
    HATA_LIMIT,            /* yazilim limitine dayandi                     */
    HATA_SAPMA,            /* stabilizasyonda hata siniri asildi           */
    HATA_SURUCU_YOK,       /* istenen eksende surucu yok                   */
    HATA_ACIL_DURDURMA,    /* kullanici acil durdurdu                      */
    HATA_TERS_YON          /* motor doniyor ama hata buyuyor -> yon ters   */
} hata_kodu_t;

/* ------------------------------------------------------------------ sensor */

typedef struct {
    int64_t t_us;
    float   gyro[3];       /* derece/s, bias duzeltmesi UYGULANMIS         */
    float   ivme[3];       /* g                                            */
    float   mag[3];        /* uT ham (kalibrasyon uygulanmis)              */
    bool    mag_var;
    bool    gecerli;
} imu_orneklem_t;

typedef struct {
    float roll, pitch, yaw;   /* derece                                    */
    float pusula;             /* manyetometreden kuzey acisi, yoksa NAN    */
    bool  kalibre;            /* gyro bias olculdu mu                      */
} durus_t;

/* ------------------------------------------------------------------ komutlar */

typedef enum {
    KOMUT_TARA = 0,
    KOMUT_ROL_ATA,        /* a = eksen indeksi, b = rol                    */
    KOMUT_EKSEN_TANI,     /* a = eksen indeksi                             */
    KOMUT_JOG,            /* a = rol, b = yon(-1/+1), c = adim/s           */
    KOMUT_JOG_DUR,        /* a = rol                                       */
    KOMUT_GIT,            /* a = rol, c = hedef derece                     */
    KOMUT_DUR,
    KOMUT_ACIL_DUR,
    KOMUT_HATA_SIL,
    KOMUT_REFERANS,       /* a = rol : burasi sifir                        */
    KOMUT_SERBEST,        /* a = rol : bobinleri birak                     */
    KOMUT_GYRO_KALIBRE,
    KOMUT_MAG_KALIBRE,    /* a = 1 basla / 0 bitir                         */
    KOMUT_STAB,           /* a = 1 ac / 0 kapat, b = rol                   */
    KOMUT_STAB_KILIT,     /* o anki aciyi hedef yap                        */
    KOMUT_PARAM,          /* metin anahtar + c degeri                      */
    KOMUT_LOG,            /* a = 1 ac / 0 kapat                            */
    KOMUT_YON_TERS,       /* a = rol : yonu ters cevir                     */
    KOMUT_LIMIT_OGRET,    /* a = rol, b = 0 alt / 1 ust : burasi limit     */
    KOMUT_OLCUM_EKSENI,   /* a = rol, b = 0 roll / 1 pitch / 2 yaw         */
    KOMUT_KULLANICI_MODU, /* a = 0 manuel / 1 otomatik                     */
    KOMUT_TAKIPTEN_DUS,   /* sartname: takipten dusurme                    */
    KOMUT_MUDAHALE_TEST,  /* a = rol, c = derece : 8 sn testini calistir   */
    KOMUT_PERF_SIFIRLA    /* stabilizasyon performans sayaclarini sifirla   */
} komut_tipi_t;

/* Stabilizasyonun hangi IMU acisini takip ettigi. Mekanik montaj IMU'yu nasil
   oturttuysa o degisir; varsayim yerine OLCULMESI gerekiyor. */
typedef enum {
    OLCUM_ROLL = 0,
    OLCUM_PITCH = 1,
    OLCUM_YAW = 2
} olcum_ekseni_t;

typedef struct {
    komut_tipi_t tip;
    int32_t      a;
    int32_t      b;
    float        c;
    char         anahtar[20];   /* KOMUT_PARAM icin                        */
} komut_t;

/* ------------------------------------------------------------------ durum */

typedef struct {
    bool     var;
    uint8_t  hat;
    uint8_t  adres;
    uint8_t  rol;
    int8_t   yon;
    int32_t  adim;
    float    derece;
    float    sps;          /* anlik isaretli hiz                           */
    float    hedef_sps;
    uint32_t i2c_hata;
} eksen_durum_t;

/* Bir eksenin stabilizasyon durumu. Iki eksen ayni anda calisabiliyor. */
typedef struct {
    bool     aktif;        /* bu eksen stabilize ediliyor mu                */
    bool     surucu_var;
    float    hedef;        /* kilitlenen aci (derece)                       */
    float    olculen;      /* IMU'nun o an okudugu aci                      */
    float    hata;         /* hedef - olculen                               */
    float    kp;           /* kontrolcunun o anki kazanci                   */
    float    sps;          /* motora verilen hiz komutu                     */
    float    sapma;        /* eksenin kilit noktasindan uzakligi            */
    uint8_t  olcum;        /* hangi IMU acisi (OLCUM_*)                     */
} stab_durum_t;

typedef struct {
    int64_t        t_us;
    sistem_modu_t  mod;
    hata_kodu_t    hata;
    hata_kodu_t    uyari;      /* sistemi durdurmayan sorun                */
    kullanici_modu_t kmod;

    imu_orneklem_t imu;
    durus_t        durus;

    eksen_durum_t  eksen[HUT_MAKS_SURUCU];
    uint8_t        surucu_sayisi;

    bool           imu_var;
    uint8_t        imu_adres;
    uint8_t        mag_adres;
    char           mag_tip[12];

    /* stabilizasyon */
    bool     stab_acik;        /* otomatik takip calisiyor mu              */
    bool     kilitli;          /* hedef kilitlendi mi                      */
    bool     mudahale;         /* operator gecici olarak elle yonlendiriyor */
    stab_durum_t stab[ROL_SAYISI];

    /* sartname: mudahale sonrasi yeniden yonelim suresi (sinir 8 sn) */
    uint32_t kilitlenme_ms;
    bool     kilitlenme_ok;
    bool     kilitlenme_olculuyor;

    /* stabilizasyon performansi (hedefe ne kadar iyi tutunuyor) */
    float    perf_rms;         /* hata RMS, derece                         */
    float    perf_maks;        /* en buyuk mutlak hata, derece             */
    uint32_t perf_sure_s;      /* kac saniyedir olculuyor                  */

    /* kalibrasyon */
    bool     mag_kalibre_suruyor;
    uint32_t mag_orneklem;

    /* saglik */
    uint32_t dongu_us;      /* son dongunun suresi                         */
    uint32_t asim;          /* 1 ms dilimini kacirma sayaci                */
    uint32_t log_satir;
    bool     log_acik;
} sistem_durum_t;

/* ------------------------------------------------------------------ log */

/* Bir log orneklemi. Tarayici bunu CSV'ye ceviriyor. */
typedef struct {
    int64_t t_us;
    float   gyro[3];
    float   ivme[3];
    float   roll, pitch, yaw;
    int32_t adim[2];
    float   sps[2];
    float   hedef[2];      /* eksen basina kilitlenen aci                  */
    float   hata[2];       /* eksen basina hata                            */
    float   kp[2];
    uint8_t kmod;          /* 0 manuel / 1 otomatik                        */
    uint8_t mudahale;
} log_orneklem_t;
