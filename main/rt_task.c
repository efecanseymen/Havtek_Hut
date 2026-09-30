/*
 * rt_task.c -- gercek zaman gorevi. I2C hattinin TEK sahibi.
 *
 * Neden tek gorev: hatta hem M20 suruculer hem IMU var ve I2C'de ayni anda
 * tek islem yapilabiliyor. Birden fazla gorev hatti paylasirsa adim
 * zamanlamasi ongorulemez hale gelir (mutex beklemesi = jitter = adim
 * kacirma). Burada her sey tek bir 1 ms izgarada siraya giriyor.
 *
 * Dilim plani:
 *   her 1 ms : bekleyen komutlari isle, her eksene en fazla bir adim attir
 *   her 10 ms: IMU oku, aci kestir, kontrolcuyu calistir, durum yayinla, logla
 *
 * Bir dilimin butcesi: IMU okumasi ~0.4 ms (400 kHz) + adim basina ~1.26 ms
 * (50 kHz). Yani 10 ms'lik pencerede rahat rahat sigiyor; "asim" sayaci
 * gercekte ne kadar sigdigini olcuyor.
 */

#include "app_priv.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "config.h"
#include "pins.h"
#include "hut_state.h"
#include "i2c_hub.h"
#include "eksen.h"
#include "imu.h"
#include "ahrs.h"
#include "kontrol.h"

static const char *TAG = "rt";

#define KONTROL_BOLEN    10       /* 1 ms izgarada 10 = 100 Hz            */
#define IMU_ZAMAN_ASIMI_US  100000
#define TANI_ADIM        120      /* eksen tani testinin attigi adim      */
#define TANI_ZAMAN_US    8000000

/* ------------------------------------------------------------- durum */

static eksen_t  s_eksen[HUT_MAKS_SURUCU];
static uint8_t  s_eksen_sayi;

static ahrs_t     s_ahrs;
static kontrol_t  s_kontrol;
static imu_orneklem_t s_imu;
static int64_t    s_imu_son_us;

static sistem_modu_t s_mod = MOD_BOSTA;
static hata_kodu_t   s_hata = HATA_YOK;

static bool    s_stab_acik;
static uint8_t s_stab_rol = ROL_EL;
static float   s_stab_hedef;
static float   s_stab_olculen, s_stab_hata, s_stab_sps;
static int32_t s_stab_merkez_adim;

/* Ters yon tespiti: eksen hareket ediyor ama hata duzelmiyorsa polarite
   yanlis demektir. 200 ms'lik orneklerle (sapma, hata) degisimlerinin
   isaretine bakiyoruz; ayni yonde degisiyorlarsa geri besleme POZITIF. */
static int64_t s_ters_ornek_us;
static float   s_ters_son_sapma, s_ters_son_hata;
static int     s_ters_sayac;

static bool s_log_acik;
static uint32_t s_log_satir;
static uint32_t s_asim;
static uint32_t s_dongu_us;   /* son yayindan beri en uzun dilim */

/* manyetometre kalibrasyonu */
static bool  s_mag_kal;
static float s_mag_min[3], s_mag_maks[3];
static uint32_t s_mag_n;

/* eksen tani durumu */
static struct {
    bool     aktif;
    int      indeks;
    int64_t  baslangic_us;
    float    roll0, pitch0, yaw0;
    int32_t  adim0;
} s_tani;

/* --------------------------------------------------------- yardimcilar */

static const char *olcum_adi(uint8_t e);

static int eksen_bul_rol(int rol)
{
    for (int i = 0; i < s_eksen_sayi; i++) {
        if (s_eksen[i].var && s_eksen[i].rol == rol) {
            return i;
        }
    }
    return -1;
}

/* Ayar tablosundaki slot: iki surucu de 0x16 olabildigi icin anahtar
   (hat, adres) ikilisi. */
static int slot_no(int hat, uint8_t adres)
{
    int idx = hat * 4 + (adres - ADR_MOTOR_ILK);

    return (idx >= 0 && idx < HUT_MAKS_SLOT) ? idx : -1;
}

/* Arayuz eksenleri adresle degil, telemetrideki SIRA ile gosteriyor:
   iki surucu ayni adreste olabildigi icin adres tek basina benzersiz degil. */
static int eksen_gecerli(int indeks)
{
    return (indeks >= 0 && indeks < s_eksen_sayi && s_eksen[indeks].var)
           ? indeks : -1;
}

static float rol_reduktor(int rol)
{
    return g_cfg.reduktor[(rol >= 0 && rol < ROL_SAYISI) ? rol : 0];
}

static void hepsini_durdur(void)
{
    for (int i = 0; i < s_eksen_sayi; i++) {
        eksen_dur(&s_eksen[i]);
    }
}

static void acil_durdur(void)
{
    for (int i = 0; i < s_eksen_sayi; i++) {
        eksen_acil_dur(&s_eksen[i]);
    }
    s_stab_acik = false;
    s_tani.aktif = false;
    s_mod  = MOD_HATA;
    s_hata = HATA_ACIL_DURDURMA;
}

static void hataya_dus(hata_kodu_t kod)
{
    for (int i = 0; i < s_eksen_sayi; i++) {
        eksen_acil_dur(&s_eksen[i]);
    }
    s_stab_acik = false;
    s_tani.aktif = false;
    s_mod  = MOD_HATA;
    s_hata = kod;
    ESP_LOGE(TAG, "hata durumuna gecildi: %d", (int)kod);
}

/* --------------------------------------------------------- kesif / kurulum */

static void hat_kesfet(int hat)
{
    i2c_tarama_t tarama;

    if (!i2c_hub_hat_var(hat)) {
        return;
    }
    i2c_hub_tara(hat, &tarama);

    /* 0x16 (varsayilan) her zaman ilk eklenir.
       ADR2 lehimlendi: ikinci surucu 0x18'de. */
    static const uint8_t aday[] = {
        M20_ADRES_VARSAYILAN,    /* 0x16 -- fabrika cikisi      */
        M20_ADRES_ALT2,          /* 0x18 -- ADR2 lehim koprusu  */
    };

    for (int a = 0; a < (int)(sizeof(aday) / sizeof(aday[0])); a++) {
        uint8_t adres = aday[a];
        bool bulundu = false;

        for (int i = 0; i < tarama.sayi; i++) {
            if (tarama.adres[i] == adres) {
                bulundu = true;
                break;
            }
        }
        if (!bulundu || s_eksen_sayi >= HUT_MAKS_SURUCU) {
            continue;
        }

        int slot = slot_no(hat, adres);
        uint8_t rol = (slot >= 0) ? g_cfg.rol_map[slot] : ROL_AZ;
        int8_t  yon = (slot >= 0 && g_cfg.yon_map[slot]) ? g_cfg.yon_map[slot] : 1;

        if (eksen_baslat(&s_eksen[s_eksen_sayi], hat, adres, rol, yon) == ESP_OK) {
            ESP_LOGI(TAG, "eksen %d eklendi: hat %d 0x%02X rol=%s",
                     s_eksen_sayi, hat, adres, rol == ROL_AZ ? "yatay" : "dikey");
            s_eksen_sayi++;
        }
    }
}

static void suruculeri_kesfet(void)
{
    for (int i = 0; i < s_eksen_sayi; i++) {
        eksen_bitir(&s_eksen[i]);
    }
    s_eksen_sayi = 0;

    for (int hat = 0; hat < I2C_HUB_MAKS_HAT; hat++) {
        hat_kesfet(hat);
    }

    if (s_eksen_sayi == 0) {
        ESP_LOGW(TAG, "hicbir motor surucusu bulunamadi (0x16, 0x17, 0x18)");
        ESP_LOGW(TAG, "iki surucu de 0x16 ise ayni hatta takilamaz: ya ikinci");
        ESP_LOGW(TAG, "hatti kullanin (SDA=%d SCL=%d) ya da lehim koprusuyle",
                 PIN_I2C2_SDA, PIN_I2C2_SCL);
        ESP_LOGW(TAG, "birini 0x17 veya 0x18 yapin.");
    } else if (s_eksen_sayi == 1) {
        ESP_LOGI(TAG, "tek surucu bulundu (0x%02X). Ikinci surucu icin lehim "
                 "koprusuyle 0x17 veya 0x18 yapip ayni hatta takin veya ikinci "
                 "hatti kullanin.", s_eksen[0].adres);
    } else {
        ESP_LOGI(TAG, "%d surucu bulundu:", s_eksen_sayi);
        for (int i = 0; i < s_eksen_sayi; i++) {
            ESP_LOGI(TAG, "  eksen %d: hat %d, adres 0x%02X, rol=%s",
                     i, s_eksen[i].hat, s_eksen[i].adres,
                     s_eksen[i].rol == ROL_AZ ? "yatay" : "dikey");
        }

        /* Ayni hatta iki farkli adresli surucu bulunduysa ve rolleri henuz
           NVS'te atanmamissa, varsayilan atamayi yap: dusuk adres -> AZ,
           yuksek adres -> EL. Kullanici sonradan arayuzden veya eksen tani
           ile degistirebilir. */
        if (s_eksen_sayi == 2 &&
            s_eksen[0].hat == s_eksen[1].hat &&
            s_eksen[0].adres != s_eksen[1].adres) {

            int dusuk = (s_eksen[0].adres < s_eksen[1].adres) ? 0 : 1;
            int yuksek = 1 - dusuk;
            int slot_d = slot_no(s_eksen[dusuk].hat, s_eksen[dusuk].adres);
            int slot_y = slot_no(s_eksen[yuksek].hat, s_eksen[yuksek].adres);

            bool d_atanmis = (slot_d >= 0 && g_cfg.rol_atandi[slot_d]);
            bool y_atanmis = (slot_y >= 0 && g_cfg.rol_atandi[slot_y]);

            if (!d_atanmis && !y_atanmis) {
                s_eksen[dusuk].rol  = ROL_AZ;
                s_eksen[yuksek].rol = ROL_EL;
                ESP_LOGI(TAG, "oto-atama: 0x%02X -> yatay (AZ), "
                         "0x%02X -> dikey (EL)",
                         s_eksen[dusuk].adres, s_eksen[yuksek].adres);
                ESP_LOGI(TAG, "degistirmek icin 'eksen tani' veya 'rol ata' "
                         "komutunu kullanin.");
            }
        }
    }
}

/* ----------------------------------------------------------- eksen tani */

/*
 * "Hangi motor bagli?" sorusunun cevabini surucu vermiyor -- kart motoru
 * tanimiyor, hatta bagli olup olmadigini bile bilmiyor. Ama IMU canak
 * uzerinde oldugu icin dolayli olarak anlayabiliyoruz: birkac adim at,
 * hangi acinin degistigine bak.
 *   pitch degistiyse -> dikey (elevasyon)
 *   yaw degistiyse   -> yatay (azimut)
 * Degisimin isareti de motor yonunu veriyor.
 */
static void tani_basla(int indeks)
{
    if (indeks < 0 || !s_imu.gecerli) {
        s_hata = HATA_IMU_YOK;
        return;
    }
    s_tani.aktif        = true;
    s_tani.indeks       = indeks;
    s_tani.baslangic_us = esp_timer_get_time();
    s_tani.roll0        = s_ahrs.roll;
    s_tani.pitch0       = s_ahrs.pitch;
    s_tani.yaw0         = s_ahrs.yaw;
    s_tani.adim0        = s_eksen[indeks].adim;

    eksen_git(&s_eksen[indeks], s_tani.adim0 + TANI_ADIM);
    ESP_LOGI(TAG, "eksen tani basladi (0x%02X)", s_eksen[indeks].adres);
}

static void tani_isle(int64_t simdi)
{
    if (!s_tani.aktif) {
        return;
    }
    eksen_t *e = &s_eksen[s_tani.indeks];
    bool bitti = !e->git_aktif;
    bool zaman_asimi = (simdi - s_tani.baslangic_us) > TANI_ZAMAN_US;

    if (!bitti && !zaman_asimi) {
        return;
    }

    float d_roll  = s_ahrs.roll  - s_tani.roll0;
    float d_pitch = s_ahrs.pitch - s_tani.pitch0;
    float d_yaw   = s_ahrs.yaw   - s_tani.yaw0;
    int32_t d_adim = e->adim - s_tani.adim0;

    s_tani.aktif = false;
    eksen_git(e, s_tani.adim0);     /* baslangica don */

    if (fabsf(d_roll) < 1.0f && fabsf(d_pitch) < 1.0f && fabsf(d_yaw) < 1.0f) {
        ESP_LOGW(TAG, "eksen tani: aci degismedi (d_roll=%.2f d_pitch=%.2f "
                 "d_yaw=%.2f, %ld adim atildi)", d_roll, d_pitch, d_yaw,
                 (long)d_adim);
        ESP_LOGW(TAG, "IMU hareket eden eksene MONTELI degilse bu test calismaz;");
        ESP_LOGW(TAG, "tezgahta IMU masada duruyorsa rolu elle secin. Diger");
        ESP_LOGW(TAG, "ihtimaller: motor bagli degil, adim kaciriyor, mekanik tutmus.");
        return;
    }

    int slot = slot_no(e->hat, e->adres);
    float degisim;
    uint8_t olcum;

    /*
     * En cok degisen aci bu ekseni temsil ediyor. Roll de adaylar arasinda:
     * IMU'nun montaj yonune gore elevasyon hareketi pitch yerine roll'de
     * gorunebiliyor ve bunu varsaymak yerine olcmek gerekiyor.
     *
     * Yaw ayrica kaydigi icin kucuk bir esikle one gecmesin diye en son
     * degerlendiriliyor.
     */
    if (fabsf(d_roll) >= fabsf(d_pitch) && fabsf(d_roll) >= fabsf(d_yaw)) {
        olcum   = OLCUM_ROLL;
        degisim = d_roll;
        e->rol  = ROL_EL;      /* roll/pitch = egim -> dikey eksen */
    } else if (fabsf(d_pitch) >= fabsf(d_yaw)) {
        olcum   = OLCUM_PITCH;
        degisim = d_pitch;
        e->rol  = ROL_EL;
    } else {
        olcum   = OLCUM_YAW;
        degisim = d_yaw;
        e->rol  = ROL_AZ;
    }

    g_cfg.olcum_ekseni[e->rol] = olcum;

    /* Aci, komut edilen yonde mi degisti? Degilse motor yonu ters. */
    bool ters = (degisim > 0.0f) != (d_adim > 0);
    e->yon = ters ? (int8_t)(-e->yon) : e->yon;

    if (slot >= 0) {
        g_cfg.rol_map[slot]    = e->rol;
        g_cfg.yon_map[slot]    = e->yon;
        g_cfg.rol_atandi[slot] = true;
        hut_cfg_kirlet();
    }

    ESP_LOGI(TAG, "eksen tani: hat %d 0x%02X -> %s eksen, olcum=%s, yon=%d",
             e->hat, e->adres, e->rol == ROL_AZ ? "yatay" : "dikey",
             olcum_adi(olcum), e->yon);
    ESP_LOGI(TAG, "  olculen degisim: roll %.1f  pitch %.1f  yaw %.1f  (%ld adim)",
             d_roll, d_pitch, d_yaw, (long)d_adim);
}

/* ------------------------------------------------------ mag kalibrasyonu */

static void mag_kal_basla(void)
{
    s_mag_kal = true;
    s_mag_n   = 0;
    for (int i = 0; i < 3; i++) {
        s_mag_min[i]  =  1e9f;
        s_mag_maks[i] = -1e9f;
    }
    ESP_LOGI(TAG, "mag kalibrasyonu: azimut eksenini yavasca bir tur dondurun");
}

static void mag_kal_bitir(void)
{
    s_mag_kal = false;

    if (s_mag_n < 100) {
        ESP_LOGW(TAG, "mag kalibrasyonu icin yeterli ornek yok (%lu)",
                 (unsigned long)s_mag_n);
        return;
    }

    float yaricap[3], ortalama = 0.0f;
    for (int i = 0; i < 3; i++) {
        g_cfg.mag_ofset[i] = (s_mag_maks[i] + s_mag_min[i]) * 0.5f;
        yaricap[i]         = (s_mag_maks[i] - s_mag_min[i]) * 0.5f;
        ortalama += yaricap[i];
    }
    ortalama /= 3.0f;

    for (int i = 0; i < 3; i++) {
        g_cfg.mag_olcek[i] = (yaricap[i] > 0.1f) ? (ortalama / yaricap[i]) : 1.0f;
    }
    g_cfg.mag_kalibre = true;
    hut_cfg_kirlet();

    ESP_LOGI(TAG, "mag ofset %.1f %.1f %.1f  olcek %.2f %.2f %.2f",
             g_cfg.mag_ofset[0], g_cfg.mag_ofset[1], g_cfg.mag_ofset[2],
             g_cfg.mag_olcek[0], g_cfg.mag_olcek[1], g_cfg.mag_olcek[2]);
}

/* ---------------------------------------------------------- stabilizasyon */

/* Hangi IMU acisi bu ekseni temsil ediyor? Mekanik montaja bagli, o yuzden
   ayardan okunuyor ("Eksen Tani" bunu olcup yaziyor). */
static float stab_olcum(void)
{
    uint8_t eksen = g_cfg.olcum_ekseni[s_stab_rol < ROL_SAYISI ? s_stab_rol : 0];

    switch (eksen) {
    case OLCUM_ROLL:  return s_ahrs.roll;
    case OLCUM_YAW:   return s_ahrs.yaw;
    case OLCUM_PITCH:
    default:          return s_ahrs.pitch;
    }
}

static const char *olcum_adi(uint8_t e)
{
    switch (e) {
    case OLCUM_ROLL: return "roll";
    case OLCUM_YAW:  return "yaw";
    default:         return "pitch";
    }
}

/* Olculen aci 360 derecede doniyor mu? Yalnizca yaw icin evet. */
static bool olcum_sarmali(void)
{
    return g_cfg.olcum_ekseni[s_stab_rol < ROL_SAYISI ? s_stab_rol : 0]
           == OLCUM_YAW;
}

static void stab_kilitle(void)
{
    s_stab_hedef = stab_olcum();
    int i = eksen_bul_rol(s_stab_rol);
    s_stab_merkez_adim = (i >= 0) ? s_eksen[i].adim : 0;
    kontrol_sifirla(&s_kontrol);

    s_ters_ornek_us  = esp_timer_get_time();
    s_ters_son_sapma = 0.0f;
    s_ters_son_hata  = 0.0f;
    s_ters_sayac     = 0;

    ESP_LOGI(TAG, "hedef kilitlendi: %.2f derece (%s)", s_stab_hedef,
             olcum_adi(g_cfg.olcum_ekseni[s_stab_rol]));
}

static void stab_ac(bool ac, int rol)
{
    if (!ac) {
        s_stab_acik = false;
        hepsini_durdur();
        if (s_mod == MOD_STAB) {
            s_mod = MOD_BOSTA;
        }
        return;
    }

    if (!s_imu.gecerli) {
        s_hata = HATA_IMU_YOK;
        ESP_LOGW(TAG, "IMU yok, stabilizasyon acilamaz");
        return;
    }
    if (rol >= 0) {
        s_stab_rol = (uint8_t)rol;
    }
    if (eksen_bul_rol(s_stab_rol) < 0) {
        /* Tezgahta tek motorla calisirken panelde secilen rol ile takili
           surucunun rolu tutmuyorsa is durmasin: tek eksen varsa onu kullan
           ve hangisini sectigimizi acikca soyle. */
        if (s_eksen_sayi == 1 && s_eksen[0].var) {
            s_stab_rol = s_eksen[0].rol;
            ESP_LOGW(TAG, "secilen rolde surucu yok; tek takili eksen (%s) "
                     "kullaniliyor", s_stab_rol == ROL_AZ ? "yatay" : "dikey");
        } else {
            s_hata = HATA_SURUCU_YOK;
            ESP_LOGW(TAG, "%s rolunde surucu yok, stabilizasyon acilamaz",
                     s_stab_rol == ROL_AZ ? "yatay" : "dikey");
            return;
        }
    }
    if (!g_cfg.gyro_kalibre) {
        ESP_LOGW(TAG, "gyro kalibre edilmemis -- yaw hizla kayacak");
    }

    stab_kilitle();
    s_stab_acik = true;
    s_mod = MOD_STAB;

    /* Acilis ozeti: sonradan "hangi ayarla calisiyordu" sorusuna cevap. */
    ESP_LOGI(TAG, "STABILIZASYON ACIK -- eksen=%s olcum=%s reduktor=%.2f",
             s_stab_rol == ROL_AZ ? "yatay" : "dikey",
             olcum_adi(g_cfg.olcum_ekseni[s_stab_rol]),
             rol_reduktor(s_stab_rol));
    ESP_LOGI(TAG, "  pencere=%.0f derece  sapma siniri=%.0f derece  "
             "maks hiz=%.0f adim/s", g_cfg.stab_pencere,
             g_cfg.stab_hata_sinir, g_cfg.maks_sps);
}

static void stab_calistir(float dt, int64_t simdi_us)
{
    int i = eksen_bul_rol(s_stab_rol);
    if (i < 0) {
        hataya_dus(HATA_SURUCU_YOK);
        return;
    }

    eksen_t *e = &s_eksen[i];

    s_stab_olculen = stab_olcum();
    s_stab_hata    = s_stab_hedef - s_stab_olculen;

    /* Yaw'da -180/+180 gecisini kisa yoldan gec. Roll/pitch sarmaz. */
    if (olcum_sarmali()) {
        if (s_stab_hata >  180.0f) s_stab_hata -= 360.0f;
        if (s_stab_hata < -180.0f) s_stab_hata += 360.0f;
    }

    /* Guvenlik 1: hata siniri. Genelde "yanlis eksen secildi" ya da
       "motor adim kaciriyor" demek. */
    if (fabsf(s_stab_hata) > g_cfg.stab_hata_sinir) {
        ESP_LOGE(TAG, "sapma siniri asildi (%.1f derece)", s_stab_hata);
        hataya_dus(HATA_SAPMA);
        return;
    }

    float reduktor = rol_reduktor(s_stab_rol);
    float merkez_der = (float)s_stab_merkez_adim *
                       (EKSEN_MOTOR_DERECE_ADIM / reduktor);
    float sapma = eksen_derece(e, reduktor) - merkez_der;

    /*
     * Guvenlik 2: TERS YON tespiti.
     *
     * Kontrolcu hatayi kapatmak icin ekseni cevirir. Polarite dogruysa eksen
     * ilerledikce hata kuculuyor olmali. Ikisi AYNI yonde degisiyorsa geri
     * besleme pozitiftir: motor hatayi buyuterek kacar ve saniyeler icinde
     * pencereye carpar. Bunu "limit hatasi" diye bildirmek kullaniciyi yanlis
     * yere bakmaya iter; o yuzden once burada yakaliyoruz.
     *
     * 200 ms'lik orneklerle bakiyoruz ve ust uste 3 ornek istiyoruz: elle
     * platformu egmek de hatayi buyutur ama eksen hareketiyle ayni yonde
     * tutarli bir iliski uretmez.
     */
    if (simdi_us - s_ters_ornek_us >= 200000) {
        float d_sapma = sapma - s_ters_son_sapma;
        float d_hata  = fabsf(s_stab_hata) - fabsf(s_ters_son_hata);

        if (fabsf(d_sapma) > 0.5f && d_hata > 0.1f && fabsf(s_stab_hata) > 2.0f) {
            s_ters_sayac++;
        } else {
            s_ters_sayac = 0;
        }

        s_ters_son_sapma = sapma;
        s_ters_son_hata  = s_stab_hata;
        s_ters_ornek_us  = simdi_us;

        if (s_ters_sayac >= 3) {
            ESP_LOGE(TAG, "TERS YON: eksen %.1f derece dondu ama hata "
                     "%.1f dereceye BUYUDU", sapma, s_stab_hata);
            ESP_LOGE(TAG, "Yapilacak: Cihazlar sekmesinde bu eksen icin");
            ESP_LOGE(TAG, "'Yonu Ters' deyin, sonra hedefi tekrar kilitleyin.");
            ESP_LOGE(TAG, "Ikinci ihtimal: olcum ekseni yanlis (%s secili).",
                     olcum_adi(g_cfg.olcum_ekseni[s_stab_rol]));
            hataya_dus(HATA_TERS_YON);
            return;
        }
    }

    /* Guvenlik 3: calisma penceresi. Eksen referanstan fazla uzaklasmasin. */
    if (fabsf(sapma) > g_cfg.stab_pencere) {
        ESP_LOGE(TAG, "calisma penceresi disina cikildi (%.1f derece, sinir %.0f)",
                 sapma, g_cfg.stab_pencere);
        hataya_dus(HATA_LIMIT);
        return;
    }

    s_stab_sps = kontrol_hesapla(&s_kontrol, s_stab_hata, dt, reduktor);
    eksen_hiz(e, s_stab_sps);
}

/* ------------------------------------------------------------- komutlar */

static void komut_uygula(const komut_t *k)
{
    int i;

    /* Hata durumundayken sadece iki komut gecerli. */
    if (s_mod == MOD_HATA &&
        k->tip != KOMUT_HATA_SIL && k->tip != KOMUT_ACIL_DUR) {
        return;
    }

    /*
     * Gyro kalibrasyonu sirasinda motor donerse titresim bias olarak olculur
     * ve yanlis deger kalici olarak kaydedilir. Hareket komutlarini yok say.
     */
    if (s_mod == MOD_KALIBRE) {
        switch (k->tip) {
        case KOMUT_JOG:
        case KOMUT_GIT:
        case KOMUT_EKSEN_TANI:
        case KOMUT_STAB:
            ESP_LOGW(TAG, "kalibrasyon suruyor, hareket komutu yok sayildi");
            return;
        default:
            break;
        }
    }

    switch (k->tip) {
    case KOMUT_TARA:
        hepsini_durdur();
        suruculeri_kesfet();
        break;

    case KOMUT_ROL_ATA: {
        i = eksen_gecerli(k->a);
        if (i < 0 || k->b < 0 || k->b >= ROL_SAYISI) {
            break;
        }
        s_eksen[i].rol = (uint8_t)k->b;

        int slot = slot_no(s_eksen[i].hat, s_eksen[i].adres);
        if (slot >= 0) {
            g_cfg.rol_map[slot]    = (uint8_t)k->b;
            g_cfg.rol_atandi[slot] = true;
            hut_cfg_kirlet();
        }
        break;
    }

    case KOMUT_EKSEN_TANI:
        tani_basla(eksen_gecerli(k->a));
        break;

    case KOMUT_YON_TERS:
        i = eksen_bul_rol(k->a);
        if (i >= 0) {
            s_eksen[i].yon = (int8_t)(-s_eksen[i].yon);
            int slot = slot_no(s_eksen[i].hat, s_eksen[i].adres);
            if (slot >= 0) {
                g_cfg.yon_map[slot] = s_eksen[i].yon;
                hut_cfg_kirlet();
            }
        }
        break;

    case KOMUT_JOG:
        if (s_stab_acik) {
            break;                  /* stabilizasyon acikken elle surme yok */
        }
        i = eksen_bul_rol(k->a);
        if (i < 0) {
            s_hata = HATA_SURUCU_YOK;
            break;
        }
        {
            float hiz = (k->c > 0.0f) ? k->c : g_cfg.jog_sps;
            eksen_hiz(&s_eksen[i], (k->b < 0) ? -hiz : hiz);
            s_mod = MOD_JOG;
        }
        break;

    case KOMUT_JOG_DUR:
        i = eksen_bul_rol(k->a);
        if (i >= 0) {
            eksen_dur(&s_eksen[i]);
        }
        if (s_mod == MOD_JOG) {
            s_mod = MOD_BOSTA;
        }
        break;

    case KOMUT_GIT:
        if (s_stab_acik) {
            break;
        }
        i = eksen_bul_rol(k->a);
        if (i < 0) {
            s_hata = HATA_SURUCU_YOK;
            break;
        }
        eksen_git(&s_eksen[i], eksen_derece_adim(k->c, rol_reduktor(k->a)));
        s_mod = MOD_GIT;
        break;

    case KOMUT_DUR:
        hepsini_durdur();
        if (s_mod == MOD_JOG || s_mod == MOD_GIT) {
            s_mod = MOD_BOSTA;
        }
        break;

    case KOMUT_ACIL_DUR:
        acil_durdur();
        break;

    case KOMUT_HATA_SIL:
        s_hata = HATA_YOK;
        s_mod  = MOD_BOSTA;
        break;

    case KOMUT_REFERANS:
        i = eksen_bul_rol(k->a);
        if (i >= 0) {
            eksen_referans(&s_eksen[i]);
        }
        break;

    case KOMUT_SERBEST:
        i = eksen_bul_rol(k->a);
        if (i >= 0) {
            eksen_serbest(&s_eksen[i]);
        }
        break;

    case KOMUT_GYRO_KALIBRE:
        hepsini_durdur();
        ahrs_kalibrasyon_basla(&s_ahrs);
        s_mod = MOD_KALIBRE;
        break;

    case KOMUT_MAG_KALIBRE:
        if (k->a) {
            mag_kal_basla();
        } else {
            mag_kal_bitir();
        }
        break;

    case KOMUT_STAB:
        stab_ac(k->a != 0, k->b);
        break;

    case KOMUT_STAB_KILIT:
        stab_kilitle();
        break;

    case KOMUT_LIMIT_OGRET: {
        /*
         * "Anten su an neredeyse, burasi limit olsun."
         *
         * Limiti sayiyla girmek icin mekanigi cetvelle olcmek gerekir; bunun
         * yerine ekseni elle sinira getirip bu dugmeye basmak hem daha hizli
         * hem daha dogru. Takim tezgahlarinda limit ogretme bu sekilde yapilir.
         */
        i = eksen_bul_rol(k->a);
        if (i < 0) {
            s_hata = HATA_SURUCU_YOK;
            break;
        }
        float aci = eksen_derece(&s_eksen[i], rol_reduktor(k->a));

        if (k->a == ROL_AZ) {
            /* AZ limiti simetrik (+-): mutlak deger alinir. */
            g_cfg.az_limit = fabsf(aci);
            ESP_LOGI(TAG, "AZ limiti ogretildi: +-%.2f derece", g_cfg.az_limit);
        } else if (k->b == 0) {
            g_cfg.el_min = aci;
            ESP_LOGI(TAG, "EL alt limiti ogretildi: %.2f derece", aci);
        } else {
            g_cfg.el_maks = aci;
            ESP_LOGI(TAG, "EL ust limiti ogretildi: %.2f derece", aci);
        }

        /* Alt ust karismissa duzelt -- yon ters bagliysa bu olur. */
        if (g_cfg.el_min > g_cfg.el_maks) {
            float t = g_cfg.el_min;
            g_cfg.el_min = g_cfg.el_maks;
            g_cfg.el_maks = t;
            ESP_LOGW(TAG, "EL alt/ust ters girilmis, yer degistirildi");
        }
        hut_cfg_kirlet();
        break;
    }

    case KOMUT_OLCUM_EKSENI:
        if (k->a >= 0 && k->a < ROL_SAYISI && k->b >= 0 && k->b <= OLCUM_YAW) {
            g_cfg.olcum_ekseni[k->a] = (uint8_t)k->b;
            hut_cfg_kirlet();
            ESP_LOGI(TAG, "%s eksen artik %s acisini takip edecek",
                     k->a == ROL_AZ ? "yatay" : "dikey", olcum_adi(k->b));
        }
        break;

    case KOMUT_PARAM:
        if (!hut_cfg_param_ayarla(&g_cfg, k->anahtar, k->c)) {
            ESP_LOGW(TAG, "bilinmeyen parametre: %s", k->anahtar);
        }
        break;

    case KOMUT_LOG:
        s_log_acik = (k->a != 0);
        if (s_log_acik) {
            hut_log_temizle();
            s_log_satir = 0;
        }
        break;
    }
}

/* ------------------------------------------------------------ yayinlama */

static void durum_yayinla(int64_t simdi, uint32_t dongu_us)
{
    sistem_durum_t d;

    memset(&d, 0, sizeof(d));
    d.t_us = simdi;
    d.mod  = s_mod;
    d.hata = s_hata;
    d.imu  = s_imu;
    ahrs_durum(&s_ahrs, &d.durus);

    const imu_bilgi_t *bilgi = imu_bilgi();
    d.imu_var   = bilgi->var;
    d.imu_adres = bilgi->adres;
    d.mag_adres = bilgi->mag_adres;
    snprintf(d.mag_tip, sizeof(d.mag_tip), "%s",
             bilgi->mag_var ? bilgi->mag_tip : "yok");

    d.surucu_sayisi = s_eksen_sayi;
    for (int i = 0; i < s_eksen_sayi; i++) {
        d.eksen[i].var      = s_eksen[i].var;
        d.eksen[i].hat      = (uint8_t)s_eksen[i].hat;
        d.eksen[i].adres    = s_eksen[i].adres;
        d.eksen[i].rol      = s_eksen[i].rol;
        d.eksen[i].yon      = s_eksen[i].yon;
        d.eksen[i].adim     = s_eksen[i].adim;
        d.eksen[i].derece   = eksen_derece(&s_eksen[i],
                                           rol_reduktor(s_eksen[i].rol));
        d.eksen[i].sps      = s_eksen[i].anlik_sps;
        d.eksen[i].hedef_sps = s_eksen[i].hedef_sps;
        d.eksen[i].i2c_hata = s_eksen[i].i2c_hata;
    }

    d.stab_acik    = s_stab_acik;
    d.stab_rol     = s_stab_rol;
    d.stab_hedef   = s_stab_hedef;
    d.stab_olculen = s_stab_olculen;
    d.stab_hata    = s_stab_hata;
    d.stab_kp      = kontrol_son_kp(&s_kontrol);
    d.stab_sps     = s_stab_sps;

    d.mag_kalibre_suruyor = s_mag_kal;
    d.mag_orneklem        = s_mag_n;

    d.dongu_us  = dongu_us;
    d.asim      = s_asim;
    d.log_acik  = s_log_acik;
    d.log_satir = s_log_satir;

    hut_durum_yaz(&d);
}

static void logla(int64_t simdi)
{
    log_orneklem_t o;

    memset(&o, 0, sizeof(o));
    o.t_us = simdi;
    memcpy(o.gyro, s_imu.gyro, sizeof(o.gyro));
    memcpy(o.ivme, s_imu.ivme, sizeof(o.ivme));
    o.roll  = s_ahrs.roll;
    o.pitch = s_ahrs.pitch;
    o.yaw   = s_ahrs.yaw;

    for (int i = 0; i < s_eksen_sayi && i < 2; i++) {
        int rol = s_eksen[i].rol < 2 ? s_eksen[i].rol : 0;
        o.adim[rol] = s_eksen[i].adim;
        o.sps[rol]  = s_eksen[i].anlik_sps;
    }
    o.hedef = s_stab_hedef;
    o.hata  = s_stab_hata;
    o.kp    = kontrol_son_kp(&s_kontrol);

    hut_log_yaz(&o);
    s_log_satir++;
}

/* ----------------------------------------------------------------- gorev */

static void kontrol_dilimi(int64_t simdi, float dt)
{
    /* --- IMU --- */
    if (imu_bilgi()->var) {
        if (imu_oku(&s_imu) == ESP_OK) {
            s_imu_son_us = simdi;
            ahrs_guncelle(&s_ahrs, &s_imu, dt);

            if (s_mag_kal && s_imu.mag_var) {
                for (int i = 0; i < 3; i++) {
                    if (s_imu.mag[i] < s_mag_min[i])  s_mag_min[i]  = s_imu.mag[i];
                    if (s_imu.mag[i] > s_mag_maks[i]) s_mag_maks[i] = s_imu.mag[i];
                }
                s_mag_n++;
            }
        } else if (simdi - s_imu_son_us > IMU_ZAMAN_ASIMI_US) {
            s_imu.gecerli = false;
            if (s_stab_acik) {
                /* Stabilizasyon acikken korlesmek en tehlikeli durum. */
                hataya_dus(HATA_IMU_ZAMAN_ASIMI);
                (void)i2c_hub_kurtar(0);
            }
        }
    }

    /* --- gyro kalibrasyonu bitti mi --- */
    if (s_mod == MOD_KALIBRE && !s_ahrs.kalibre_suruyor) {
        s_mod = MOD_BOSTA;
    }

    /* --- eksen tani --- */
    tani_isle(simdi);

    /* --- stabilizasyon --- */
    if (s_stab_acik && s_mod == MOD_STAB) {
        stab_calistir(dt, simdi);
    }

    /* --- GIT bitti mi --- */
    if (s_mod == MOD_GIT) {
        bool suren = false;
        for (int i = 0; i < s_eksen_sayi; i++) {
            suren |= s_eksen[i].git_aktif;
        }
        if (!suren) {
            s_mod = MOD_BOSTA;
        }
    }

    if (s_log_acik) {
        logla(simdi);
    }
}

static void rt_gorev(void *arg)
{
    (void)arg;

    ESP_ERROR_CHECK(i2c_hub_baslat(0, PIN_I2C_SDA, PIN_I2C_SCL));

    /* Ikinci hat istege bagli: pini yoksa ya da kurulamazsa sistem tek hatla
       devam eder. Ayni adresli ikinci suruculer icin var. */
    (void)i2c_hub_baslat(1, PIN_I2C2_SDA, PIN_I2C2_SCL);

    ahrs_baslat(&s_ahrs);
    kontrol_sifirla(&s_kontrol);

    if (imu_baslat() != ESP_OK) {
        ESP_LOGW(TAG, "IMU olmadan devam ediliyor (sadece motor kontrolu)");
    }
    suruculeri_kesfet();

    /* Acilista gyro kalibrasyonu: kart hareketsizse bias'i olcer. Hareketli
       bir platformda acilirsa kullanici arayuzden tekrar tetikler. */
    if (imu_bilgi()->var) {
        ahrs_kalibrasyon_basla(&s_ahrs);
        s_mod = MOD_KALIBRE;
    }

    TickType_t son_uyanma = xTaskGetTickCount();
    int64_t onceki_us = esp_timer_get_time();
    int64_t kontrol_onceki_us = onceki_us;
    uint32_t sayac = 0;

    for (;;) {
        vTaskDelayUntil(&son_uyanma, pdMS_TO_TICKS(1));

        int64_t simdi = esp_timer_get_time();
        float   dt    = (float)(simdi - onceki_us) / 1000000.0f;

        /* 1 ms'lik dilim 2 ms'yi asmissa adim zamanlamasi kaymis demektir. */
        if (simdi - onceki_us > 2000) {
            s_asim++;
        }
        onceki_us = simdi;

        komut_t k;
        while (hut_komut_al(&k)) {
            komut_uygula(&k);
        }

        if ((sayac % KONTROL_BOLEN) == 0) {
            float kdt = (float)(simdi - kontrol_onceki_us) / 1000000.0f;
            kontrol_onceki_us = simdi;
            kontrol_dilimi(simdi, kdt);
        }

        for (int i = 0; i < s_eksen_sayi; i++) {
            eksen_servis(&s_eksen[i], esp_timer_get_time(), dt);
        }

        /* Dilimin gercekte ne kadar surdugu: I2C'nin hatti ne kadar mesgul
           ettigini gosteren tek dogru olcu. En kotu deger yayinlaniyor. */
        uint32_t sure = (uint32_t)(esp_timer_get_time() - simdi);
        if (sure > s_dongu_us) {
            s_dongu_us = sure;
        }

        if ((sayac % KONTROL_BOLEN) == 5) {
            durum_yayinla(simdi, s_dongu_us);
            s_dongu_us = 0;
        }
        sayac++;
    }
}

void rt_gorev_baslat(void)
{
    /* Core 0, yuksek oncelik: WiFi ve httpd Core 1'de. */
    xTaskCreatePinnedToCore(rt_gorev, "rt", 6144, NULL, 20, NULL, 0);
}
