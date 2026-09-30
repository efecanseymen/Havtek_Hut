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

/* Son taramada hattaki her adres. Motor olmayanlar da burada: arayuzde
   IMU ve manyetometre de motor suruculeriyle ayni tabloda gorunuyor. */
static struct {
    uint8_t hat;
    uint8_t adres;
} s_bulunan[HUT_MAKS_CIHAZ];
static uint8_t s_bulunan_sayi;

static ahrs_t     s_ahrs;
static imu_orneklem_t s_imu;
static int64_t    s_imu_son_us;

static sistem_modu_t s_mod = MOD_BOSTA;
static hata_kodu_t   s_hata = HATA_YOK;

/* Sistemi durdurmayan sorunlar. Operator gorsun ama is durmasin. */
static hata_kodu_t s_uyari = HATA_YOK;
static int64_t     s_uyari_us;

/* Sartnamenin istedigi iki mod. Operator secer, guc kesilse de korunmaz
   (acilista her zaman MANUEL -- guvenli taraf). */
static kullanici_modu_t s_kmod = KMOD_MANUEL;

/*
 * Eksen basina stabilizasyon durumu. Iki eksen ayni anda calisiyor; her
 * eksenin kendi hedefi, kendi kontrolcusu ve kendi guvenlik sayaclari var.
 */
typedef struct {
    bool      aktif;
    float     hedef;              /* kilitlenen aci                        */
    float     olculen, hata, sps;
    int32_t   merkez_adim;        /* kilit anindaki eksen konumu           */
    kontrol_t kontrol;

    /* ters yon tespiti: eksen donuyor ama hata buyuyorsa polarite yanlis */
    int64_t   ters_us;
    float     ters_sapma, ters_hata;
    int       ters_sayac;

    /* performans: hedefe ne kadar iyi tutunuyor */
    double    hata_kare_toplam;
    uint32_t  hata_n;
    float     hata_maks;
} stab_eksen_t;

static stab_eksen_t s_stab[ROL_SAYISI];
static bool s_stab_acik;      /* en az bir eksen stabilize ediliyor        */
static bool s_kilitli;        /* hedef kilitlendi mi                       */
static int64_t s_perf_bas_us;

/*
 * MUDAHALE (sartname): "Anten hareket esnasinda baska bir aciya
 * yonlendirilebilecek olup tekrardan hedefe yonelim suresi 8s olacaktir."
 *
 * Otomatik modda jog/git komutu gelirse stabilizasyonu askiya aliyoruz,
 * operator birakinca yeniden devreye alip hedefe kilitlenme suresini
 * OLCUYORUZ. Olculen sure telemetride ve log'da; 8 sn siniri gecti mi
 * bilgisi jurinin onunde gosterilecek kanittir.
 */
static bool     s_mudahale;
static int64_t  s_kilitlenme_bas_us;
static bool     s_kilitlenme_olculuyor;
static uint32_t s_kilitlenme_ms;
static bool     s_kilitlenme_ok;

/* Mudahale testi: sistemi kendi kendine kaydirip geri donmesini olcer. */
static struct {
    bool    aktif;
    int     rol;
    int64_t bitis_us;      /* kaydirmanin ne zaman biteceği */
} s_mtest;

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

    /* Taramanin TAMAMINI kaydet: arayuz hepsini listeleyecek. */
    for (int t = 0; t < tarama.sayi && s_bulunan_sayi < HUT_MAKS_CIHAZ; t++) {
        s_bulunan[s_bulunan_sayi].hat   = (uint8_t)hat;
        s_bulunan[s_bulunan_sayi].adres = tarama.adres[t];
        s_bulunan_sayi++;
    }

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
    s_eksen_sayi   = 0;
    s_bulunan_sayi = 0;

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

/*
 * Iki eksenli otomatik takip.
 *
 * Tek eksen icin prensip: hedef aci kilitlenir, IMU o ekseni temsil eden
 * aciyi okur, fark (hata) Fuzzy-PI'ye verilir, cikan hiz komutu motora gider.
 * Iki eksende ayni sey bagimsiz iki kontrolcuyle yapiliyor -- azimut ve
 * elevasyon birbirini beklemiyor.
 *
 * Neden bagimsiz: eksenler mekanik olarak birbirine bagli degil ve platform
 * hareket zarfi kucuk (+-8 derece). Cakisma terimlerini modellemek bu olcekte
 * olculebilir kazanc getirmiyor, ama iki ayri dongu tezgahta ayri ayri
 * ayarlanabildigi icin cok daha kolay.
 */

/* Hangi IMU acisi bu ekseni temsil ediyor? Mekanik montaja bagli, ayardan
   okunuyor ("Eksen Tani" bunu olcup yaziyor). */
static float olcum_oku(int rol)
{
    uint8_t eksen = g_cfg.olcum_ekseni[(rol >= 0 && rol < ROL_SAYISI) ? rol : 0];

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

/* Olculen aci 360 derecede sariyor mu? Yalnizca yaw icin evet. */
static bool olcum_sarmali(int rol)
{
    return g_cfg.olcum_ekseni[(rol >= 0 && rol < ROL_SAYISI) ? rol : 0]
           == OLCUM_YAW;
}

static const char *rol_adi(int rol)
{
    return (rol == ROL_AZ) ? "yatay" : "dikey";
}

/* Sistemi durdurmayan sorun: operator gorsun ama is durmasin. */
static void uyar(hata_kodu_t kod)
{
    s_uyari    = kod;
    s_uyari_us = esp_timer_get_time();
}

static void perf_sifirla(void)
{
    for (int r = 0; r < ROL_SAYISI; r++) {
        s_stab[r].hata_kare_toplam = 0.0;
        s_stab[r].hata_n           = 0;
        s_stab[r].hata_maks        = 0.0f;
    }
    s_perf_bas_us = esp_timer_get_time();
}

/*
 * Hedefi kilitle: o anki acilar korunacak acilar olur.
 *
 * Yarisma senaryosu tam olarak bu: lazer hedef cemberin merkezine getirilir,
 * operator kilitler, sonra platform hareket etmeye baslar.
 */
static void hedef_kilitle(void)
{
    int64_t simdi = esp_timer_get_time();

    for (int r = 0; r < ROL_SAYISI; r++) {
        int i = eksen_bul_rol(r);

        s_stab[r].hedef       = olcum_oku(r);
        s_stab[r].olculen     = s_stab[r].hedef;
        s_stab[r].hata        = 0.0f;
        s_stab[r].sps         = 0.0f;
        s_stab[r].merkez_adim = (i >= 0) ? s_eksen[i].adim : 0;
        kontrol_sifirla(&s_stab[r].kontrol);

        s_stab[r].ters_us     = simdi;
        s_stab[r].ters_sapma  = 0.0f;
        s_stab[r].ters_hata   = 0.0f;
        s_stab[r].ters_sayac  = 0;

        ESP_LOGI(TAG, "kilit %s: %.2f derece (%s ekseni)", rol_adi(r),
                 s_stab[r].hedef, olcum_adi(g_cfg.olcum_ekseni[r]));
    }
    s_kilitli = true;
    perf_sifirla();
}

/* Tek eksenin bir kontrol dilimi. Guvenlik ihlalinde false doner. */
static bool stab_eksen_calistir(int rol, float dt, int64_t simdi_us)
{
    stab_eksen_t *st = &s_stab[rol];
    int i = eksen_bul_rol(rol);

    if (i < 0) {
        st->aktif = false;
        return true;             /* surucu yok: bu eksen sessizce atlanir */
    }

    eksen_t *e = &s_eksen[i];

    st->olculen = olcum_oku(rol);
    st->hata    = st->hedef - st->olculen;

    if (olcum_sarmali(rol)) {
        if (st->hata >  180.0f) st->hata -= 360.0f;
        if (st->hata < -180.0f) st->hata += 360.0f;
    }

    float mutlak = fabsf(st->hata);

    /* performans birikimi */
    st->hata_kare_toplam += (double)mutlak * (double)mutlak;
    st->hata_n++;
    if (mutlak > st->hata_maks) {
        st->hata_maks = mutlak;
    }

    /* --- Guvenlik 1: sapma siniri --- */
    if (mutlak > g_cfg.stab_hata_sinir) {
        ESP_LOGE(TAG, "%s: sapma siniri asildi (%.1f derece)", rol_adi(rol),
                 st->hata);
        hataya_dus(HATA_SAPMA);
        return false;
    }

    float reduktor = rol_reduktor(rol);
    float adim_der = EKSEN_MOTOR_DERECE_ADIM / reduktor;
    float merkez   = (float)st->merkez_adim * adim_der;
    float sapma    = eksen_derece(e, reduktor) - merkez;

    /*
     * --- Guvenlik 2: TERS YON ---
     * Polarite dogruysa eksen ilerledikce hata kuculur. Ikisi ayni yonde
     * degisiyorsa geri besleme pozitiftir: motor hatayi buyuterek kacar.
     * Bunu "limit hatasi" diye bildirmek operatoru yanlis yere bakmaya iter.
     * 200 ms araliklarla ve ust uste 3 kez istiyoruz -- elle egmek de hatayi
     * buyutur ama eksen hareketiyle tutarli bir iliski uretmez.
     */
    if (simdi_us - st->ters_us >= 200000) {
        float d_sapma = sapma - st->ters_sapma;
        float d_hata  = mutlak - fabsf(st->ters_hata);

        if (fabsf(d_sapma) > 0.5f && d_hata > 0.1f && mutlak > 2.0f) {
            st->ters_sayac++;
        } else {
            st->ters_sayac = 0;
        }
        st->ters_sapma = sapma;
        st->ters_hata  = st->hata;
        st->ters_us    = simdi_us;

        if (st->ters_sayac >= 3) {
            ESP_LOGE(TAG, "%s TERS YON: eksen %.1f derece dondu ama hata "
                     "%.1f dereceye BUYUDU", rol_adi(rol), sapma, st->hata);
            ESP_LOGE(TAG, "Yapilacak: Cihazlar sekmesinde bu eksen icin");
            ESP_LOGE(TAG, "'Yonu Ters' deyin ve hedefi tekrar kilitleyin.");
            ESP_LOGE(TAG, "Ikinci ihtimal: olcum ekseni yanlis (%s secili).",
                     olcum_adi(g_cfg.olcum_ekseni[rol]));
            hataya_dus(HATA_TERS_YON);
            return false;
        }
    }

    /* --- Guvenlik 3: calisma penceresi --- */
    if (fabsf(sapma) > g_cfg.stab_pencere) {
        ESP_LOGE(TAG, "%s: calisma penceresi disina cikildi (%.1f derece, "
                 "sinir %.0f)", rol_adi(rol), sapma, g_cfg.stab_pencere);
        hataya_dus(HATA_LIMIT);
        return false;
    }

    st->sps = kontrol_hesapla(&st->kontrol, st->hata, dt, reduktor);
    eksen_hiz(e, st->sps);
    st->aktif = true;
    return true;
}

/* Otomatik takibi baslat/bitir. */
static void otomatik_ac(bool ac)
{
    if (!ac) {
        for (int r = 0; r < ROL_SAYISI; r++) {
            s_stab[r].aktif = false;
            s_stab[r].sps   = 0.0f;
        }
        s_stab_acik = false;
        s_mudahale  = false;
        s_kilitlenme_olculuyor = false;
        s_mtest.aktif = false;
        hepsini_durdur();
        if (s_mod == MOD_STAB) {
            s_mod = MOD_BOSTA;
        }
        ESP_LOGI(TAG, "otomatik takip kapandi");
        return;
    }

    if (!s_imu.gecerli) {
        s_hata = HATA_IMU_YOK;
        ESP_LOGW(TAG, "IMU yok, otomatik moda gecilemez");
        return;
    }
    if (s_eksen_sayi == 0) {
        s_hata = HATA_SURUCU_YOK;
        ESP_LOGW(TAG, "surucu yok, otomatik moda gecilemez");
        return;
    }
    if (!g_cfg.gyro_kalibre) {
        ESP_LOGW(TAG, "gyro kalibre edilmemis -- yaw hizla kayacak");
        uyar(HATA_IMU_YOK);
    }

    /* Hedef daha once kilitlenmediyse simdiki aci hedef olur. */
    if (!s_kilitli) {
        hedef_kilitle();
    } else {
        for (int r = 0; r < ROL_SAYISI; r++) {
            kontrol_sifirla(&s_stab[r].kontrol);
        }
        perf_sifirla();
    }

    s_stab_acik = true;
    s_mudahale  = false;
    s_mod = MOD_STAB;

    ESP_LOGI(TAG, "OTOMATIK TAKIP ACIK");
    for (int r = 0; r < ROL_SAYISI; r++) {
        int i = eksen_bul_rol(r);
        ESP_LOGI(TAG, "  %s: %s  hedef=%.2f  olcum=%s  reduktor=%.2f",
                 rol_adi(r), (i >= 0) ? "surucu var" : "SURUCU YOK",
                 s_stab[r].hedef, olcum_adi(g_cfg.olcum_ekseni[r]),
                 rol_reduktor(r));
    }
    ESP_LOGI(TAG, "  pencere=%.0f  sapma siniri=%.0f  maks hiz=%.0f adim/s  "
             "kilit siniri=%lu ms", g_cfg.stab_pencere, g_cfg.stab_hata_sinir,
             g_cfg.maks_sps, (unsigned long)g_cfg.kilit_sinir_ms);
}

/*
 * Operator otomatik modda elle mudahale etti: takibi askiya al.
 * Bu bir hata degil, sartnamenin istedigi bir yetenek.
 */
static void mudahale_basla(void)
{
    if (s_mudahale) {
        return;
    }
    s_mudahale = true;
    s_kilitlenme_olculuyor = false;

    for (int r = 0; r < ROL_SAYISI; r++) {
        s_stab[r].aktif = false;
        s_stab[r].sps   = 0.0f;
        kontrol_sifirla(&s_stab[r].kontrol);
    }
    ESP_LOGI(TAG, "mudahale: takip askiya alindi");
}

/* Operator birakti: takibi geri al, hedefe kilitlenme suresini olcmeye basla. */
static void mudahale_bitir(void)
{
    if (!s_mudahale) {
        return;
    }
    s_mudahale = false;

    for (int r = 0; r < ROL_SAYISI; r++) {
        kontrol_sifirla(&s_stab[r].kontrol);
    }

    s_kilitlenme_bas_us    = esp_timer_get_time();
    s_kilitlenme_olculuyor = true;
    s_kilitlenme_ms        = 0;
    ESP_LOGI(TAG, "mudahale bitti, yeniden kilitlenme olculuyor (sinir %lu ms)",
             (unsigned long)g_cfg.kilit_sinir_ms);
}

/*
 * Yeniden kilitlenme olcumu.
 *
 * Kriter: TUM takili eksenlerin hatasi olu bandin iki katinin altina insin.
 * Olu bandin kendisini kullanmak olcumu gurultuye acik yapardi; iki kati
 * "operator gozuyle hedefte" demek icin makul bir esik.
 */
static void kilitlenme_izle(int64_t simdi_us)
{
    if (!s_kilitlenme_olculuyor) {
        return;
    }

    bool  hepsi_hedefte = true;
    int   sayilan = 0;
    float esik = g_cfg.olu_bant * 2.0f;

    for (int r = 0; r < ROL_SAYISI; r++) {
        if (eksen_bul_rol(r) < 0) {
            continue;
        }
        sayilan++;
        if (fabsf(s_stab[r].hata) > esik) {
            hepsi_hedefte = false;
        }
    }

    uint32_t gecen = (uint32_t)((simdi_us - s_kilitlenme_bas_us) / 1000);

    if (sayilan > 0 && hepsi_hedefte) {
        s_kilitlenme_ms        = gecen;
        s_kilitlenme_ok        = (gecen <= g_cfg.kilit_sinir_ms);
        s_kilitlenme_olculuyor = false;
        ESP_LOGI(TAG, "yeniden kilitlenme: %lu ms -- %s", (unsigned long)gecen,
                 s_kilitlenme_ok ? "GECTI" : "SINIR ASILDI");
        return;
    }

    /* Sinirin iki katini gectiyse olcumu bitir: sonsuza kadar "olculuyor"
       yazip beklemek operatoru yanlis bilgilendirir. */
    if (gecen > g_cfg.kilit_sinir_ms * 2) {
        s_kilitlenme_ms        = gecen;
        s_kilitlenme_ok        = false;
        s_kilitlenme_olculuyor = false;
        ESP_LOGW(TAG, "yeniden kilitlenme %lu ms icinde tamamlanmadi",
                 (unsigned long)gecen);
    }
}

/*
 * MUDAHALE TESTI -- sartnamenin 8 saniye maddesini tekrarlanabilir sekilde
 * gostermek icin.
 *
 * Akis: ekseni verilen aci kadar kaydir (mudahale gibi davran), 1 saniye
 * bekle, birak, yeniden kilitlenme suresini olc. Sonuc telemetride.
 */
static void mudahale_testi_basla(int rol, float derece)
{
    if (!s_stab_acik) {
        ESP_LOGW(TAG, "mudahale testi icin once otomatik moda gec");
        return;
    }
    int i = eksen_bul_rol(rol);
    if (i < 0) {
        ESP_LOGW(TAG, "%s rolunde surucu yok", rol_adi(rol));
        return;
    }
    if (derece < 1.0f) {
        derece = g_cfg.mudahale_test;
    }

    mudahale_basla();

    int32_t hedef_adim = s_eksen[i].adim +
                         eksen_derece_adim(derece, rol_reduktor(rol));
    eksen_git(&s_eksen[i], hedef_adim);

    s_mtest.aktif    = true;
    s_mtest.rol      = rol;
    s_mtest.bitis_us = 0;

    ESP_LOGI(TAG, "MUDAHALE TESTI: %s ekseni %.1f derece kaydiriliyor",
             rol_adi(rol), derece);
}

static void mudahale_testi_isle(int64_t simdi_us)
{
    if (!s_mtest.aktif) {
        return;
    }
    int i = eksen_bul_rol(s_mtest.rol);
    if (i < 0) {
        s_mtest.aktif = false;
        return;
    }

    /* Kaydirma bitti mi? Bittiyse 1 saniye bekle, sonra birak. */
    if (s_mtest.bitis_us == 0) {
        if (!s_eksen[i].git_aktif) {
            s_mtest.bitis_us = simdi_us + 1000000;
        }
        return;
    }
    if (simdi_us >= s_mtest.bitis_us) {
        s_mtest.aktif = false;
        mudahale_bitir();
    }
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
        i = eksen_bul_rol(k->a);
        if (i < 0) {
            s_hata = HATA_SURUCU_YOK;
            break;
        }
        /*
         * Otomatik moddayken jog gelmesi bir hata degil: sartname "anten
         * hareket esnasinda baska bir aciya yonlendirilebilecek" diyor.
         * Takibi askiya aliyoruz, birakildiginda geri aliyoruz.
         */
        if (s_stab_acik) {
            mudahale_basla();
        }
        {
            /* Arayuz hizi DERECE/S olarak yolluyor; adim/s'ye burada
               ceviriyoruz. Boylece disli orani degisince komutlar degismiyor. */
            float dps = (k->c > 0.0f) ? k->c : g_cfg.jog_dps;
            float sps = dps * rol_reduktor(k->a) / EKSEN_MOTOR_DERECE_ADIM;

            eksen_hiz(&s_eksen[i], (k->b < 0) ? -sps : sps);
            if (!s_stab_acik) {
                s_mod = MOD_JOG;
            }
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
        /* Otomatik modda: operator birakti, hedefe geri don ve sureyi olc. */
        if (s_stab_acik && s_mudahale) {
            mudahale_bitir();
        }
        break;

    case KOMUT_GIT:
        i = eksen_bul_rol(k->a);
        if (i < 0) {
            s_hata = HATA_SURUCU_YOK;
            break;
        }
        /* Otomatik modda aciya gitmek de mudahaledir: hareket bitince
           kendiliginden hedefe donulur. */
        if (s_stab_acik) {
            mudahale_basla();
            s_mtest.aktif    = true;     /* bitisini ayni makine izliyor */
            s_mtest.rol      = k->a;
            s_mtest.bitis_us = 0;
        } else {
            s_mod = MOD_GIT;
        }
        eksen_git(&s_eksen[i], eksen_derece_adim(k->c, rol_reduktor(k->a)));
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
        s_hata  = HATA_YOK;
        s_uyari = HATA_YOK;
        s_mod   = MOD_BOSTA;
        /*
         * Hatadan cikinca MANUEL'e dusuyoruz. Otomatik moda geri donmeyi
         * operatorun bilincli olarak istemesi gerekir: hatanin sebebi
         * duzelmediyse sistem aninda ayni hataya girer.
         */
        s_kmod = KMOD_MANUEL;
        ESP_LOGI(TAG, "hata temizlendi, manuel moda dusuldu");
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
        /* Eski tek eksen komutu: artik otomatik modu aciyor/kapatiyor. */
        otomatik_ac(k->a != 0);
        s_kmod = (k->a != 0) ? KMOD_OTOMATIK : KMOD_MANUEL;
        break;

    case KOMUT_STAB_KILIT:
        hedef_kilitle();
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

    case KOMUT_KULLANICI_MODU:
        if (k->a) {
            otomatik_ac(true);
            if (s_stab_acik) {
                s_kmod = KMOD_OTOMATIK;
            }
        } else {
            otomatik_ac(false);
            s_kmod = KMOD_MANUEL;
        }
        break;

    case KOMUT_TAKIPTEN_DUS:
        /* Sartname: "takip modunu durdurarak". Hedef kilidi KORUNUR ki
           operator tekrar otomatige basinca ayni noktaya donsun. */
        otomatik_ac(false);
        s_kmod = KMOD_MANUEL;
        ESP_LOGI(TAG, "takipten dusuruldu (hedef kilidi korunuyor)");
        break;

    case KOMUT_MUDAHALE_TEST:
        mudahale_testi_basla((k->a >= 0 && k->a < ROL_SAYISI) ? k->a : ROL_EL,
                             k->c);
        break;

    case KOMUT_PERF_SIFIRLA:
        perf_sifirla();
        s_kilitlenme_ms = 0;
        s_kilitlenme_ok = false;
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

    /*
     * Cihaz envanteri: hattaki her adres, ne oldugu ve ne ise yaradigi.
     * Tanima adrese bakarak yapiliyor; IMU/manyetometre icin gercekten
     * okunup okunmadigini imu_bilgi() soyluyor.
     */
    d.cihaz_sayisi = 0;
    for (int c = 0; c < s_bulunan_sayi && d.cihaz_sayisi < HUT_MAKS_CIHAZ; c++) {
        cihaz_kaydi_t *kayit = &d.cihaz[d.cihaz_sayisi++];
        uint8_t adres = s_bulunan[c].adres;
        uint8_t hat   = s_bulunan[c].hat;

        memset(kayit, 0, sizeof(*kayit));
        kayit->hat   = hat;
        kayit->adres = adres;
        kayit->eksen = -1;

        if (adres >= ADR_MOTOR_ILK && adres <= ADR_MOTOR_SON) {
            snprintf(kayit->tip, sizeof(kayit->tip), "M20 surucu");
            kayit->motor = true;

            for (int i = 0; i < s_eksen_sayi; i++) {
                if (s_eksen[i].adres == adres && s_eksen[i].hat == hat) {
                    kayit->eksen = (int8_t)i;
                    snprintf(kayit->gorev, sizeof(kayit->gorev), "%s eksen",
                             rol_adi(s_eksen[i].rol));
                    break;
                }
            }
            if (kayit->eksen < 0) {
                snprintf(kayit->gorev, sizeof(kayit->gorev), "acilamadi");
            }
        } else if (adres == bilgi->adres && bilgi->var) {
            snprintf(kayit->tip, sizeof(kayit->tip), "%s", bilgi->tip);
            snprintf(kayit->gorev, sizeof(kayit->gorev), "aci + gyro");
        } else if (bilgi->mag_var && adres == bilgi->mag_adres) {
            snprintf(kayit->tip, sizeof(kayit->tip), "%s", bilgi->mag_tip);
            snprintf(kayit->gorev, sizeof(kayit->gorev), "manyetometre");
        } else if (adres == ADR_IMU_A || adres == ADR_IMU_B) {
            snprintf(kayit->tip, sizeof(kayit->tip), "LSM6DS?");
            snprintf(kayit->gorev, sizeof(kayit->gorev), "kullanilmiyor");
        } else {
            snprintf(kayit->tip, sizeof(kayit->tip), "bilinmiyor");
            snprintf(kayit->gorev, sizeof(kayit->gorev), "kullanilmiyor");
        }
    }

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
    d.kmod     = s_kmod;
    d.uyari    = s_uyari;
    d.kilitli  = s_kilitli;
    d.mudahale = s_mudahale;

    for (int r = 0; r < ROL_SAYISI; r++) {
        int idx = eksen_bul_rol(r);
        float reduktor = rol_reduktor(r);
        float merkez = (float)s_stab[r].merkez_adim *
                       (EKSEN_MOTOR_DERECE_ADIM / reduktor);

        d.stab[r].aktif      = s_stab[r].aktif;
        d.stab[r].surucu_var = (idx >= 0);
        d.stab[r].hedef      = s_stab[r].hedef;
        d.stab[r].olculen    = s_stab[r].olculen;
        d.stab[r].hata       = s_stab[r].hata;
        d.stab[r].kp         = kontrol_son_kp(&s_stab[r].kontrol);
        d.stab[r].sps        = s_stab[r].sps;
        d.stab[r].sapma      = (idx >= 0)
                               ? eksen_derece(&s_eksen[idx], reduktor) - merkez
                               : 0.0f;
        d.stab[r].olcum      = g_cfg.olcum_ekseni[r];
    }

    d.kilitlenme_ms        = s_kilitlenme_ms;
    d.kilitlenme_ok        = s_kilitlenme_ok;
    d.kilitlenme_olculuyor = s_kilitlenme_olculuyor;

    /* Performans: iki eksenin birlesik RMS'i ve en buyuk hatasi. */
    {
        double kare = 0.0;
        uint32_t n = 0;
        float maks = 0.0f;

        for (int r = 0; r < ROL_SAYISI; r++) {
            kare += s_stab[r].hata_kare_toplam;
            n    += s_stab[r].hata_n;
            if (s_stab[r].hata_maks > maks) {
                maks = s_stab[r].hata_maks;
            }
        }
        d.perf_rms    = (n > 0) ? (float)sqrt(kare / (double)n) : 0.0f;
        d.perf_maks   = maks;
        d.perf_sure_s = (s_perf_bas_us > 0)
                        ? (uint32_t)((simdi - s_perf_bas_us) / 1000000) : 0;
    }

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
    for (int r = 0; r < ROL_SAYISI; r++) {
        o.hedef[r] = s_stab[r].hedef;
        o.hata[r]  = s_stab[r].hata;
        o.kp[r]    = kontrol_son_kp(&s_stab[r].kontrol);
    }
    o.kmod     = (uint8_t)s_kmod;
    o.mudahale = s_mudahale ? 1 : 0;

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

    /* --- otomatik takip: iki eksen bagimsiz --- */
    if (s_stab_acik && s_mod == MOD_STAB && !s_mudahale) {
        for (int r = 0; r < ROL_SAYISI; r++) {
            if (!stab_eksen_calistir(r, dt, simdi)) {
                break;          /* guvenlik ihlali: hata durumuna gecildi */
            }
        }
        kilitlenme_izle(simdi);
    }

    /* --- mudahale testi ve GIT tabanli mudahalenin bitisi --- */
    mudahale_testi_isle(simdi);

    /* --- uyarilar 5 saniye sonra kendiliginden dusuyor --- */
    if (s_uyari != HATA_YOK && simdi - s_uyari_us > 5000000) {
        s_uyari = HATA_YOK;
    }

    /* --- eksen limitine dayanma: hata degil UYARI --- */
    for (int r = 0; r < s_eksen_sayi; r++) {
        if (s_eksen[r].limit_vurdu) {
            uyar(HATA_LIMIT);
        }
        if (s_eksen[r].i2c_hata > 0 && s_eksen[r].i2c_hata < 20) {
            uyar(HATA_I2C);
        }
    }

    /* --- GIT bitti mi (manuel modda) --- */
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
    for (int r = 0; r < ROL_SAYISI; r++) {
        kontrol_sifirla(&s_stab[r].kontrol);
    }

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
