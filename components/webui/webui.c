/*
 * webui.c -- HTTP sunucu, WebSocket komut/telemetri kanali.
 */

#include "webui.h"
#include "config.h"
#include "hut_state.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_http_server.h"
#include "esp_log.h"

static const char *TAG = "web";

#define MAKS_ISTEMCI      4
#define TELEMETRI_MS      50     /* arayuz guncelleme: 20 Hz                */
#define LOG_FRAME_MS      100    /* log paketi: saniyede 10 paket           */
#define LOG_FRAME_ORNEK   32
#define CFG_KAYIT_MS      3000

/* Gomulu arayuz dosyalari. */
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");
extern const uint8_t app_js_start[]     asm("_binary_app_js_start");
extern const uint8_t app_js_end[]       asm("_binary_app_js_end");
extern const uint8_t style_css_start[]  asm("_binary_style_css_start");
extern const uint8_t style_css_end[]    asm("_binary_style_css_end");

static httpd_handle_t s_sunucu;
static int  s_istemci[MAKS_ISTEMCI];
static int  s_istemci_sayi;
static int  s_kontrol_fd = -1;   /* kontrol yetkisi ilk baglanan istemcide */

/* Listeye iki gorev dokunuyor: httpd (baglanti acilis/kapanis) ve telemetri
   (yayin). Kilit sadece listeyi koruyor -- gonderme islemi bloke edebildigi
   icin kilit disinda yapiliyor. */
static SemaphoreHandle_t s_liste_kilit;

static void liste_kilitle(void)
{
    if (s_liste_kilit) {
        xSemaphoreTake(s_liste_kilit, portMAX_DELAY);
    }
}

static void liste_birak(void)
{
    if (s_liste_kilit) {
        xSemaphoreGive(s_liste_kilit);
    }
}

/* ---------------------------------------------------------- istemci listesi */

static void istemci_ekle_kilitsiz(int fd)
{
    for (int i = 0; i < s_istemci_sayi; i++) {
        if (s_istemci[i] == fd) {
            return;
        }
    }
    if (s_istemci_sayi < MAKS_ISTEMCI) {
        s_istemci[s_istemci_sayi++] = fd;
    }
    if (s_kontrol_fd < 0) {
        s_kontrol_fd = fd;
        ESP_LOGI(TAG, "kontrol yetkisi fd=%d", fd);
    }
}

static void istemci_cikar_kilitsiz(int fd)
{
    for (int i = 0; i < s_istemci_sayi; i++) {
        if (s_istemci[i] != fd) {
            continue;
        }
        s_istemci[i] = s_istemci[--s_istemci_sayi];
        break;
    }

    if (fd != s_kontrol_fd) {
        return;
    }

    /*
     * Kontrol eden istemci koptu. Suren elle hareketi DURDUR: tarayici
     * kapandigi icin "birak" komutu hic gelmeyebilir ve motor sonsuza kadar
     * doner. Stabilizasyon ise kasitli olarak devam eder -- kendi basina
     * calismasi gereken sey o.
     */
    komut_t k = { .tip = KOMUT_DUR };
    hut_komut_gonder(&k);

    s_kontrol_fd = (s_istemci_sayi > 0) ? s_istemci[0] : -1;
    ESP_LOGW(TAG, "kontrol istemcisi koptu, hareket durduruldu");
}

static void istemci_ekle(int fd)
{
    liste_kilitle();
    istemci_ekle_kilitsiz(fd);
    liste_birak();
}

static void istemci_cikar(int fd)
{
    liste_kilitle();
    istemci_cikar_kilitsiz(fd);
    liste_birak();
}

static void yayinla(const char *veri, size_t uzunluk)
{
    int kopya[MAKS_ISTEMCI];
    int adet;

    httpd_ws_frame_t paket = {
        .final   = true,
        .type    = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)veri,
        .len     = uzunluk,
    };

    liste_kilitle();
    adet = s_istemci_sayi;
    memcpy(kopya, s_istemci, sizeof(int) * (size_t)adet);
    liste_birak();

    for (int i = 0; i < adet; i++) {
        if (httpd_ws_send_frame_async(s_sunucu, kopya[i], &paket) != ESP_OK) {
            istemci_cikar(kopya[i]);
        }
    }
}

/* ------------------------------------------------------------ komut ayristir */

/*
 * Minik JSON okuyucu.
 *
 * Neden cJSON degil: IDF v6'da 'json' bileseni cekirdekten cikarildi ve
 * managed component olarak ekleniyor. Bizim komutlarimiz tek seviyeli ve
 * sabit semali ({"c":"jog","rol":0,...}), bu yuzden bagimlilik eklemek yerine
 * anahtar arayan 40 satir yaziyoruz. Bilinmeyen alanlar sessizce atlaniyor.
 */
static const char *alan_bul(const char *metin, const char *ad)
{
    char desen[24];
    int n = snprintf(desen, sizeof(desen), "\"%s\"", ad);
    const char *p = metin;

    /*
     * Anahtar ile ayni yaziyi tasiyan bir DEGER'e takilmamak icin ardindan
     * ':' gelmesini sart kosuyoruz. Ornek: {"c":"rol","rol":1} icinde "rol"
     * iki kez geciyor ve ilki degerin kendisi.
     */
    while ((p = strstr(p, desen)) != NULL) {
        const char *q = p + n;
        while (*q == ' ') {
            q++;
        }
        if (*q == ':') {
            q++;
            while (*q == ' ') {
                q++;
            }
            return q;
        }
        p += n;
    }
    return NULL;
}

static bool metin_alan(const char *metin, const char *ad, char *cikis, int boyut)
{
    const char *p = alan_bul(metin, ad);

    if (!p || *p != '"') {
        return false;
    }
    p++;

    int i = 0;
    while (*p && *p != '"' && i < boyut - 1) {
        cikis[i++] = *p++;
    }
    cikis[i] = 0;
    return true;
}

static float sayi(const char *metin, const char *ad, float varsayilan)
{
    const char *p = alan_bul(metin, ad);

    if (!p) {
        return varsayilan;
    }
    if (*p == 't') return 1.0f;      /* true  */
    if (*p == 'f') return 0.0f;      /* false */
    if (*p != '-' && *p != '+' && *p != '.' && (*p < '0' || *p > '9')) {
        return varsayilan;
    }
    return strtof(p, NULL);
}

/* Rol hem sayi ("rol":1) hem metin ("rol":"el") olarak gelebilir. */
static int rol_coz(const char *metin, const char *ad)
{
    char s[8];

    if (metin_alan(metin, ad, s, sizeof(s))) {
        if (strcmp(s, "az") == 0) return ROL_AZ;
        if (strcmp(s, "el") == 0) return ROL_EL;
        return -1;
    }
    const char *p = alan_bul(metin, ad);
    if (!p || *p < '0' || *p > '9') {
        return -1;
    }
    return *p - '0';
}

static bool komut_ayristir(const char *j, komut_t *k)
{
    char komut[20];

    memset(k, 0, sizeof(*k));

    if (!metin_alan(j, "c", komut, sizeof(komut))) {
        return false;
    }

    if (strcmp(komut, "tara") == 0) {
        k->tip = KOMUT_TARA;
    } else if (strcmp(komut, "rol") == 0) {
        /* "i" = telemetrideki eksen sirasi. Adres kullanilamaz: iki surucu
           de 0x16 olabiliyor (ayri I2C hatlarinda). */
        k->tip = KOMUT_ROL_ATA;
        k->a   = (int32_t)sayi(j, "i", -1);
        k->b   = rol_coz(j, "rol");
    } else if (strcmp(komut, "tani") == 0) {
        k->tip = KOMUT_EKSEN_TANI;
        k->a   = (int32_t)sayi(j, "i", -1);
    } else if (strcmp(komut, "jog") == 0) {
        k->tip = KOMUT_JOG;
        k->a   = rol_coz(j, "rol");
        k->b   = (int32_t)sayi(j, "yon", 1);
        k->c   = sayi(j, "sps", 0);
    } else if (strcmp(komut, "jog_dur") == 0) {
        k->tip = KOMUT_JOG_DUR;
        k->a   = rol_coz(j, "rol");
    } else if (strcmp(komut, "git") == 0) {
        k->tip = KOMUT_GIT;
        k->a   = rol_coz(j, "rol");
        k->c   = sayi(j, "derece", 0);
    } else if (strcmp(komut, "dur") == 0) {
        k->tip = KOMUT_DUR;
    } else if (strcmp(komut, "estop") == 0) {
        k->tip = KOMUT_ACIL_DUR;
    } else if (strcmp(komut, "hata_sil") == 0) {
        k->tip = KOMUT_HATA_SIL;
    } else if (strcmp(komut, "ref") == 0) {
        k->tip = KOMUT_REFERANS;
        k->a   = rol_coz(j, "rol");
    } else if (strcmp(komut, "serbest") == 0) {
        k->tip = KOMUT_SERBEST;
        k->a   = rol_coz(j, "rol");
    } else if (strcmp(komut, "gyro_kalibre") == 0) {
        k->tip = KOMUT_GYRO_KALIBRE;
    } else if (strcmp(komut, "mag_kalibre") == 0) {
        k->tip = KOMUT_MAG_KALIBRE;
        k->a   = (int32_t)sayi(j, "on", 1);
    } else if (strcmp(komut, "stab") == 0) {
        k->tip = KOMUT_STAB;
        k->a   = (int32_t)sayi(j, "on", 0);
        k->b   = rol_coz(j, "rol");
    } else if (strcmp(komut, "kilit") == 0) {
        k->tip = KOMUT_STAB_KILIT;
    } else if (strcmp(komut, "yon_ters") == 0) {
        k->tip = KOMUT_YON_TERS;
        k->a   = rol_coz(j, "rol");
    } else if (strcmp(komut, "log") == 0) {
        k->tip = KOMUT_LOG;
        k->a   = (int32_t)sayi(j, "on", 0);
    } else if (strcmp(komut, "param") == 0) {
        k->tip = KOMUT_PARAM;
        k->c   = sayi(j, "v", 0);
        if (!metin_alan(j, "k", k->anahtar, sizeof(k->anahtar))) {
            return false;
        }
    } else {
        return false;
    }

    return true;
}

/* ------------------------------------------------------------- ws isleyici */

static esp_err_t ws_isle(httpd_req_t *istek)
{
    if (istek->method == HTTP_GET) {
        istemci_ekle(httpd_req_to_sockfd(istek));
        ESP_LOGI(TAG, "websocket acildi (fd=%d)", httpd_req_to_sockfd(istek));
        return ESP_OK;
    }

    httpd_ws_frame_t paket = { 0 };
    esp_err_t hata = httpd_ws_recv_frame(istek, &paket, 0);
    if (hata != ESP_OK) {
        return hata;
    }
    if (paket.type == HTTPD_WS_TYPE_CLOSE) {
        istemci_cikar(httpd_req_to_sockfd(istek));
        return ESP_OK;
    }
    if (paket.type != HTTPD_WS_TYPE_TEXT || paket.len == 0 || paket.len > 512) {
        return ESP_OK;
    }

    uint8_t tampon[513];
    paket.payload = tampon;
    hata = httpd_ws_recv_frame(istek, &paket, sizeof(tampon) - 1);
    if (hata != ESP_OK) {
        return hata;
    }
    tampon[paket.len] = 0;

    int fd = httpd_req_to_sockfd(istek);
    istemci_ekle(fd);

    komut_t k;
    if (!komut_ayristir((const char *)tampon, &k)) {
        const char *cevap = "{\"err\":\"komut anlasilmadi\"}";
        httpd_ws_frame_t c = { .final = true, .type = HTTPD_WS_TYPE_TEXT,
                               .payload = (uint8_t *)cevap, .len = strlen(cevap) };
        return httpd_ws_send_frame(istek, &c);
    }

    /* Izleyici istemci sadece bakar. Acil durdurma herkese acik: guvenlik
       komutunu yetki kontrolune takmak yanlis olur. */
    if (fd != s_kontrol_fd && k.tip != KOMUT_ACIL_DUR) {
        const char *cevap = "{\"err\":\"kontrol yetkisi baska istemcide\"}";
        httpd_ws_frame_t c = { .final = true, .type = HTTPD_WS_TYPE_TEXT,
                               .payload = (uint8_t *)cevap, .len = strlen(cevap) };
        return httpd_ws_send_frame(istek, &c);
    }

    hut_komut_gonder(&k);

    char cevap[64];
    int n = snprintf(cevap, sizeof(cevap), "{\"ack\":%d}", (int)k.tip);
    httpd_ws_frame_t c = { .final = true, .type = HTTPD_WS_TYPE_TEXT,
                           .payload = (uint8_t *)cevap, .len = n };
    return httpd_ws_send_frame(istek, &c);
}

/* --------------------------------------------------------- statik dosyalar */

static esp_err_t dosya_gonder(httpd_req_t *istek, const uint8_t *bas,
                              const uint8_t *son, const char *tip)
{
    httpd_resp_set_type(istek, tip);
    return httpd_resp_send(istek, (const char *)bas, son - bas - 1);
}

static esp_err_t kok_isle(httpd_req_t *r)
{
    return dosya_gonder(r, index_html_start, index_html_end, "text/html");
}

static esp_err_t js_isle(httpd_req_t *r)
{
    return dosya_gonder(r, app_js_start, app_js_end, "application/javascript");
}

static esp_err_t css_isle(httpd_req_t *r)
{
    return dosya_gonder(r, style_css_start, style_css_end, "text/css");
}

/* Tarayici her acilista istiyor; simgemiz yok, 404 log'u kirletmesin. */
static esp_err_t favicon_isle(httpd_req_t *r)
{
    httpd_resp_set_status(r, "204 No Content");
    return httpd_resp_send(r, NULL, 0);
}

static esp_err_t info_isle(httpd_req_t *r)
{
    char tampon[1024];
    hut_cfg_json(&g_cfg, tampon, sizeof(tampon));
    httpd_resp_set_type(r, "application/json");
    return httpd_resp_send(r, tampon, strlen(tampon));
}

/* -------------------------------------------------------- telemetri gorevi */

static const char *mod_adi(sistem_modu_t m)
{
    switch (m) {
    case MOD_BOSTA:   return "bosta";
    case MOD_JOG:     return "jog";
    case MOD_GIT:     return "git";
    case MOD_STAB:    return "stab";
    case MOD_KALIBRE: return "kalibre";
    case MOD_HATA:    return "hata";
    }
    return "?";
}

static const char *hata_adi(hata_kodu_t h)
{
    switch (h) {
    case HATA_YOK:              return "yok";
    case HATA_I2C:              return "i2c";
    case HATA_IMU_YOK:          return "imu yok";
    case HATA_IMU_ZAMAN_ASIMI:  return "imu zaman asimi";
    case HATA_LIMIT:            return "limit";
    case HATA_SAPMA:            return "sapma siniri";
    case HATA_SURUCU_YOK:       return "surucu yok";
    case HATA_ACIL_DURDURMA:    return "acil durdurma";
    }
    return "?";
}

static void telemetri_gonder(void)
{
    static char tampon[1600];
    sistem_durum_t d;
    int n = 0;

    hut_durum_oku(&d);

    n += snprintf(tampon + n, sizeof(tampon) - n,
        "{\"t\":%lld,\"mod\":\"%s\",\"hata\":\"%s\","
        "\"imu\":{\"var\":%d,\"adres\":%d,\"mag\":\"%s\","
        "\"r\":%.2f,\"p\":%.2f,\"y\":%.2f,\"pusula\":%.1f,\"kalibre\":%d,"
        "\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f,"
        "\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f},",
        (long long)(d.t_us / 1000), mod_adi(d.mod), hata_adi(d.hata),
        d.imu_var ? 1 : 0, d.imu_adres, d.mag_tip,
        d.durus.roll, d.durus.pitch, d.durus.yaw,
        isnan(d.durus.pusula) ? -1.0f : d.durus.pusula,
        d.durus.kalibre ? 1 : 0,
        d.imu.gyro[0], d.imu.gyro[1], d.imu.gyro[2],
        d.imu.ivme[0], d.imu.ivme[1], d.imu.ivme[2]);

    n += snprintf(tampon + n, sizeof(tampon) - n, "\"eksen\":[");
    for (int i = 0; i < d.surucu_sayisi && n < (int)sizeof(tampon) - 200; i++) {
        n += snprintf(tampon + n, sizeof(tampon) - n,
            "%s{\"i\":%d,\"hat\":%d,\"adres\":%d,\"rol\":%d,\"yon\":%d,"
            "\"adim\":%ld,\"der\":%.2f,\"sps\":%.1f,\"hata\":%lu}",
            i ? "," : "", i, d.eksen[i].hat,
            d.eksen[i].adres, d.eksen[i].rol, d.eksen[i].yon,
            (long)d.eksen[i].adim, d.eksen[i].derece, d.eksen[i].sps,
            (unsigned long)d.eksen[i].i2c_hata);
    }
    n += snprintf(tampon + n, sizeof(tampon) - n, "],");

    n += snprintf(tampon + n, sizeof(tampon) - n,
        "\"stab\":{\"on\":%d,\"rol\":%d,\"hedef\":%.2f,\"olculen\":%.2f,"
        "\"e\":%.2f,\"kp\":%.2f,\"sps\":%.1f},"
        "\"sis\":{\"dongu\":%lu,\"asim\":%lu,\"log\":%d,\"satir\":%lu,"
        "\"dusen\":%lu,\"istemci\":%d}}",
        d.stab_acik ? 1 : 0, d.stab_rol, d.stab_hedef, d.stab_olculen,
        d.stab_hata, d.stab_kp, d.stab_sps,
        (unsigned long)d.dongu_us, (unsigned long)d.asim,
        d.log_acik ? 1 : 0, (unsigned long)d.log_satir,
        (unsigned long)hut_log_dusen(), s_istemci_sayi);

    yayinla(tampon, (size_t)n);
}

static void log_gonder(void)
{
    static log_orneklem_t ornekler[LOG_FRAME_ORNEK];
    static char tampon[4096];

    int adet = hut_log_oku(ornekler, LOG_FRAME_ORNEK);
    if (adet <= 0) {
        return;
    }

    int n = snprintf(tampon, sizeof(tampon), "{\"L\":[");
    for (int i = 0; i < adet && n < (int)sizeof(tampon) - 200; i++) {
        const log_orneklem_t *o = &ornekler[i];
        n += snprintf(tampon + n, sizeof(tampon) - n,
            "%s[%lld,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,"
            "%ld,%ld,%.1f,%.1f,%.3f,%.3f,%.2f]",
            i ? "," : "", (long long)o->t_us,
            o->gyro[0], o->gyro[1], o->gyro[2],
            o->ivme[0], o->ivme[1], o->ivme[2],
            o->roll, o->pitch, o->yaw,
            (long)o->adim[0], (long)o->adim[1],
            o->sps[0], o->sps[1], o->hedef, o->hata, o->kp);
    }
    n += snprintf(tampon + n, sizeof(tampon) - n, "]}");

    yayinla(tampon, (size_t)n);
}

static void telemetri_gorevi(void *arg)
{
    (void)arg;
    TickType_t son = xTaskGetTickCount();
    int sayac = 0;

    for (;;) {
        vTaskDelayUntil(&son, pdMS_TO_TICKS(TELEMETRI_MS));

        if (s_istemci_sayi > 0) {
            telemetri_gonder();
            if ((sayac % (LOG_FRAME_MS / TELEMETRI_MS)) == 0) {
                log_gonder();
            }
        }

        /*
         * NVS yazimi burada: flash islemi 10-50 ms surebiliyor ve gercek
         * zaman gorevinde olsa adim zamanlamasini bozardi.
         */
        if ((sayac % (CFG_KAYIT_MS / TELEMETRI_MS)) == 0 && hut_cfg_kirli_mi()) {
            if (hut_cfg_kaydet(&g_cfg)) {
                hut_cfg_temizle();
            }
        }
        sayac++;
    }
}

/* ----------------------------------------------------------------- kurulum */

esp_err_t webui_baslat(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();

    cfg.core_id        = 1;      /* gercek zaman gorevi Core 0'da yalniz kalsin */
    cfg.task_priority  = 5;
    cfg.stack_size     = 6144;
    cfg.max_open_sockets = MAKS_ISTEMCI + 2;
    cfg.lru_purge_enable = true;

    s_liste_kilit = xSemaphoreCreateMutex();

    esp_err_t hata = httpd_start(&s_sunucu, &cfg);
    if (hata != ESP_OK) {
        ESP_LOGE(TAG, "http sunucu baslamadi: %s", esp_err_to_name(hata));
        return hata;
    }

    httpd_uri_t yollar[] = {
        { .uri = "/",          .method = HTTP_GET, .handler = kok_isle  },
        { .uri = "/app.js",    .method = HTTP_GET, .handler = js_isle   },
        { .uri = "/style.css", .method = HTTP_GET, .handler = css_isle  },
        { .uri = "/api/info",  .method = HTTP_GET, .handler = info_isle },
        { .uri = "/favicon.ico", .method = HTTP_GET, .handler = favicon_isle },
        { .uri = "/ws",        .method = HTTP_GET, .handler = ws_isle,
          .is_websocket = true },
    };
    for (size_t i = 0; i < sizeof(yollar) / sizeof(yollar[0]); i++) {
        httpd_register_uri_handler(s_sunucu, &yollar[i]);
    }

    xTaskCreatePinnedToCore(telemetri_gorevi, "tlm", 5120, NULL, 4, NULL, 1);

    ESP_LOGI(TAG, "arayuz hazir: http://192.168.4.1");
    return ESP_OK;
}
