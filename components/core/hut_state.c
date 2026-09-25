/*
 * hut_state.c -- kuyruk, durum anlik goruntusu ve log halka tamponu.
 */

#include "hut_state.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"

static const char *TAG = "state";

#define KOMUT_KUYRUK_DERINLIK   16
#define LOG_TAMPON_ORNEKLEM     256   /* 100 Hz'de ~2.5 s pay */

static QueueHandle_t s_komut_q;

/* Durum anlik goruntusu: yazan tek gorev, okuyan birkac gorev. Kopyalama
   kisa (birkac yuz bayt) oldugu icin kritik bolum yeterli; mutex'e gerek yok
   ve gercek zaman gorevi hicbir zaman beklemiyor. */
static portMUX_TYPE   s_durum_kilit = portMUX_INITIALIZER_UNLOCKED;
static sistem_durum_t s_durum;

/* Log halkasi: tek yazan, tek okuyan. */
static portMUX_TYPE     s_log_kilit = portMUX_INITIALIZER_UNLOCKED;
static log_orneklem_t   s_log[LOG_TAMPON_ORNEKLEM];
static uint32_t         s_log_bas, s_log_son;
static uint32_t         s_log_dusen;

void hut_state_baslat(void)
{
    s_komut_q = xQueueCreate(KOMUT_KUYRUK_DERINLIK, sizeof(komut_t));
    if (!s_komut_q) {
        ESP_LOGE(TAG, "komut kuyrugu olusturulamadi");
    }
    memset(&s_durum, 0, sizeof(s_durum));
    s_log_bas = s_log_son = s_log_dusen = 0;
}

/* ------------------------------------------------------------------ komut */

bool hut_komut_gonder(const komut_t *k)
{
    if (!s_komut_q) {
        return false;
    }
    /* Web gorevi kuyruk dolu diye beklemesin: komutu dusur ve soyle. */
    if (xQueueSend(s_komut_q, k, 0) != pdTRUE) {
        ESP_LOGW(TAG, "komut kuyrugu dolu, komut %d dusuruldu", (int)k->tip);
        return false;
    }
    return true;
}

bool hut_komut_al(komut_t *k)
{
    if (!s_komut_q) {
        return false;
    }
    return xQueueReceive(s_komut_q, k, 0) == pdTRUE;
}

/* ------------------------------------------------------------------ durum */

void hut_durum_yaz(const sistem_durum_t *d)
{
    portENTER_CRITICAL(&s_durum_kilit);
    s_durum = *d;
    portEXIT_CRITICAL(&s_durum_kilit);
}

void hut_durum_oku(sistem_durum_t *d)
{
    portENTER_CRITICAL(&s_durum_kilit);
    *d = s_durum;
    portEXIT_CRITICAL(&s_durum_kilit);
}

/* -------------------------------------------------------------------- log */

void hut_log_yaz(const log_orneklem_t *o)
{
    portENTER_CRITICAL(&s_log_kilit);
    uint32_t sonraki = (s_log_son + 1) % LOG_TAMPON_ORNEKLEM;

    if (sonraki == s_log_bas) {
        /* Tampon dolu. En eskiyi dusurmek yerine YENIYI dusuruyoruz:
           boylece kayit bosluklari surekli ve sayilabilir kaliyor. */
        s_log_dusen++;
    } else {
        s_log[s_log_son] = *o;
        s_log_son = sonraki;
    }
    portEXIT_CRITICAL(&s_log_kilit);
}

int hut_log_oku(log_orneklem_t *dizi, int maks)
{
    int n = 0;

    portENTER_CRITICAL(&s_log_kilit);
    while (n < maks && s_log_bas != s_log_son) {
        dizi[n++] = s_log[s_log_bas];
        s_log_bas = (s_log_bas + 1) % LOG_TAMPON_ORNEKLEM;
    }
    portEXIT_CRITICAL(&s_log_kilit);
    return n;
}

void hut_log_temizle(void)
{
    portENTER_CRITICAL(&s_log_kilit);
    s_log_bas = s_log_son = 0;
    s_log_dusen = 0;
    portEXIT_CRITICAL(&s_log_kilit);
}

uint32_t hut_log_dusen(void)
{
    return s_log_dusen;
}
