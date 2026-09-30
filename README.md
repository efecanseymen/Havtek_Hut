# HAVTEK — Hareketli Uydu Terminali (v0.2)

Teknofest 2026 Hareketli Uydu Terminali yarışması için ESP-IDF tabanlı
Deneyap Kart v2 (ESP32-S3) yazılımı.

**v0.1'in amacıydı stabilizasyonu bitirmek değil, onu tasarlamak için gereken
ölçümleri toplamak.** Motorlar güvenle sürülüyor, IMU 100 Hz'de okunuyor,
her şey CSV olarak indirilebiliyor ve tek eksende deneysel bir stabilizasyon
denenebiliyor.

## Hızlı başlangıç

```bash
idf.py -p COM5 flash monitor
```

Kart açılınca `HAVTEK-HUT` adlı bir WiFi ağı yayınlar (parola `havtek2026`).
Bağlanıp tarayıcıdan **http://192.168.4.1** adresine gidin.

Açılışta kart hareketsizse gyro kalibrasyonu otomatik başlar (3 saniye).

## Ne var, ne yok

**Var:** I2C taraması ve cihaz keşfi, sürücü rol atama, "Eksen Tanı", jog ve
açıya git, hız rampası, yazılım limitleri, referans ayarlama, IMU okuma,
gyro/manyetometre kalibrasyonu, **Manuel/Otomatik mod, iki eksenli stabilizasyon**, müdahale ve 8 saniye testi, (PI ve Fuzzy-PI),
100 Hz log + CSV indirme, acil durdurma, hata yönetimi, seri komut satırı.

**Yok (sonraki sürümler):** hedef noktası ve
paralaks düzeltmesi, uydu (Türksat) hesabı, GPS, homing sensörü, dişli
boşluğu telafisi.

## Mimari

```
main/
  main.c        açılış sırası
  rt_task.c     gerçek zaman görevi (Core 0) — I2C hattının TEK sahibi
  cli.c         USB seri yedek komut satırı
components/
  core/         ortak tipler, parametreler (NVS), kuyruk + durum + log halkası
  i2c_hub/      tek I2C veri yolu: tarama, cihaz ekleme, kurtarma
  motor_m20/    M20 protokolü + blokemeyen eksen katmanı
  imu/          LSM6DS ailesi + manyetometre (tara-ve-tanı)
  ahrs/         gyro bias kalibrasyonu, roll/pitch için Kalman, yaw integrali
  control/      Fuzzy-PI ve sabit PI
  webui/        SoftAP + HTTP + WebSocket + gömülü arayüz
```

### Görevler

| Görev | Çekirdek | Periyot | İşi |
|---|---|---|---|
| `rt` | 0 | 1 ms | Komutlar, adım üretimi (her iki I2C hattı); her 10 ms'de IMU + kontrol + log |
| `httpd` | 1 | olay | HTTP ve WebSocket |
| `tlm` | 1 | 50 ms | Telemetri, log paketleri, NVS yazımı |
| `cli` | 1 | olay | Seri komut satırı |

Görevler arasında paylaşılan global değişken yok: komutlar kuyrukla gider,
durum kilitli anlık görüntüyle okunur.

### Neden tek I2C görevi

Hatta hem STM8 tabanlı M20 sürücüler (50 kHz) hem de IMU (400 kHz) var.
I2C'de aynı anda tek işlem yapılabilir; birden fazla görev hattı paylaşırsa
adım zamanlaması öngörülemez olur ve step motor sessizce adım kaçırır.
Her şey tek bir 1 ms ızgarada sıraya giriyor. Telemetrideki **döngü** alanı
en kötü dilim süresini, **aşım** alanı kaçırılan dilim sayısını gösterir.

## WebSocket protokolü

Komutlar (`ws://192.168.4.1/ws`):

```json
{"c":"jog","rol":"az","yon":1,"sps":150}   {"c":"jog_dur","rol":"az"}
{"c":"git","rol":"el","derece":10}          {"c":"dur"}   {"c":"estop"}
{"c":"ref","rol":"el"}                      {"c":"serbest","rol":"el"}
{"c":"tara"}  {"c":"tani","i":0}            {"c":"rol","i":0,"rol":"el"}
{"c":"gyro_kalibre"}                        {"c":"mag_kalibre","on":1}
{"c":"stab","on":1,"rol":"el"}              {"c":"kilit"}
{"c":"param","k":"kp_maks","v":10}          {"c":"log","on":1}
```

Telemetri 20 Hz, log paketleri 10 Hz (paket başına 32 örneklem).
Kontrol yetkisi ilk bağlanan istemcidedir; diğerleri yalnızca izler.
Acil durdurma her istemciye açıktır.

## Seri komutlar

`durum`, `tara`, `dur`, `estop`, `hatasil`, `jog az +`, `jog az stop`,
`git el 10`, `ref el`, `serbest el`, `gyro`, `mag on|off`,
`stab on el`, `kilit`, `log on|off`, `param kp_maks 10`

## Donanım notları

- I2C: SDA = GPIO47 (D10), SCL = GPIO21 (D11)
- M20 sürücü **STEP/DIR değil**: her adım için bir I2C paketi gidiyor,
  pratik tavan ~500 adım/s
- Yalnızca 8 durumlu yarım adım tablosu çalışıyor (400 adım = 1 tur,
  0.9°/adım). Tam adım tablosu donanımda denendi, yanlış çıktı.
- İki sürücü de fabrikadan 0x16 adresinde gelir ve **aynı hatta takılamaz**.
  İki çözüm var, ikisi de destekleniyor:
  - **İkinci I2C hattı** (SDA = GPIO40, SCL = GPIO39) — lehim gerektirmez,
    adım trafiği de iki hatta bölünür. `pins.h`'de `PIN_I2C2_*`; `-1` yaparsanız
    ikinci hat hiç kurulmaz.
  - **Lehim köprüsü** ile birini 0x17 yapmak (ADR1) — tek hatta ikisi birden.
- Kod motoru değil sürücüyü tanır; kimliği **(hat, adres)** ikilisidir. Hangi
  motorun bağlı olduğunu ya arayüzden seçersiniz ya da "Eksen Tanı" birkaç adım
  atıp IMU'da hangi açının değiştiğine bakar. Arayüz eksenleri adresle değil
  telemetrideki sırayla gösterir (iki sürücü aynı adreste olabilir).

## Manuel / Otomatik mod (v0.2)

Şartnamenin istediği iki kullanıcı modu. Arayüzde **Otomatik Mod** sekmesinden
seçilir.

| Mod | Ne yapar |
|---|---|
| **MANUEL** | Jog ve "açıya git" komutları uygulanır, sistem gittiği yerde bekler. Stabilizasyon kapalı. |
| **OTOMATİK** | Kilitlenen hedef sensörle korunur. **İki eksen birlikte**, bağımsız iki Fuzzy-PI döngüsüyle. |

### Müdahale (şartname: 8 saniye)

Otomatik moddayken jog veya "açıya git" vermek hata değildir — şartnamenin
istediği yetenektir: *"Anten hareket esnasında başka bir açıya
yönlendirilebilecek olup tekrardan hedefe yönelim süresi 8s olacaktır."*

Akış: komut gelince takip askıya alınır (`müdahale`), bıraktığınızda takip geri
alınır ve **hedefe yeniden kilitlenme süresi ölçülür**. Ölçüm arayüzde ve
log'da; sınır `kilit_sinir_ms` (varsayılan 8000).

Kilitlenme kriteri: takılı tüm eksenlerin hatası ölü bandın iki katının altına
insin.

### Takipten düşürme

"Takipten Düşür" otomatik takibi kapatır ama **hedef kilidini korur**; tekrar
OTOMATİK'e bastığınızda aynı noktaya döner.

### Otomatik mod testi — adım adım

1. IMU'nun **hareket eden gövdeye monteli** olduğundan emin olun. Masada duran
   IMU ile otomatik mod çalışmaz (motor döner, açı değişmez, sistem pencere
   korumasına çarpar).
2. Sistem hareketsizken **Gyro Kalibre**.
3. Cihazlar sekmesinde rolleri ve ölçüm eksenlerini kontrol edin.
4. Anteni istediğiniz yere getirin → **Hedefi Kilitle**.
5. Kayıt sekmesinden **Kaydı Başlat**.
6. **OTOMATİK**'e basın. Eksen durumu panosunda her eksen için `takip` yazmalı.
7. Düzeneği elinizle yavaşça eğin: motor ters yönde tepki vermeli, hata
   sıfıra dönmeli.
8. **Müdahale Testini Çalıştır** (varsayılan 15°). Sistem kendini kaydırır,
   bırakır, süreyi ölçer. Sonuç: `son yeniden kilitlenme: 3.42 s -> GEÇTİ`.
9. Performans kutusunda hata RMS ve en büyük hatayı izleyin; 5 dakika
   kesintisiz çalıştırıp bu sayıları rapora alın.
10. **Durdur** → **CSV İndir**.

### İlk denemede beklenen sorun: ters yön

Motorun artı yönü açının artı yönü değilse kontrolcü hatayı büyütür. Sistem
bunu ayrı bir hata olarak yakalar (`ters yon`) ve ne yapılacağını log'a yazar:
Cihazlar sekmesinde **Yönü Ters**, sonra hedefi tekrar kilitleyin. İkinci
olasılık, o eksenin **ölçüm ekseninin** yanlış seçilmiş olması.

### Hata / uyarı ayrımı

- **Uyarı** (sarı şerit, sistem çalışmaya devam eder): geçici I2C hatası,
  limite dayanma, gyro kalibre edilmemiş. 5 saniye sonra kendiliğinden düşer.
- **Hata** (kırmızı, motorlar durur): IMU zaman aşımı, sapma sınırı, ters yön,
  çalışma penceresi, sürücü kopması, acil durdurma.

Hatadan çıkış: üst şeritteki **Hatayı Temizle**. Sistem manuel moda düşer —
otomatiğe dönmeyi operatörün bilinçli olarak istemesi gerekir.

### Jog hızı artık derece/saniye

Arayüz adım/s değil **antende derece/s** konuşuyor; dişli oranını
değiştirdiğinizde komutlarınız aynı kalır. Dönüşümü firmware yapar
(`adım/s = derece/s × redüktör ÷ 0,9`).

## Toplanacak ölçümler

| Test | Ne yapılır | Ne öğreniriz |
|---|---|---|
| T-1 | 5 dk hareketsiz, log açık | Gyro sapması, gerçek yaw kayması |
| T-2 | ±90° tur, IMU ile sayaç karşılaştırması | Redüktör oranı, adım kaçırma |
| T-3 | Hız kademeli artırılır | Güvenli hız tavanı (5 V yetiyor mu) |
| T-4 | Yön değiştirmeli küçük hareketler | Dişli boşluğu |
| T-5 | Kare dalga jog | Yerleşme süresi, rampa ayarı |
| T-6 | Çanak elle sallanır | IMU gürültüsü, filtre kesimi |
| T-7 | Döngü ve aşım sayaçları | 100 Hz tutuyor mu, tek hat yetiyor mu |

## Bilinmeyenler

- IMU çipi donanımda doğrulanmadı. Sürücü tarayıp tanıyor; tanımadığı bir çip
  bulursa WHO_AM_I değerini log'a basar, sürücü ona göre güncellenir.
- Redüktör oranı varsayılan 3.6 (10:36). Tezgâhta doğrulanacak (T-2).
- IMU eksen yönelimi (hangi eksen roll/pitch) doğrulanmadı; gerekirse
  `ahrs.c` içindeki işaretler düzeltilecek.
