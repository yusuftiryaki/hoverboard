# Kaba Bağlantı Haritası — Otonom Yer Robotu (v0, prototip)

> Amaç: güç hattı + Pi + ESP32 + STM32 MCU + sensörlerin nasıl bağlanacağının
> üst düzey resmi. Pin numaraları başlangıç önerisi; ⚠️ işaretli her şey
> takmadan önce **multimetreyle / veri sayfasıyla doğrulanmalı**.

Tasarımın temel prensibi — **iki hız katmanı:**
- **Pi** = yavaş, akıllı sensörler (lokalizasyon + algı + planlama).
- **ESP32** = hızlı refleks + güvenlik (motor komutu, watchdog, E-stop, çarpma).
- Pi asla motorlarla doğrudan konuşmaz.

---

## 1. Üst düzey blok şema

```
                         ┌─────────────────────────────────────────┐
                         │          36V 10S Li-ion BATARYA          │
                         │              (hoverboard #1)             │
                         └───────────────┬─────────────────────────┘
                                    [+]  │  [-]───────── ORTAK GND ──────────┐
                                         │                                   │
                                  [30–40A SİGORTA]                           │
                                         │                                   │
                          ┌──────────────┴───────────────┐                   │
                          │                               │                  │
              (her zaman açık)                      [E-STOP kontaktör]        │
                          │                          NC mantar buton         │
                    [BUCK 36V→5V/5A]                       │                  │
                          │                        (basınca motor gücü keser)│
                     5V RAYI                               │                  │
                    │        │                     [Hoverboard STM32 MCU]     │
                 [Pi 4]      │                       │  36V motor gücü        │
                    │        │                       ├── 2× hub motor (faz)   │
                (USB kablo:  │                       └── 2× hall sensör       │
                 güç+veri)   │                             │                  │
                    │        └────► [ESP32] ◄── UART ───────┘ (3.3V, GND/TX/RX)│
                    │                  │  ▲                                   │
                    │  USB seri        │  │ E-stop sense (GPIO25)             │
                    └──────────────────┘  ├─ çarpma sensörü (refleks)         │
                                          └─ ultrasonik ×4 (refleks, opsiyon) │
                                                                              │
   ┌── Pi çevre birimleri (I2C / UART / CSI) ──┐                             │
   │  IMU (MPU6050, I2C)  ── direkte, motordan uzak                          │
   │  Manyetometre (QMC5883L, I2C) ── AYNI direkte, motordan >30cm ──────────┤
   │  GPS (NEO-6M, UART/USB-TTL) ── anten göğe, Pi'dan uzak                   │
   │  Pi Camera V2 (CSI şerit)                                               │
   └────────────────────────────────────────────┘                           │
                                                                             │
   HERKESİN GND'si tek noktada birleşir ───────────────────────────────────┘
```

---

## 2. Güç hattı

| Kaynak | → Hedef | Gerilim/Akım | Not |
|--------|---------|--------------|-----|
| Batarya [+] | Sigorta | 36V | **30–40A**, bataryanın hemen çıkışında |
| Sigorta | Dal 1: Buck girişi | 36V | Pi+ESP32 **her zaman açık** (E-stop'tan bağımsız) |
| Sigorta | Dal 2: E-stop kontaktör → MCU | 36V | Sadece **motor gücü** bu dalda |
| Buck çıkışı | Pi 4 | 5V / ≥5A | Pi tam yükte ~3A; buck'ta cimrilik yapma |
| Pi USB | ESP32 (5V/VIN) | 5V | Güç + veri tek kabloda (prototip kolaylığı) |
| Batarya [-] | **Ortak GND** | — | Buck, Pi, ESP32, MCU sinyal GND'si hepsi birleşir |

**E-stop neden sadece motoru kesiyor, her şeyi değil?**
Basınca Pi'ı da keserse SD kart bozulur ve ESP32 durumu raporlayamaz.
Bu yüzden E-stop yalnızca **36V motor gücünü** kontaktörle kesiyor; Pi + ESP32
ayakta kalıyor. Motorlar serbest dönüşe (coast) geçip duruyor — yer robotu
için en güvenli davranış, kaçış (runaway) fiziksel olarak imkânsız.

**MCU nasıl besleniyor? Ayrı besleme YOK.**
Hoverboard anakartının üzerinde kendi step-down regülatörü var; MCU'nun logic
gücünü 36V girişinden kendisi üretir. Yani MCU'ya ayrı buck çekmiyorsun —
36V anakarta gittiği sürece MCU beslenir. O 36V da E-stop dalından geldiği için:
- E-stop → MCU de söner → motorlar coast ile durur (aktif fren yok).
- Geri dönüş: hoverboard **güç butonuna tekrar basılmalı** (self-latch). İstersek
  ESP32'ye buton pad'lerini tetikletip bunu otomatikleştiririz.
- MOSFET köprüleri motor gücüyle aynı 36V barasında olduğundan "sadece motoru
  kes, MCU'yu ayakta tut" fiziksel olarak mümkün değil — bilinçli bir seçim.
- Flash'ta bataryanın bağlı olması gerekmesinin sebebi de bu (MCU gücünü 36V'tan alır).

**⚠️ Kararlar:**
- Kontaktör bobinini nasıl süreceğin: en basiti mantar butonun NC kontağı
  bobin/gücü doğrudan keser (akım düşükse). Yüksek akımda ayrı kontaktör.
- Pi'ı 5V GPIO header'dan mı USB-C'den mi besleyeceğin — ikisi de olur,
  GPIO header koruma devrelerini bypass eder ama robotta yaygın.

---

## 3. Veri / sinyal bağlantıları

### 3a. ESP32 ⟷ Hoverboard STM32 MCU (kritik, ⚠️ ölçülecek)
| ESP32 | Yön | MCU (yan kart konnektörü) | Not |
|-------|-----|---------------------------|-----|
| GPIO16 (RX2) | ◄── | MCU **TX** | çapraz bağlanır |
| GPIO17 (TX2) | ──► | MCU **RX** | çapraz bağlanır |
| GND | ─── | konnektör GND | ortak GND şart |
| — | ✗ | konnektör **15V** | **BAĞLAMA** — yakar |

- 115200 baud, ikisi de 3.3V mantık, seviye çevirici yok.
- Sol kablo = USART2, sağ = USART3. Hangisinin fiziksel hangisi olduğunu
  ve GND/15V/TX/RX sırasını **bringup-checklist.md adım 5** ile doğrula.

### 3b. ESP32 ⟷ Pi
| Bağlantı | Arayüz | Not |
|----------|--------|-----|
| USB kablo | UART0 (USB seri) + 5V güç | Tek kablo: hem veri hem ESP32 beslemesi |
| — | 115200, binary çerçeve protokolü | Pi'da `/dev/ttyUSB0` görünür |

> **Denge:** ESP32 gücünü Pi'dan aldığı için Pi ölürse ESP32 de ölür.
> Yedek: hoverboard firmware'inin kendi seri-timeout'u (komut gelmezse durur)
> + fiziksel E-stop. Daha sağlam istersek ESP32'yi buck'tan bağımsız besleyip
> Pi ile GPIO UART (ortak GND) kullanırız — yükseltme yolu.

### 3c. ESP32 refleks sensörleri (GPIO)
| Sensör | ESP32 pini | Durum | Not |
|--------|-----------|-------|-----|
| E-stop sense | GPIO25 | ✅ kodda var | NC kontak → GND, INPUT_PULLUP. **Açık devre = HIGH = DUR** (kopuk kablo da durdurur) |
| Çarpma sensörü | GPIO26 | ✅ kodda var | **NC kontak → GND, INPUT_PULLUP** — E-stop'la aynı fail-safe deseni. Bkz. aşağısı. |
| Ultrasonik ×4 | trig+echo ×4 (8 pin) | ⏳ envanterde YOK | Alım kararı verilmedi (wiring-map'te "opsiyon"). ⚠️ HC-SR04 echo **5V** → 3.3V'a bölücü gerekir |

**⚠️ Çarpma sensörü NC (normally-closed) bağlanmalı — NO değil.**
E-stop'la aynı mantık: kontak **kapalı = çarpma YOK = pin LOW**. Basılınca
**veya kablo koparsa** kontak açılır → pullup → **HIGH = engel var**. Yani kopuk
bir çarpma sensörü "engel yok" diye yalan söylemez, ileri gidişi reddeder —
arıza görünür olur, tehlikeli değil. NO bağlarsan bu tersine döner ve kopuk
kablo sessizce korumayı kapatır.

**Çarpma refleksi ne yapar (firmware):** ileri gidişi **veto** eder, geri ve
dönüşü serbest bırakır. Latch YOK — kontak bırakılınca (100 ms kararlı) kendi
kendine açılır, Pi'la el sıkışma gerekmez. Robot çarptığı şeyden kendi başına
geri çekilebilsin diye. E-stop'tan farkı bu: E-stop latch'li ve **her yönü** keser.

> ⚠️ **`BUMP_BLOCKS_POSITIVE_SPEED` tezgahta doğrulanmalı.** Firmware pozitif
> `speed`'in ileri olduğunu varsayıyor (Pi'ın eşlemesi öyle). Kartın mixer'ı ters
> çıkarsa veto **geriyi** kesip ileriyi serbest bırakır — yani robotu çarptığı
> şeye bindirir. Tekerlekler havadayken, `main.cpp`'deki sabitle birlikte kontrol et.

### 3d. Pi lokalizasyon/algı sensörleri
| Sensör | Arayüz | Pi bağlantısı | Not |
|--------|--------|---------------|-----|
| IMU (MPU6050, 6-eks) | I2C | SDA=GPIO2, SCL=GPIO3 | Dönme merkezine yakın idealdir |
| Manyetometre (QMC5883L) | I2C | aynı bus | **⚠️ direğe, motordan >30cm, hard/soft-iron kalibrasyon** |
| GPS (NEO-6M) | UART | USB-TTL öneri (`/dev/ttyUSB1`) | **⚠️ anten Pi'dan >20cm, USB3/HDMI gürültüsü GPS bandında** |
| Pi Camera V2 | CSI | kamera şerit konnektörü | Zemin segmentasyonu (adım 7) |

---

## 4. Topraklama ve mantık seviyeleri (atlanırsa hiçbir şey çalışmaz)

- **Tek ortak GND:** batarya eksi, buck GND, Pi GND, ESP32 GND, MCU sinyal
  GND'si hepsi tek noktada. UART'ın çalışması buna bağlı — ayrı GND'ler
  "veri geliyor ama saçma" olarak görünür, saatlerce aratır.
- **Mantık seviyeleri:** Pi, ESP32, STM32 MCU, IMU, manyetometre → hepsi 3.3V,
  doğrudan bağlanır.
- **İstisnalar (⚠️ seviye uyumu):**
  - HC-SR04 echo = **5V** → ESP32/Pi için gerilim bölücü (örn. 1kΩ/2kΩ).
  - NEO-6M TX genelde 3.3V ama modülüne göre değişir — **ölç**.

---

## 5. Hızlı referans — ne nereye

```
STM32 MCU  : 36V güç, 2× motor faz, 2× hall  → (hazır bağlı, dokunma)
             UART 3 tel (GND/TX/RX)          → ESP32 GPIO16/17
ESP32      : USB (güç+veri)                  → Pi
             GPIO25 E-stop, GPIO26 çarpma     → refleks
             (ultrasonik ×4 → eklenti)
Pi 4       : 5V buck girişi                  → güç
             I2C (GPIO2/3): IMU + manyetometre
             USB-TTL: GPS
             CSI: kamera
             USB: ESP32
Güç        : Batarya → sigorta → {buck→Pi/ESP32} + {E-stop→MCU}
```

---

## 6. Açık uçlar / karar bekleyenler
- [ ] Kontaktör modeli + E-stop bobin sürüş şekli (akıma göre).
- [ ] Manyetometre alımı (QMC5883L, ~100 TL) — 6-eksen IMU'nun eksik parçası.
- [ ] **Ultrasonik alınacak mı?** Katman ESP32'de olacak (karar 8) ama sensörler
      **envanterde yok ve alım listesinde de değil**. Alınacaksa önce karar:
      kaç adet, hangi açılara bakacak, echo bölücüsü nasıl. Montaj geometrisi
      belli olmadan "ön engel" mantığı yazılamaz.
- [ ] GPS: USB-TTL mü Pi donanım UART'ı mı (USB-TTL daha az dertli).
- [ ] Buck akım marjı (Pi 3A + ESP32 + sensörler → ≥5A buck).
