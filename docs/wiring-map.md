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
   ELEKTRONİK BESLEMESİ                 MEKANİK BESLEMESİ
   ┌──────────────────────┐             ┌─────────────────────────────────────┐
   │  POWERBANK 5V / ≥3A  │             │       36V 10S Li-ion BATARYA        │
   │   (her zaman açık)   │             │   (hoverboard #1, dolu 42V)         │
   └───────┬──────────────┘             └──────────────┬──────────────────────┘
       [+] │  [-]── ORTAK GND ──┐                 [+]  │  [-]── ORTAK GND ────┐
           │                    │                      │                      │
           │                    │              [20A/60V SİGORTA]              │
           │                    │                      │                      │
           │                    │               [E-STOP anahtarı]             │
           │                    │              (motor gücünü keser)           │
           │                    │                      │                      │
        [Pi 4]                  │            [Hoverboard STM32 MCU]           │
           │                    │              │  36V motor gücü              │
       (USB kablo:              │              ├── 2× hub motor (faz)         │
        güç+veri)               │              └── 2× hall sensör             │
           │                    │                      │                      │
           └────────► [ESP32] ◄── UART ────────────────┘ (3.3V, GND/TX/RX)    │
                         │  ▲                                                 │
                         │  ├─ E-stop sense (GPIO25)                          │
                         │  └─ çarpma sensörü (GPIO26, refleks)               │
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
| Batarya [+] | Sigorta | 36V nominal, **42V dolu** | **20A / 60V**, yuvalı (elde var), bataryanın hemen çıkışında. ⚠️ Sigortanın VOLTAJ dayanımı pakette görülecek en yüksek gerilimin üstünde olmalı — otomobil bıçak sigortalarının çoğu 32V'tur ve 42V'ta arkı kesemeyebilir. |
| Sigorta | E-stop → MCU | 36V | Bataryadan **sadece motor gücü** çıkar; başka dal yok |
| **Powerbank** | Pi 4 (USB-C) | 5V / **≥3A** | Elektronik motor bataryasından **ayrı** beslenir (karar 13, 2026-10-08). Buck yok. |
| Pi USB | ESP32 (5V/VIN) | 5V | Güç + veri tek kabloda (prototip kolaylığı) |
| Batarya [-] + Powerbank [-] | **Ortak GND** | — | ⚠️ İki kaynak ayrı ama **eksileri birleşmek zorunda**: ESP32↔MCU UART'ı ve INA228 buna bağlı. Powerbank'in eksisi Pi'ın GND pininden ortak noktaya gelir. |

**Powerbank'in bedelleri (bilerek kabul edildi):**
- Ayrı şarj edilir; bittiğinde Pi **uyarısız** kapanır → SD kart bozulma riski.
  Pi powerbank'in doluluğunu göremez (`/battery` sadece motor paketini izler).
- Bazı powerbank'ler şarj olurken çıkış vermez ya da yük değişiminde anlık
  keser. Seçilen modelle Pi'ı tam yükte (kamera + USB cihazları) dene.
- Kazancı: motorların çektiği akım ve gürültü Pi'ın beslemesine hiç ulaşmaz,
  42V'a dayanıklı pahalı bir buck gerekmez.

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
| ~~Ultrasonik ×4~~ | — | ❌ tasarımdan çıkarıldı (2026-10-08) | Alınmayacak. ESP32'nin refleks katmanı E-stop + çarpma ile sınırlı; yakın menzilde **temastan önce** uyaran hiçbir şey yok. |

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

### INA228 akım sensörü (SP1 — batarya izleme)

Amaç: coulomb sayan SoC için paket akımını ölçmek. Pi I2C bus 1, adres 0x40
(MPU6050 0x68 ile çakışmaz).

**Şönt yerleşimi — MULTİMETREYLE DOĞRULA, TAHMİN YÜRÜTME:**
- Tercih: paketin ortak ana eksi hattı; böylece aynı şönt hem deşarjı hem de
  şarj akımını görür.
- Hoverboard BMS'inde şarj ve deşarj eksileri ayrıysa süreklilik ölçümüyle
  doğrula. Ortak hat yoksa şönt deşarj hattına konur; şarj tespiti voltaj
  sıçramasından yapılır.
- Düşük-taraf montajda INA228 VBUS pini şasi civarında kalır ve kullanılmaz;
  paket voltajı köprünün ham `/battery_raw` yayınıdır. VBUS'u boşta bırakma,
  GND'ye bağla.
- Şönt: eldeki INA228 kartının **üzerindeki** direnç, işareti `R002` = 2 mΩ
  (2026-10-08'de parçadan okundu). Harici şönt alınmıyor. Sürücünün ±163.84 mV
  aralığında bu ±81.9 A tam skala; 20 A'de direnç 0.8 W harcar.
  ⚠️ Kartın kendi yolları ve klemensleri paket akımını taşıyacak — bunların
  akım dayanımı **doğrulanmadı**. İlk yüklü sürüşte kartın ısısına bak.
  `shunt_ohms` config'te 0.002; yine de CALIBRATE edilir.
- `invert_current` ile işareti düzelt: robot bataryadan çalışırken
  `/battery.current` küçük NEGATİF olmalı.
- INA228 I2C tarafı Pi'ın 3.3 V alanındadır. Tüm GND'ler mevcut tek ortak
  noktada birleşir.

---

## 4. Topraklama ve mantık seviyeleri (atlanırsa hiçbir şey çalışmaz)

- **Tek ortak GND:** batarya eksi, powerbank eksi (Pi GND üzerinden), ESP32 GND, MCU sinyal
  GND'si hepsi tek noktada. UART'ın çalışması buna bağlı — ayrı GND'ler
  "veri geliyor ama saçma" olarak görünür, saatlerce aratır.
- **Mantık seviyeleri:** Pi, ESP32, STM32 MCU, IMU, manyetometre → hepsi 3.3V,
  doğrudan bağlanır.
- **İstisnalar (⚠️ seviye uyumu):**
  - NEO-6M TX genelde 3.3V ama modülüne göre değişir — **ölç**.

---

## 5. Hızlı referans — ne nereye

```
STM32 MCU  : 36V güç, 2× motor faz, 2× hall  → (hazır bağlı, dokunma)
             UART 3 tel (GND/TX/RX)          → ESP32 GPIO16/17
ESP32      : USB (güç+veri)                  → Pi
             GPIO25 E-stop, GPIO26 çarpma     → refleks
Pi 4       : powerbank (USB-C)               → güç
             I2C (GPIO2/3): IMU + manyetometre
             USB-TTL: GPS
             CSI: kamera
             USB: ESP32
Güç        : Powerbank → Pi/ESP32   |   Batarya → sigorta → E-stop → MCU
```

---

## 6. Açık uçlar / karar bekleyenler
- [ ] Kontaktör modeli + E-stop bobin sürüş şekli (akıma göre).
- [ ] Manyetometre alımı (QMC5883L, ~100 TL) — 6-eksen IMU'nun eksik parçası.
- [x] ~~Ultrasonik alınacak mı?~~ — **Hayır, tasarımdan çıkarıldı (2026-10-08).**
      Sonucu: yakın menzil katmanı yok; robot survey edilmemiş bir engeli ancak
      tampona değince fark eder (`handoff.md` karar 8).
- [ ] GPS: USB-TTL mü Pi donanım UART'ı mı (USB-TTL daha az dertli).
- [x] ~~Buck~~ — **tasarımdan çıktı (2026-10-08):** Pi powerbank'ten beslenir.
      (Not: buck'a dönülürse girişi ≥60V olmalı; dolu paket 42V ve yaygın
      XL4016/XL4015/LM2596 modülleri buna dayanmaz.)
- [ ] Powerbank seçimi: 5V ≥3A çıkış, Pi tam yükte denenecek.
- [x] ~~INA228 modülü + şönt alımı~~ — kart elde, üzerinde 2 mΩ şönt var.
- [ ] BMS şönt konumunun multimetreyle doğrulanması (yukarıdaki 3. bölüm).
