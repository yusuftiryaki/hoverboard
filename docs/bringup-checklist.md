# Hoverboard Anakart Bring-Up Checklist — TXTY150914NNC-6052MAIN_V2.1

> Bu kart topluluk veritabanlarında **belgesiz**. Aşağıdaki hiçbir pin
> konumu/kablo rengi garanti değil — her ⚠️ adımında **multimetreyle
> doğrula**. MCU/firmware seviyesindeki adımlar (unlock, config, protokol)
> tüm STM32F103R kartlarında aynıdır ve güvenilirdir.

MCU: STM32F103, LQFP64 (F103R ailesi) — firmware hedefi `STM32F103RCT6`.
Firmware: `EFeru/hoverboard-firmware-hack-FOC` (PlatformIO).

---

## 0. Güvenlik ön koşulu (pazarlıksız)

- [ ] Tekerlekler **yerden kesik** — kartı/motoru mengeneye al ya da robotu sehpaya kaldır.
- [ ] Batarya çıkışına **30–40 A sigorta** takılı.
- [ ] Fiziksel **E-stop** (mantar buton, NC) güç hattını fiziksel kesiyor.
- [ ] Yangın söndürücü / kova kum yakında (Li-ion, ilk flash denemesi).

---

## 1. Çip kimliğini kesinleştir (GD32 klon riski)

> Sen "biri STM32 biri GD32" demiştin. Üzerinde "STM32F103" yazsa bile
> çoğu klon GD32 çıkıyor. Karar veren tek şey device ID.

- [ ] ST-Link'i **sadece GND + SWDIO + SWCLK** ile bağla. **3V3 pinini bağlama**
      (programlayıcının 3.3V'u kartı beslerse yakabilir — wiki'nin açık uyarısı).
- [ ] Batarya bağlı, güç butonuna basılı tut (self-latch — bırakırsan MCU kapanır).
- [ ] Çalıştır:
      ```
      st-info --probe
      ```
- [ ] Sonucu not et:
  - `F1xx medium/high-density`, gerçek ST → standart yol, devam.
  - Farklı/GD32 imzası → GD32 portu gerekir, DUR ve bana device ID'yi söyle.

**⚠️ SWD pad sırası sabit değil.** Pad'leri çipe kadar izle:
`SWDIO = PA13`, `SWCLK = PA14`. Renge/sıraya güvenme.

---

## 2. Okuma korumasını (RDP) kaldır

> Fabrika firmware'i kilitli. Unlock, stok firmware'i **geri dönüşsüz** siler.
> Tuğlalama riski pratikte yok — sonrasında istediğin kadar flash'larsın.

STM32CubeProgrammer (GUI, önerilen):
- [ ] Bağlantı modu: **"Connect Under Reset"** (şart — normal modda bağlanamaz).
- [ ] `OB` (Option Bytes) → `RDP` = `AA` (Disable) → Apply.
- [ ] Kartın gücünü kes-ver.

veya OpenOCD:
```
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg \
  -c init -c "reset halt" -c "stm32f1x unlock 0"
```

---

## 3. Firmware'i yapılandır (`Inc/config.h`)

Kullanacağın kabloya göre USART seç (bkz. adım 5):
```c
// Kontrol arayüzü — seri porttan hız/tork komutu
#define CONTROL_SERIAL_USART2    // sol kablo. VEYA _USART3 (sağ kablo, 5V toleranslı)
#define FEEDBACK_SERIAL_USART2   // aynı port — hall'dan hız feedback'i geri gelsin

// Kontrol tipi ve modu — güvenli başlangıç
#define CTRL_TYP_SEL   FOC_CTRL  // FOC (sessiz, verimli)
#define CTRL_MOD_REQ   SPD_MODE  // hız modu (VLT/TRQ yerine — kontrollü test)

// Yön (test sonrası ayarla)
// #define INVERT_L_DIRECTION
// #define INVERT_R_DIRECTION

// Batarya limitleri — 10S için kontrol et
#define BAT_CELLS      10
```
- [ ] Diğer varyantları kapat (`VARIANT_ADC`, `VARIANT_PPM`, `VARIANT_HOVERCAR`…).
- [ ] `pio run` ile derle. Hata yoksa devam.

---

## 4. Flash

- [ ] Batarya bağlı (>36V), güç butonuna basılı tut, tekerlekler havada.
- [ ] `pio run -t upload` (veya CubeProgrammer ile `.bin` yaz).
- [ ] Flash *sırasında* MCU halt → motor dönmez. Risk flash *sonrası* firmware
      çalışınca başlar; onu havada tekerlekle karşılıyorsun.

---

## 5. ⚠️ UART hattını multimetreyle doğrula (ESP32 bağlamadan ÖNCE)

> Firmware kuralı: **sol kablo = USART2**, **sağ kablo = USART3 (5V toleranslı)**.
> Ama fiziksel pin sırası ve GND/15V yeri bu kartta belgesiz.
> "Bazı kartlarda siyah kablo GND değil 15V!" — yakma riski burada.

Yan kart (BLB) sökülü. Yan karta giden ince kablo demeti üzerinde:
- [ ] Süreklilik modu: her teli kartın **GND'sine** (batarya eksi / büyük bakır alan)
      değdir → **GND telini** bul.
- [ ] Batarya bağlıyken (dikkatli) her tele karşı GND voltajı ölç:
      **~15V olan = besleme.** Bunu **ESP32'ye ASLA bağlama.**
- [ ] Kalan iki tel = TX / RX. ESP32 tarafında **çapraz** bağla (STM32 TX → ESP32 RX).
- [ ] Sonuç tablosu:

  | Tel rengi | Ölçüm | Rol | ESP32'ye |
  |-----------|-------|-----|----------|
  |           | 0V/süreklilik | GND | GND |
  |           | ~15V  | 15V | **BAĞLAMA** |
  |           | ~3.3V | TX  | RX2 (GPIO16) |
  |           | ~3.3V | RX  | TX2 (GPIO17) |

- [ ] STM32 ve ESP32 ikisi de 3.3V mantık — seviye çevirici gerekmez.

---

## 6. Tekerlek testi (başarı kriteri)

- [ ] ESP32'yi (veya USB-TTL) UART'a bağla, 115200 baud.
- [ ] Protokol çerçevesi gönder (start `0xABCD`, int16 steer, int16 speed, checksum).
- [ ] **Tekerlek dönüyor.** ✅ Projenin en riskli kısmı bitti.
- [ ] Feedback paketinde ölçülen hızı okuyabiliyorsun (odometrinin temeli).
- [ ] Yön ters/stutter varsa: `INVERT_*_DIRECTION` veya hall kombinasyonunu değiştir
      (yanlış hall = dönmez/titrer, **hasar vermez**).

---

## Hoverboard seri protokolü (referans)

**Komut (ESP32 → STM32), 115200 baud:**
```
uint16 start   = 0xABCD
int16  steer
int16  speed
uint16 checksum = start ^ steer ^ speed
```
**Feedback (STM32 → ESP32):**
```
uint16 start = 0xABCD
int16  cmd1, cmd2
int16  speedR_meas, speedL_meas   // hall'dan — odometri
int16  batVoltage                 // ~santi-volt
int16  boardTemp
uint16 cmdLed
uint16 checksum
```

## Bilinen tuzaklar
- **Self-latch:** açılışta butona bas, kapatmak için uzun bas. Flash boyunca basılı tut.
- **Buzzer/LED pinleri** bu varyantta belgesiz — motorlar çalışsa bile
  buzzer sessiz / LED farklı olabilir. Kritik değil.
- **GD32 ise:** farklı baud (bazı raporlarda 52177) ve farklı unlock. Adım 1'de yakala.
