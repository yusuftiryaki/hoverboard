# Alım listesi

*(son güncelleme: 2026-10-08 — kullanıcıyla kalem kalem gözden geçirildi)*

Elektronik kalemler **tek tedarikçiden** alınacak. Seçilen: **Motorobit**.
Gerekçesi aşağıda. Fiyat ve stok **doğrulanmadı** — Motorobit sayfaları bunları
tarayıcıda yüklüyor, dışarıdan okunamadı. Sepete eklerken kontrol et.

## Bu siparişte alınacaklar

| # | Kalem | Aciliyet | Neyi açıyor | Dikkat |
|---|---|---|---|---|
| 1 | **ST-Link V2 klon** | 1 | B1: flash — tüm donanım izinin kilidi | Klonlarda kasadaki pin yazısı yanlış olabiliyor; bağlamadan önce doğrula. Sadece GND + SWDIO + SWCLK bağlanır, 3V3 bağlanmaz. |
| 2 | **QMC5883L (GY-271)** | 3 | Mutlak yön (B6) | ⚠️ Aynı kart **HMC5883L** ile de satılıyor. Sürücümüz 0x0D'deki QMC'yi bekler; HMC (0x1E) gelirse çalışmaz. Gelince `i2cdetect -y 1` ile bak. |
| 3 | **FT232RL USB-TTL modülü** | 3 | GPS (B6) | **Modül** olmalı (USB girişli kart), çıplak entegre değil. ESP32'nin USB çipinden farklı olmalı, yoksa udev ikisini ayıramaz (`deployment.md` adım 4). 3.3V seçilebilir olsun. |

Aciliyet 1 = yokluğu şu an her şeyi durduruyor. Sipariş tek seferde verildiği
için hepsi aynı sepete girer.

## Karar bekleyen

| Kalem | Durum |
|---|---|
| **E-stop** | Kullanıcının elinde "bir iki büyük anahtar" var. Tezgah aşamasında (tekerlekler havada) güç hattını fiziksel kesme işini bir anahtar görebilir — **DC dayanımı yeterliyse**. Anahtarın üzerindeki yazı bekleniyor. Mantar buton (basınca kilitlenen, avuçla vurulan) hareketli robotta gerekir; kontaktörle birlikte alınabilir. |

## Ertelenenler

| Kalem | Neden |
|---|---|
| **DC kontaktör** | Yük altında sürüşte (B5) gerekir, tezgahta değil. Bobin beslemesi sorusu açık (12V bobin ayrı bir hat ister; paket 30–42V arasında oynuyor). İkinci sipariş olacak. |

## Elde olanlar (alınmayacak)

| Kalem | Not |
|---|---|
| INA228 | Üzerinde `R002` = 2 mΩ şönt var → **harici şönt alınmıyor**. ±81.9 A tam skala, 20 A'de 0.8 W. Kartın yol/klemens akım dayanımı doğrulanmadı. |
| Sigorta + yuva | 20 A / 60 V. Plan 30–40 A diyordu; firmware akım limitleri buna göre ayarlanacak (`bringup-checklist.md` adım 3). |
| Pi 4, ESP32, NEO-6M, MPU6050, Pi Camera V2, hoverboard ×2, batarya, şarj aleti | Envanter (`handoff.md`). |

## Tasarımdan çıkanlar

| Kalem | Karar |
|---|---|
| Ultrasonik ×4 + bölücü dirençler | 2026-10-08'de çıkarıldı (`handoff.md` karar 8). Bedeli: yakın menzil katmanı yok. |
| Harici şönt | INA228'in kendi şöntü yetiyor. |
| Buck dönüştürücü | 2026-10-08'de çıkarıldı (`handoff.md` karar 13): Pi **powerbank**'ten beslenir, elektronik ve mekanik ayrı. 42V'a dayanan ≥5A modül pahalıydı. |

## Elektronikçide olmayanlar (hırdavatçı)

- Caster ×2, 125–150 mm, kauçuk — şasi (B3) kararına bağlı
- Mast: ≥30 cm plastik/PVC/karbon boru + pirinç ya da naylon vida (demir olmaz)
- Kalın kablo, pabuç, makaron, klemens, ortak GND barası

## Neden Motorobit

Üç mağaza kalem kalem tarandı (2026-10-08):

| Kalem | Motorobit | Direnc.net | Robotistan |
|---|---|---|---|
| Manyetometre | Başlıkta **QMC5883L** | Sadece HMC5883L | "HMC5883L / QMC5883" karışık |
| ST-Link V2 | Var | Var (116 TL, KDV dahil) | Stokta yok |
| ≥60V buck | 8–60V 15A modül | Bulunamadı | Bulunamadı |
| DC kontaktör | EVC500 (12V bobin) | Bulunamadı | Bulunamadı |
| FT232RL | Var | Var | Var |

Belirleyici olan manyetometre: sürücümüz QMC5883L bekliyor ve onu açıkça o
adla satan tek mağaza bu. (Buck artık listede değil, tablodaki satırı tarihçe.)

- [GY-271 QMC5883L](https://www.motorobit.com/gy-271-qmc5883l-3-axis-compass-sensor)
- [DC-DC 8-60V 15A](https://www.motorobit.com/dc-dc-8-60v-15a-ayarlanabilir-voltaj-dusurucu-modul)
- [ST-Link V2 CN](https://www.motorobit.com/st-link-v2-cn)
- [FT232 USB-UART](https://www.motorobit.com/ft232-usb-uart-converter)
- [Mantar E-stop 1NO+1NC](https://www.motorobit.com/mushroom-head-emergency-stop-button-1no1nc-spring-auto-reset-en) (30 VDC / 10 A — paket hattını doğrudan kesmek için yetmez, sadece kontaktör bobini için)
- [EVC500 kontaktör](https://www.motorobit.com/evc500-12-24v-high-voltage-relay-contactor-pn-2098190-1)
