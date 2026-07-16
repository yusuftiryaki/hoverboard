# Proje Devir Teslim Notu (session handoff)

> Bu dosya, projenin o ana kadarki tüm kararlarını ve durumunu taşır.
> Yeni bir Claude session'ı bunu okuyarak kaldığı yerden devam edebilir.
> **Kullanıcı Türkçe konuşuyor — Türkçe yanıt ver.**

## Hedef
Hoverboard parçalarından **dış mekan otonom yer robotu**. Açık alanda
(bahçe/park/tarla) GPS waypoint takibi. Prototip kalitesi — su geçirmezlik vb. yok.

## Kullanıcı profili
Backend yazılım geliştirme uzmanı, **ROS 2 deneyimi var**. Yazılım tarafı risksiz;
riskler fizik/elektrik/RF tarafında. Ek bütçe ~2-3 bin TL.

## Donanım envanteri
- Hoverboard ×2 (birinin bataryası ölü) → 4 hub motor, 2 anakart, 36V 10S batarya
- Raspberry Pi 4, ESP32
- **6 eksenli** IMU (MPU6050 sınıfı — magnetometer YOK)
- GPS **NEO-6M** (~2.5-5 m doğruluk)
- Pi Camera V2, buck converter, çarpma sensörü, butonlar, kablolar

## Anakartlar (kritik)
- **Kart 1 (kullanılacak):** `TXTY150914NNC-6052MAIN_V2.1`, **STM32F103 LQFP64**.
  Yan kart: `TXTY150911NNC-6052BLB_V8.1`.
- **Kart 2 (ertelendi):** `TSX1-6052-JYV2.0` yan kart etiketi, **GD32**.
  Alt kartında `FLASH` ipek baskılı pad grubu + `A B` padleri var.
- ⚠️ **Araştırma sonucu: TXTY 6052 varyantı hiçbir yerde belgelenmemiş.**
  (EFeru, RoboDurden varyant DB'si, forumlar tarandı — kayıt yok.)
  MCU/firmware seviyesi (unlock, USART rolleri, protokol) tüm STM32F103R
  kartlarında aynı → güvenilir. **Fiziksel pinout'lar (SWD pad sırası,
  hangi konnektör USART2/3, GND/15V/TX/RX sırası, hall sırası) MUTLAKA
  multimetreyle doğrulanmalı** — tahmin yürütülmemeli.

## Alınmış kararlar (gerekçeleriyle)
1. **Form: 4 noktalı statik platform** (2 tahrikli + caster), kendi kendine
   dengelenen DEĞİL. Kontrol basit kalsın, otonomiye odaklanılsın.
2. **2WD önce, STM32 kartıyla.** GD32 kartı ayrı/olgunlaşmamış firmware yolu
   gerektirdiği için **arka aks yükseltmesi olarak ertelendi**. Riski ikiye
   katlamamak için. ESP32 katmanı ikinci kartı sonradan eklemeye açık.
3. **LIDAR yok** — ucuz 2D LIDAR güneşte kötü + bütçe. Katmanlı algı:
   GPS+IMU (uzak) / kamera (orta, zemin segmentasyonu) / ultrasonik (yakın) /
   çarpma (temas).
4. ⚠️ **Magnetometer şart (QMC5883L ~100 TL).** 6-eksen IMU + NEO-6M duruyorken
   **yön (heading) veremiyor** (GPS course-over-ground sadece >1 m/s'de anlamlı).
   Direğe, gövdeden **>30 cm** yukarı, motor kablolarından uzağa monte + hard/soft
   iron kalibrasyon. Yedek: GPS ile "yön başlatma manevrası" (5 m düz git, gyro'yu
   çapala).
5. **5 m GPS hatası** → sadece **açık alan, geniş koridor** görevleri. "Kaldırımda
   markete git" bu donanımla mümkün değil (RTK gerekir).
6. **İki hız katmanı:** Pi = yavaş akıllı sensörler (IMU, mag, GPS, kamera →
   lokalizasyon/algı). ESP32 = hızlı refleks (E-stop, çarpma, ultrasonik).
   **Pi asla motorlarla doğrudan konuşmaz.**
7. **IMU → Pi I2C** (ESP32'ye değil; denge robotu yapmıyoruz, lokalizasyon Pi'da).
8. **Ultrasonik → ESP32** (refleks katmanı) — firmware eklentisi henüz YAZILMADI.
9. **GPS → USB-TTL**, tercihen **FTDI** (ESP32'nin çipiyle VID:PID çakışmasın).

## Güç / E-stop tasarımı
```
Batarya 36V → [30-40A sigorta] → ┬→ [buck 5V/5A] → Pi (+ USB ile ESP32)  [HER ZAMAN AÇIK]
                                 └→ [E-stop kontaktör] → anakart 36V (MCU + motorlar)
```
- **MCU'ya ayrı besleme YOK** — anakartın kendi regülatörü 36V'tan üretiyor.
- E-stop → MCU de söner → motorlar **coast** ile durur (aktif fren yok) →
  geri dönüşte **hoverboard güç butonuna basmak gerekir** (self-latch).
- MOSFET köprüleri motor gücüyle aynı 36V barasında → "sadece motoru kes,
  MCU'yu ayakta tut" fiziksel olarak mümkün değil. Bilinçli seçim.
- Tüm GND'ler **tek ortak nokta**da birleşir (UART'ın çalışması buna bağlı).

## Yol boyunca yapılan ÖNEMLİ düzeltmeler (tekrarlanmasın)
1. ⚠️ **Flash sırasında batarya BAĞLI olmalı.** (Önce "bağlama" denmişti, YANLIŞ.)
   Kart self-latch ile besleniyor, MCU gücünü 36V'tan alıyor. **ST-Link'in
   3.3V'undan kartı besleme — anakart öldürür.** Motor riski bataryayı sökerek
   değil, **tekerlekleri havada tutarak** yönetilir.
2. ⚠️ **E-stop polaritesi fail-safe yapıldı.** NC kontak → GND, INPUT_PULLUP.
   Kapalı(çalışıyor)=LOW, **açık (basılı VEYA kablo kopuk)=HIGH=DUR**.
   Önceki hali kablo kopunca "güvenli" sanıyordu.

## Repo durumu
```
docs/bringup-checklist.md   STM32 kartı flash prosedürü (multimetre doğrulama noktalarıyla)
docs/wiring-map.md          Kaba bağlantı haritası (güç + Pi + ESP32 + MCU + sensörler)
docs/devcontainer.md        Dev ortamı + flashing iş akışı (WSL2 usbipd vs Pi'dan flash)
docs/deployment.md          Pi deployment planı (8 adım)
docs/handoff.md             bu dosya
firmware/esp32_bridge/      platformio.ini + src/main.cpp  (pio run ile derlenir)
ros2/src/hoverboard_bridge/ ESP32 seri köprü düğümü (Python) + protokol + sahte ESP32
ros2/src/robot_bringup/     launch + ekf.yaml + urdf
.devcontainer/              Dockerfile + devcontainer.json
scripts/deploy.sh           push → Pi'da pull + colcon build
```
ESP32 köprüsü (`src/main.cpp`) hazır: watchdog (Pi 200ms susarsa dur), fail-safe
E-stop latch, hoverboard 0xABCD protokolü, Pi'a feedback (ölçülen hız/voltaj/temp).
Ultrasonik + çarpma sensörü refleksleri **henüz eklenmedi** (planlı).

### ROS 2 workspace (kuruldu, donanımsız doğrulandı)
**Mimari kararı:** kinematiği+odometriyi **kendi Python düğümümüz** yapıyor,
ros2_control/diff_drive_controller **kullanılmadı**. Gerekçe: en hızlı yoldan
sürülebilir robot (yol haritası adım 4) + protokol hata ayıklaması tek dosyada.
`protocol.py` bilinçli olarak rclpy'den bağımsız — 4WD/2. kart gündeme gelirse
ros2_control `SystemInterface`'i onun üstüne sarılır, düğüm teleop aracı olarak kalır.

| Dosya | İş |
|---|---|
| `hoverboard_bridge/protocol.py` | `PiCommand`/`EspFeedback` pack/unpack + checksum + resync eden çerçeve ayrıştırıcı. `main.cpp` ile **byte-byte aynı olduğu doğrulandı** (struct'lar firmware'den söküldü, gcc referans byte'larıyla karşılaştırıldı). |
| `hoverboard_bridge/bridge_node.py` | `/cmd_vel` → ters kinematik → seri; `EspFeedback` → `/odom`, `/joint_states`, `/battery`, `/diagnostics`. `~/clear_estop` servisi (std_srvs/Trigger). |
| `hoverboard_bridge/fake_esp32.py` | pty üzerinde **sahte ESP32** — donanımsız tam döngü testi. `ros2 run hoverboard_bridge fake_esp32` |
| `robot_bringup/config/ekf.yaml` | çift-EKF + navsat_transform, gerekçeleri yorumda |
| `robot_bringup/config/hoverboard_bridge.yaml` | düğüm parametreleri, **CALIBRATE** işaretleriyle |
| `robot_bringup/urdf/robot.urdf.xacro` | 2 tahrik + 2 caster + sensör frame'leri (direkte mag/GPS) |
| `robot_bringup/launch/` | `robot` (üst), `teleop`, `localization`, `sensors`, `description` |

**Doğrulanan (sahte ESP32'ye karşı, gerçek donanım YOK):**
protokol byte uyumu · `/cmd_vel` 0.4 m/s → ölçülen 0.389 · E-stop latch'liyken
tekerlek dönmüyor · `clear_estop` sonrası dönüyor · `/cmd_vel` kesilince
`cmd_timeout` sıfırlıyor · SIGINT/SIGTERM'de temiz çıkış (systemd için) ·
`ekf_local` `/odom`'u yiyip `odom→base_link` yayınlıyor · URDF parse + tüm launch'lar yükleniyor.

**Launch varsayılanları kasten "en az donanım":** sadece bridge + URDF.
Sensörler ve lokalizasyon opt-in (`use_localization:=true use_gps:=true use_camera:=true`).

⚠️ **Tüm bunlar simülasyon.** Gerçek kartta hiçbiri denenmedi; `cmd_per_rpm`,
`steer_sign`, `invert_left/right`, `wheel_separation`, `battery_scale`
**kalibre edilmemiş tahminler** (TXTY kartı belgesiz — tahmin yürütülemez).

## Deployment kararları
- Pi **sıfır** → **Ubuntu Server 24.04 64-bit** yüklenecek.
- ROS 2 **Jazzy native** (apt), container değil.
- Dev döngüsü: **Pi'da git pull + colcon build** (`scripts/deploy.sh`).
- Dev makine ↔ Pi **aynı DDS ağı** (`ROS_DOMAIN_ID=42`).
  ⚠️ **WSL2 tuzağı:** WSL2 NAT arkasında; `--network=host` YETMEZ.
  Windows `.wslconfig` → `networkingMode=mirrored` + `wsl --shutdown` şart.
- ⚠️ udev ile sabit isim: `/dev/esp32`, `/dev/gps` (ikisi de USB-serial,
  numaraları kayar). Aynı çip kullanırlarsa VID:PID çakışır → farklı çip al.

## Yol haritası (sıra önemli, atlanmaz)
1. **Anakartı flash'la** (STM32, EFeru FOC firmware) — en riskli adım
2. **ESP32 aracı katmanı** — tezgahta test
3. **Şasi** — en sıkıcı, en uzun
4. **ROS 2 teleop** — `/cmd_vel` → tekerlek (burada sürülebilir robot olur)
5. **Odometri + lokalizasyon** — hall + IMU + GPS → `robot_localization` çift-EKF
6. **Nav2 waypoint takibi** — açık alanda
7. **Engelden kaçınma** — ultrasonik + kamera → Nav2 costmap

## ŞU AN NEREDEYIZ / SIRADAKİ İŞ
ROS 2 workspace iskeleti **kuruldu ve sahte ESP32'ye karşı doğrulandı** (yukarı bak).
Yazılım tarafı artık donanımı bekliyor. Sıradaki iş **ST-Link gelince yol
haritası adım 1** (kartı flash'la) — o olmadan adım 2/4 açılmıyor.

Donanım beklerken yazılımda yapılabilecekler (bağımsız):
1. **⚠️ IMU sürücüsü — en büyük boşluk.** Jazzy apt'de MPU6050 sürücüsü YOK.
   `sensors.launch.py` içinde TODO olarak duruyor. Kendi rclpy düğümümüzü yazmak
   (smbus2, ~150 satır) en temizi. `/imu/data`'yı `imu_link` frame'inde, REP-103
   ile (x ileri, y sol, z yukarı) yayınlamalı ve **mutlak yaw yayınlamamalı**
   (6-eksen; karar 4). Bu olmadan `ekf_local` sadece tekerlek odometrisiyle koşar
   → yaw hızla kayar. **Lokalizasyonun bir sonraki gerçek adımı bu.**
2. Ultrasonik + çarpma refleksleri → ESP32 firmware + protokol genişletmesi
   (`EspFeedback`'e alan eklenirse `protocol.py` ile birlikte güncellenmeli —
   ikisi byte-byte bağlı, testler bunu yakalar).
3. Nav2 config (adım 6) — `robot_bringup/config/nav2.yaml` henüz yok.

## Bilinen blokerler / bekleyen alımlar
- **ST-Link V2 klon (~150 TL)** — adım 1 için şart, henüz alınmadı
- **Magnetometer QMC5883L (~100 TL)** — listedeki en yüksek getirili harcama
- Multimetre doğrulaması (UART pinout) — kart elde olunca
- Caster ×2 (125-150 mm kauçuk), mantar E-stop + kontaktör, sigorta
- Dockerfile'daki `ros-jazzy-nmea-navsat-driver` paketi Jazzy apt'de olmayabilir
  → imaj build'i orada patlarsa listeden çıkar, GPS sürücüsünü başka yolla çöz
  (`sensors.launch.py` bu düğümü `use_gps:=true` ile çağırıyor)
- ⚠️ **Repo henüz `git init` edilmedi**, ama `scripts/deploy.sh` `git push`'a ve
  `deployment.md` adım 3 `git clone`'a dayanıyor. Pi'a deploy etmeden önce
  git init + remote şart. `.gitignore` yazıldı (ros2/build,install,log + .pio).
- `camera_ros` paketi Dockerfile'da kurulu değil (`sensors.launch.py` `use_camera:=true`
  ile onu çağırıyor) — kamera adımına (7) gelince eklenecek.
