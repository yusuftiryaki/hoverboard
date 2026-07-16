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
ros2/src/mpu6050_driver/    IMU sürücüsü (Jazzy'de yok, kendimiz yazdık) + sahte I2C
ros2/src/robot_bringup/     launch + ekf.yaml + urdf
.devcontainer/              Dockerfile + devcontainer.json
scripts/deploy.sh           push → Pi'da pull + colcon build
```
ESP32 köprüsü (`src/main.cpp`) hazır: watchdog (Pi 200ms susarsa dur), fail-safe
E-stop latch, **çarpma refleksi**, hoverboard 0xABCD protokolü, Pi'a feedback
(ölçülen hız/voltaj/temp/güvenlik durumu).

### Çarpma refleksi (GPIO26) — yönlü veto
**Latch YOK.** Çarpma varken **ileri veto** edilir; **geri ve dönüş serbest**
kalır, kontak bırakılınca (100 ms kararlı) kendi kendine açılır. Gerekçe: latch'li
bir çarpma sensörü robotu dokunduğu şeye yaslanmış halde kilitler ve kurtulmak
için Pi'la el sıkışma gerektirir; veto ise Nav2'nin hiçbir şey yapmadan geri
çekilmesine izin verir ve çarpmayı asla kötüleştiremez.
- Kablolama **NC** (E-stop'la aynı fail-safe): kopuk kablo = ileri reddedilir.
- Debounce **asimetrik**: tehlikeye anında geçer (debounce yok), güvenliye
  100 ms kararlı-temiz sonrası döner. Kontak zıplaması vetoyu çarpmanın
  ortasında söndüremesin diye.
- ROS: `/bumper` (std_msgs/Bool) + `/diagnostics` (WARN, ERROR değil — robot
  bozuk değil, bir şeye dokunuyor ve kendi gücüyle çıkabilir).
- ⚠️ `BUMP_BLOCKS_POSITIVE_SPEED` sabiti tezgahta doğrulanmalı — ters çıkarsa
  veto robotu çarptığı şeye bindirir. Detay: `docs/wiring-map.md` 3c.

**Ultrasonik refleksi YAZILMADI** — sensörler envanterde yok, alım kararı da
verilmedi (`wiring-map.md` 6. bölüm).

### ROS 2 workspace (kuruldu, donanımsız doğrulandı)
**Mimari kararı:** kinematiği+odometriyi **kendi Python düğümümüz** yapıyor,
ros2_control/diff_drive_controller **kullanılmadı**. Gerekçe: en hızlı yoldan
sürülebilir robot (yol haritası adım 4) + protokol hata ayıklaması tek dosyada.
`protocol.py` bilinçli olarak rclpy'den bağımsız — 4WD/2. kart gündeme gelirse
ros2_control `SystemInterface`'i onun üstüne sarılır, düğüm teleop aracı olarak kalır.

| Dosya | İş |
|---|---|
| `hoverboard_bridge/protocol.py` | `PiCommand` (10 B) / `EspFeedback` (16 B) pack/unpack + checksum + resync eden çerçeve ayrıştırıcı. `main.cpp` ile **byte-byte aynı olduğu doğrulandı** (struct'lar firmware'den söküldü, gcc referans byte'larıyla karşılaştırıldı). ⚠️ uint8 bayraklar checksum'da **ikişerli** 16-bit kelimeye katlanıyor → yeni bayrak eklerken yanına pad gerekir. |
| `hoverboard_bridge/bridge_node.py` | `/cmd_vel` → ters kinematik → seri; `EspFeedback` → `/odom`, `/joint_states`, `/battery`, `/diagnostics`. `~/clear_estop` servisi (std_srvs/Trigger). |
| `hoverboard_bridge/fake_esp32.py` | pty üzerinde **sahte ESP32** — donanımsız tam döngü testi. `ros2 run hoverboard_bridge fake_esp32` |
| `mpu6050_driver/mpu6050.py` | register seviyesi MPU6050 (ROS'suz, I2C bus enjekte edilir) |
| `mpu6050_driver/imu_node.py` | `/imu/data` + `/imu/temperature` + `/diagnostics`; açılışta gyro bias kalibrasyonu |
| `mpu6050_driver/fake_bus.py` | **sahte I2C chip** — register seviyesinde, config register'larını geri çözüp ölçekliyor |
| `robot_bringup/config/ekf.yaml` | çift-EKF + navsat_transform, gerekçeleri yorumda |
| `robot_bringup/config/hoverboard_bridge.yaml` | düğüm parametreleri, **CALIBRATE** işaretleriyle |
| `robot_bringup/urdf/robot.urdf.xacro` | 2 tahrik + 2 caster + sensör frame'leri (direkte mag/GPS) |
| `robot_bringup/launch/` | `robot` (üst), `teleop`, `localization`, `sensors`, `description` |

**Doğrulanan (sahte ESP32 + sahte I2C'ye karşı, gerçek donanım YOK):**
protokol byte uyumu · `/cmd_vel` 0.4 m/s → ölçülen 0.389 · E-stop latch'liyken
tekerlek dönmüyor · `clear_estop` sonrası dönüyor · `/cmd_vel` kesilince
`cmd_timeout` sıfırlıyor · SIGINT/SIGTERM'de temiz çıkış (systemd için) ·
IMU 100 Hz, gyro bias kalibrasyonu sahte chip'in bias'ını tam buluyor ·
URDF parse + tüm launch'lar yükleniyor ·
**çarpma vetosu yönlü olduğu kanıtlandı** (çarpma varken ileri 0.000, geri -0.389;
bırakınca el sıkışmasız ileri döner) · **E-stop regresyonu:** E-stop hâlâ *her iki*
yönü kesiyor (veto tek kapı hâline gelmemiş) ·
**EKF füzyonu karşıt testle kanıtlandı:**
IMU açıkken EKF yaw = gyro (0.0004), kapalıyken = tekerlek (0.4838); vx her iki
durumda tekerlekten geliyor.

> ⚠️ **EKF yaw'da gyro'yu tekerleğe göre ~500× ağırlıklandırıyor** (gyro varyansı
> 1e-4 vs tekerlek vyaw 0.05). Bu **kasıtlı ve doğru**: patinajda tekerlekten
> türetilen yaw çöptür. Ama `gyro_variance_floor` sahada titreşimle birlikte
> fazla iyimser kalabilir — EKF çıktısı zıplarsa önce onu yükselt.

**Launch varsayılanları kasten "en az donanım":** sadece bridge + URDF.
Sensörler ve lokalizasyon opt-in.
Donanımsız tam yığın:
`ros2 run hoverboard_bridge fake_esp32` + `ros2 launch robot_bringup robot.launch.py
esp32_port:=/tmp/fake_esp32 use_localization:=true use_imu:=true fake_imu:=true`

⚠️ **Tüm bunlar simülasyon.** Gerçek kartta/chip'te hiçbiri denenmedi;
`cmd_per_rpm`, `steer_sign`, `invert_left/right`, `wheel_separation`,
`battery_scale` **kalibre edilmemiş tahminler** (TXTY kartı belgesiz — tahmin
yürütülemez). Sahte I2C chip gerçek MPU6050'nin **sıcaklık kayması, clipping,
eksenler arası duyarlılık, I2C glitch'leri ve motor titreşimini** modellemiyor —
yani matematik doğru, sensörün iyi olduğu kanıtlanmadı.

**IMU eksen yönü kararı:** sürücü ham veriyi chip'in kendi ekseninde `imu_link`
olarak yayınlıyor; montaj yönü **URDF'teki `imu_joint` rpy'ında** tarif edilecek
(robot_localization tf ile base_link'e döndürüyor). Eksenleri sürücüde de
"düzeltme" — dönüşüm iki kez uygulanır.

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
1. ✅ ~~IMU sürücüsü~~ — **yazıldı** (`mpu6050_driver`), sahte I2C ile doğrulandı.
   `use_imu:=true` ile açılır. Kalan: gerçek chip gelince `i2cdetect -y 1` ile
   0x68'i gör, `imu_joint` rpy'ını gerçek montaj yönüne göre ölç/yaz.
2. ✅ ~~Çarpma refleksi~~ — **yazıldı**, yönlü veto, sahte ESP32'yle doğrulandı.
   Ultrasonik kısmı **bilinçli olarak yapılmadı**: sensör envanterde yok, kaç
   adet/hangi açı kararı verilmedi — montaj geometrisi bilinmeden "ön engel"
   mantığı tahmin olurdu.
3. Nav2 config (adım 6) — `robot_bringup/config/nav2.yaml` henüz yok.
   `/bumper` artık var; Nav2'ye costmap katmanı olarak bağlanabilir.
4. Manyetometre sürücüsü (QMC5883L) — chip alınınca. `mpu6050_driver` deseni
   (register katmanı + sahte bus + ROS düğümü) aynen tekrarlanabilir.
   Geldiğinde `ekf.yaml`'da **`ekf_global`'ın** imu0 yaw bayrağı açılacak
   (`ekf_local`'ın DEĞİL — kötü heading base_link'i savurmasın).

## Bilinen blokerler / bekleyen alımlar
- **ST-Link V2 klon (~150 TL)** — adım 1 için şart, henüz alınmadı
- **Magnetometer QMC5883L (~100 TL)** — listedeki en yüksek getirili harcama
- Multimetre doğrulaması (UART pinout) — kart elde olunca
- Caster ×2 (125-150 mm kauçuk), mantar E-stop + kontaktör, sigorta
- Dockerfile'daki `ros-jazzy-nmea-navsat-driver` paketi Jazzy apt'de olmayabilir
  → imaj build'i orada patlarsa listeden çıkar, GPS sürücüsünü başka yolla çöz
  (`sensors.launch.py` bu düğümü `use_gps:=true` ile çağırıyor)
- `camera_ros` paketi Dockerfile'da kurulu değil (`sensors.launch.py` `use_camera:=true`
  ile onu çağırıyor) — kamera adımına (7) gelince eklenecek.
- ⚠️ **Devcontainer rebuild bekliyor.** Dockerfile'a iki şey eklendi ama imaj
  yeniden derlenip doğrulanmadı (mevcut konteyner eski imajdan koşuyor):
  1. `python3-smbus2` (IMU sürücüsü için). Paketin noble/**universe**'te olduğu
     Launchpad'den teyit edildi (0.4.3-1), konteynerde universe açık → gelmeli.
     Pi tarafında `rosdep install` zaten çeker.
  2. **Parolasız sudo** (`/etc/sudoers.d/ubuntu`). Sebep: taban imajın `ubuntu`
     kullanıcısı `sudo` grubunda ama parolası kilitli → sudo kimsenin bilmediği
     bir parola soruyordu. `visudo -c` build guard'ı var. Detay: `docs/devcontainer.md`.

  Rebuild sonrası kontrol: `sudo -n true && python3 -c "import smbus2"`
