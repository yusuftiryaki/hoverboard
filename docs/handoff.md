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

## A2'de bulunan tuzaklar (hepsi sessizce yanlış davranıyordu, hiçbiri hata vermedi)
1. **`navsat_transform`'un IMU topic'i `imu`, `imu/data` DEĞİL.** `("imu/data",
   "imu/data")` remap'i no-op'tu; düğüm kimsenin yayınlamadığı `/imu`'yu dinledi,
   dönüşümü hiç hesaplamadı, **`/fromLL` dünyadaki her koordinat için (0,0)
   döndürdü**. Hiçbir şey hata vermedi — GPS waypoint'leri sadece başka yere sürdü.
   Kontrol: `ros2 node info /navsat_transform`.
2. **`magnetic_declination_radians` manyetometre yokken 0 olmalı.** Sapma
   *manyetometre* okumasını düzeltir; bizde manyetometre yok, o yüzden 0.112
   koymak map frame'ini 6.4° döndürdü. `/fromLL` ile ölçüldü.
3. **`xy_goal_tolerance` > lookahead mesafesi olamaz.** RPP "hedefe vardım mı"
   sorusunu **takip noktasına** olan mesafeyle kıyaslıyor. Tolerans (1.5) >
   lookahead (0.4) olunca RPP kalıcı olarak "vardım, hedef yönüne döneyim"
   moduna girdi: **linear hız sıfır**, robot yerinde titredi. Belgelenmemiş bağlantı.
4. **`plugins: []` ROS 2'de ifade edilemez** — boş YAML listesi tipini kaybeder,
   `rclcpp` "No parameter value set" ile abort eder. Costmap katmanı istemesek de
   listeye bir şey koymak gerekiyor (`inflation_layer`, şişirecek engel yok).
5. **`default_server_timeout: 20` (ms) iş istasyonu varsayıyor.** Planner ACK'i
   yetiştiremeyince BT, robotun fiziksel olarak vardığı waypoint'i "başarısız"
   saydı. 1000 yapıldı — **Pi 4 bu kutudan yavaş, hızlı değil.**
6. **`stop_on_failure: false` + `missed_waypoints`**: action `SUCCEEDED` döner
   ama waypoint'ler kaçırılmış olabilir. Sadece status'e bakma, `missed_waypoints`'i say.
7. **Kendi sim'imizde frame çakışması:** `sim_node` ground truth'u `map` diye
   yayınlıyordu, ama `navsat`'ın `map`'i datum'un GPS hatasıyla çapalı — iki farklı
   origin aynı ismi taşıyordu. Artık **`sim_world`**.

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
ros2/src/hoverboard_bridge/ ESP32 seri köprü düğümü (Python) + protokol + ESP32 beyni
ros2/src/mpu6050_driver/    IMU sürücüsü (Jazzy'de yok, kendimiz yazdık) + sahte I2C
ros2/src/robot_sim/         kinematik dünya + ground truth + sahte IMU/GPS
ros2/src/qmc5883l_driver/   manyetometre sürücüsü + sahte I2C (mutlak yönün tek kaynağı)
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
| `hoverboard_bridge/esp32_sim.py` | **ESP32'nin beyni**, tekerleksiz: protokol + watchdog + E-stop + çarpma vetosu + mixer. Arka uç takılabilir (`Backend` protokolü). ROS'suz. |
| `hoverboard_bridge/fake_esp32.py` | ince CLI: `esp32_sim` + `LagBackend`. Dünyası/pozu yok — "çerçeveler ve güvenlik doğru mu" sorusunu cevaplar. `ros2 run hoverboard_bridge fake_esp32` |
| `robot_sim/world.py` | ROS'suz kinematik dünya: tekerlek komutu → ölçülen rpm + **ground truth poz**. Arc entegrasyonu (Euler değil). Fizik YOK. |
| `robot_sim/sim_node.py` | dünyayı koşturur; `/ground_truth`, sahte `/imu/data`, sahte `/gps/fix` yayınlar. `ros2 run robot_sim sim_node` |
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

## Yol haritası — İKİ İZ

> Eski harita tek sıralı listeydi ve yazılımı donanım sırasına zincirliyordu.
> Gerçek şu: simülasyon varsa yazılım, donanım gelmeden adım 6'ya kadar
> ilerleyebilir. İki ize ayrıldı. **İz B'nin sırası atlanmaz; İz A paralel gider.**

### İz A — Yazılım (donanımsız, şimdi yapılabilir)
- ✅ **A1. Kinematik dünya + entegrasyon testleri repoda** — **BİTTİ**
  - `esp32_sim.py` (beyin) / arka uç ayrımı yapıldı; `fake_esp32` ince CLI oldu
  - `robot_sim` paketi: `world.py` (ground truth) + `sim_node.py` (sahte IMU/GPS)
  - Entegrasyon testleri repoda: E-stop, çarpma vetosu, watchdog, EKF vs ground truth
  - **Ölçülen:** ~8 m karede EKF hatası **0.08 m (%1)**, 35 sn'de yaw **4.3°**
- 🟡 **A2. Nav2 config** — **navigasyon katmanı BİTTİ, GPS waypoint kısmı BLOKE**
  - `config/nav2.yaml` + `launch/nav2.launch.py`: RPP + rotation shim, NavFn,
    rolling costmap. Testte doğrulandı: hedef verilince robot **gerçekten gidiyor**
    (ground truth ile ölçüldü, `test_nav2.py`).
  - ⛔ **GPS waypoint takibi manyetometreye takılı** — detay aşağıda. Nav2'nin
    suçu değil; `ekf_global`'ın yaw'ı gözlemlenemiyor.
  - ⚠️ **Costmap'ler boş** (menzil sensörü yok) → Nav2 burada bir **yol takipçisi**,
    engelden kaçınıcı değil. Ağaca güvenle sürer. Engel = A3.
- **A3. Gazebo arka ucu** — aynı `fake_esp32`'nin arkasına takılır (mimari kararı
  aşağıda), fizik + patinaj + engel dünyası. Eski adım 7'nin ön koşulu.
- ✅ **A4. Manyetometre sürücüsü** — **yazıldı** (`qmc5883l_driver` + madgwick),
  yaw sorunu çözüldü. Chip hâlâ alınmadı; sahte I2C'ye karşı doğrulandı.
- **A5. CI** (GitHub Actions: colcon build + testler + `pio run`) — ertelendi

### İz B — Donanım (sıra atlanmaz)
1. **ST-Link al → anakartı flash'la** (STM32, EFeru FOC) — **tüm izin kilidi**, en riskli adım
2. **ESP32 tezgah testi** — protokol + çarpma refleksi gerçek kartla
3. **Şasi** — en sıkıcı, en uzun
4. **KALİBRASYON** — `cmd_per_rpm`, `steer_sign`, `invert_left/right`,
   `wheel_separation`, `battery_scale`, `BUMP_BLOCKS_POSITIVE_SPEED`
5. **Gerçek teleop** — burada sürülebilir robot olur
6. **Gerçek sensörler** — IMU montajı + `imu_joint` rpy ölçümü, GPS, mag
7. **Sahada Nav2 + engelden kaçınma**

### İzlerin birleştiği yer: B4
Sim'in parametreleri şu an **tahmin**. B4'te ölçülen sabitler sim'e girince
sim'in tahminleri anlamlı olur. Sim'i şimdi kurmak = kalibrasyon günü hazır olmak.

### ⚠️ Simülasyonun kanıtlamadığı şeyler (fazla güvenme)
Bu projenin kullanıcı profili notu şunu diyor: *"Yazılım tarafı risksiz; riskler
fizik/elektrik/RF tarafında."* **Simülasyon bu risklerin hiçbirini azaltmaz.**
Patinaj, belgesiz TXTY kartının kaprisleri, GPS multipath, hub motorlarının
manyetometreyi bozması, UART gürültüsü — hiçbiri sim'de yok. Sim zaten düşük
riskli olan yazılımı sağlamlaştırır ve kalibrasyon gününü hızlandırır.
**İz A ne kadar ilerlerse ilerlesin, B1 projenin darboğazı olarak kalır.**

### Sim mimarisi kararı: tek sahte ESP32, iki arka uç
```
        /cmd_vel
            ↓
   hoverboard_bridge     ← GERÇEK kod, her iki dünyada da devrede
            ↓ pty / 0xABCD çerçeveleri
      fake_esp32 (protokol + watchdog + E-stop + çarpma vetosu)
         ╱        ╲
  --backend=kinematic   --backend=gazebo
   (hızlı, CI, ground    (fizik, patinaj,
    truth)                engel, kamera)
```
**Gerekçe:** Gazebo'nun kendi diff_drive eklentisini kullansaydık `hoverboard_bridge`,
seri protokol, watchdog ve çarpma vetosu simülasyonda **hiç çalışmazdı** — yani
robotta koşacak kodun bir kısmı hiç sınanmazdı. Bu kurguda protokol tek yerde
kalır ve gerçek yığın her iki dünyada da devrededir.

> **İstisna — IMU sürücüsü sim'de baypas edilir.** `sim_node` `/imu/data`'yı
> doğrudan ground truth'tan üretir; `mpu6050_driver` devreye girmez. Sebebi:
> o sürücünün değeri register seviyesinde ve zaten `fake_bus` ile birim test
> edilmiş — sim'de tekrar sınamak bir şey eklemez. Bilinçli bir boşluk.
>
> ⚠️ **Bunun bir bedeli var, A1'de bizzat ısırdı:** sürücüyü baypas edince
> `sim_node` çipin **ham çıktısını** değil, sürücünün **yayınladığı** şeyi
> modellemek zorunda. İlk halinde çipin ham 2.4 dps gyro bias'ını yayınlıyordu
> ve onu kimse çıkarmıyordu (gerçekte sürücü kalibre ediyor) → EKF 35 saniyede
> **100° saptı**. Doğrusu: `imu_gyro_residual_bias_dps` = kalibrasyon **sonrası
> artık** bias (varsayılan 0.1 dps).

### A1'in ölçtüğü sayılar (kinematik dünya, fizik YOK)
| Ölçüm | Değer |
|---|---|
| ~8 m karede EKF poz hatası | **0.083 m** (yolun %1'i) |
| 35 sn'de yaw hatası | **4.3°** |
| Yaw hatasının kaynağı | 0.1 dps artık gyro bias × 35 sn ≈ 3.5° — **neredeyse tamamı** |

> **Bu sayıları fazla okuma.** Patinaj yok, kütle yok, devrilme yok. "Matematik
> doğru bağlanmış" der; "robot bahçede iyi lokalize olur" DEMEZ. Patinaj gerçek
> odometri hatasının en büyük tek kaynağı ve bu dünyada hiç yok.
>
> **Ama bir şeyi net gösteriyor:** yaw sapmasının tamamı gyro artık bias'ının
> integralinden geliyor. 6-eksen IMU'da mutlak heading'i hiçbir şey gözlemlemiyor,
> yani bias ne kadar küçük olursa olsun **sonsuza kadar birikiyor**. Karar 4'ün
> "manyetometre en yüksek getirili harcama" demesinin sebebi artık bir sayı.
> `test_yaw_drifts_without_a_magnetometer` bunu kalıcı olarak belgeliyor.

## ŞU AN NEREDEYIZ / SIRADAKİ İŞ
Yazılım İz A'da: **A1 bitti, A2'nin navigasyon yarısı bitti**; donanım B1'de
(ST-Link) kilitli.

### ✅ A4 (manyetometre) yazıldı — yaw sorunu ÇÖZÜLDÜ, ölçüldü
`qmc5883l_driver` + `imu_filter_madgwick` devrede. Zincir:
```
mpu6050  ──► /imu/data_raw ─┐
                            ├──► imu_filter_madgwick ──► /imu/data (orientation VAR)
qmc5883l ──► /imu/mag ──────┘                               │
                                          ekf_global (yaw AÇIK) + navsat_transform
```
**Ölçüldü (dönüşlerle birlikte):**
| | truth | ekf_global | hata |
|---|---|---|---|
| başlangıç | +0.0° | +1.1° | **1.1°** |
| dönüş sonrası | +166.9° | +169.7° | **2.8°** |
| dönüş sonrası | -26.8° | -22.1° | **4.7°** |
| dönüş sonrası | -137.7° | -129.7° | **8.0°** |

**Öncesi -178° idi.** `ekf_local` hâlâ sapıyor (8→10°) — **kasıtlı**: mutlak yaw
sadece `ekf_global`'a veriliyor, `ekf_local`'a asla (kötü heading base_link'i
savurmasın; karar 4'ün zaten söylediği ayrım).

⚠️ **Chip HÂLÂ ALINMADI** (~100 TL). Sürücü `mpu6050_driver` deseniyle yazıldı
ve sahte I2C'ye karşı doğrulandı: kalibrasyonsuz hard iron **>15° yön hatası**
veriyor, kalibrasyonla **<0.5°**. Gerçek çipte `i2cdetect -y 1` ile 0x0D'yi gör.

### ⛔ AÇIK SORUN: GPS waypoint'i hâlâ tamamlanmıyor (ama sebebi ARTIK yaw DEĞİL)
Manyetometre sonrası: waypoint 0 **başarılı**, waypoint 1'de Nav2 hedefi
(7.45, 2.94) iken robot 2 dakika **doğuya sürüp 40 m'ye gitti**, sonra "Goal failed".

**Lokalizasyon suçsuz — ölçüldü:** ground truth (40.23, 3.37), `/odometry/gps`
(38.45, 3.63), `ekf_global` (38.14, 3.15) → üçü ~2 m içinde uyuşuyor. Yani
konum ve yaw doğru; **kontrolcü robotu yanlış yöne sürüyor**.

Sıradaki oturumun ilk işi bu. Şüpheler (sırayla denenecek):
1. `ekf_global`'ın yaw düzeltmeleri `map→odom`'u sıçratıyor → kontrolcünün
   base_link'e dönüştürdüğü yol sürekli dönüyor, robot kaçan havucu kovalıyor.
   Kontrol: `/received_global_plan` ve `/lookahead_point`'i sürüş sırasında izle.
2. `ekf_local` yaw'ı (10° sapma) ile `ekf_global` yaw'ı arasındaki fark
   `map→odom`'a yığılıyor.
3. RPP'nin `min_lookahead_dist: 1.2` + `xy_goal_tolerance: 1.0` ayarı A2'de
   NavigateToPose için ayarlandı; GPS gürültüsüyle birlikte yeniden bakılmalı.

### (tarihçe) A2 sırasında GPS'in bloke olma sebebi — çözüldü
`navsat_transform`, robotun **mutlak yönünü** `/imu/data`'nın orientation
quaternion'undan okuyor. Bizim 6-eksen IMU'muzda orientation YOK ve bunu açıkça
söylüyor (`orientation_covariance[0] = -1`). **`navsat_transform` o bayrağı
kontrol etmiyor** — birim quaternion'u okuyup "robot doğuya bakıyor" sonucuna
varıyor. Aynı anda `ekf_global`'ın yaw'ı da gözlemlenemez durumda (hiçbir yerde
mutlak yön yok), GPS konum düzeltmeleri onu döndürüyor.

**Ölçüldü:** ground truth yaw **+38.8°** iken `ekf_global` **-178.2°** dedi,
sonra **-47.1°**. `ekf_global` `map→odom`'u yayınladığı için oradaki yanlış
rotasyon **Nav2'nin her hedefini döndürüyor** → robot kuzeydeki waypoint'e hiç
gitmedi ama Nav2 "vardım" dedi.

**Simülasyonda kısmen çalışıyor olması ŞANS:** `robot_sim` robotu yaw=0
(doğu) başlatıyor, birim quaternion da tesadüfen bunu söylüyor. Gerçek robot
kuzeye bakarak başlarsa her GPS waypoint'i 90° şaşar.

Çözüm Nav2 parametresi değil: **QMC5883L** (karar 4, A4/B6) ya da yön-başlatma
manevrası + `use_odometry_yaw: true`. **Manyetometre artık İz A'nın da blokeri.**

Tamamlananlar:
- ✅ ROS 2 workspace iskeleti, sahte ESP32'ye karşı doğrulandı
- ✅ IMU sürücüsü (`mpu6050_driver`). Kalan: gerçek chip gelince `i2cdetect -y 1`
  ile 0x68'i gör, `imu_joint` rpy'ını gerçek montaj yönüne göre ölç/yaz (B6).
- ✅ Çarpma refleksi — yönlü veto. Ultrasonik **bilinçli yapılmadı**.
- ✅ **A1: kinematik dünya + kalıcı entegrasyon testleri**

### Testleri koşmak
```bash
cd ros2 && source install/setup.bash
python3 -m pytest src/hoverboard_bridge/test -q   # 20 birim + 3 entegrasyon (~50 sn)
python3 -m pytest src/robot_sim/test -q           # 10 birim + 3 lokalizasyon + 1 nav2 (~140 sn)
python3 -m pytest src/mpu6050_driver/test -q      # 16 birim (~0.1 sn)
# hepsi: 53 test, ~190 sn
```
**ROS'suz da koşarlar:** protokol ve dünya birim testleri saf Python (bilinçli
tasarım); entegrasyon testleri `importorskip` ile temizce atlanır. Hook bunu
her düzenlemede zorluyor.

### Donanımsız tam yığın
```bash
ros2 run robot_sim sim_node
ros2 launch robot_bringup robot.launch.py esp32_port:=/tmp/fake_esp32 \
    use_localization:=true use_imu:=false     # sim_node /imu/data'yı kendi yayınlıyor
```

⚠️ **Neden A1 ilk işti:** E-stop, çarpma vetosu ve EKF doğrulamaları scratchpad'de
(`/tmp`) yazılmıştı; devcontainer rebuild'i sildi, aynı test iki kez yazıldı.
Artık repodalar.

## Bilinen blokerler / bekleyen alımlar
- **ST-Link V2 klon (~150 TL)** — adım 1 için şart, henüz alınmadı
- **Magnetometer QMC5883L (~100 TL)** — listedeki en yüksek getirili harcama
- Multimetre doğrulaması (UART pinout) — kart elde olunca
- Caster ×2 (125-150 mm kauçuk), mantar E-stop + kontaktör, sigorta
- ✅ ~~`ros-jazzy-nmea-navsat-driver` Jazzy apt'de olmayabilir~~ — **VAR**
  (2.0.1-3noble, apt'te doğrulandı). Bloker değil.
- `camera_ros` apt'te **var** (0.6.0-1noble) ama Dockerfile'a eklenmedi
  (`sensors.launch.py` `use_camera:=true` ile onu çağırıyor) — kamera işine
  gelince Dockerfile'a ekle, engel yok.
- Gazebo: `ros-jazzy-ros-gz` 1.0.22 apt'te **var** → A3 için engel yok.
  Makine: 4 çekirdek / 7 GB RAM (WSL2) — fizik simülasyonu koşar ama hızlı değil.
- ⚠️ **Devcontainer rebuild bekliyor.** Dockerfile'a iki şey eklendi ama imaj
  yeniden derlenip doğrulanmadı (mevcut konteyner eski imajdan koşuyor):
  1. `python3-smbus2` (IMU sürücüsü için). Paketin noble/**universe**'te olduğu
     Launchpad'den teyit edildi (0.4.3-1), konteynerde universe açık → gelmeli.
     Pi tarafında `rosdep install` zaten çeker.
  2. **Parolasız sudo** (`/etc/sudoers.d/ubuntu`). Sebep: taban imajın `ubuntu`
     kullanıcısı `sudo` grubunda ama parolası kilitli → sudo kimsenin bilmediği
     bir parola soruyordu. `visudo -c` build guard'ı var. Detay: `docs/devcontainer.md`.

  Rebuild sonrası kontrol: `sudo -n true && python3 -c "import smbus2"`
