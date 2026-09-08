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
10. ⚠️ **Sahte sensörler elle türetilmiş fiziğe çivilenir, kendi matematiğine
    değil** (2026-07-17, A6'dan sonra alındı). Bir fikstürü kendi formülüyle
    test etmek, onun sadece kendisiyle tutarlı olduğunu kanıtlar. Somut bedeli:
    `fake_bus` alanı aynalıyordu, testin `heading_of`'u aynı işareti geri
    çeviriyordu, 17 test yeşildi ve A2 haftalarca "bozuk" göründü. Kural:
    her sahte sensör için elle hesaplanmış sayılara karşı **en az bir** test
    (`test_field_matches_hand_computed_physics` deseni).
11. ⚠️ **Yönle ilgili her şey SÜPÜRÜLEREK test edilir, tek noktada değil**
    (2026-07-17). Ayna, 90° montaj ofseti ve yanlış declination — üçü de tek bir
    yönde kusursuz görünür, ve tüm testlerimiz yaw=0'da başlıyordu (`sin(0)=0`,
    aynalı ve doğru alan orada birebir aynı). Yön testleri robotu **tam tur**
    döndürmeli (`test_absolute_yaw_holds_through_a_full_turn`).

12. ⚠️ **Engel haritası `map` frame'inde yayınlanır, `sim_world`'de değil**
    (2026-09-08). Bu bir etiket seçimi değil, bir **iddia**: engellerin robotun
    navigasyon yaptığı **aynı datum'da survey edildiğini** söyler. Böylece datum
    hatası ortak-mod olur — haritayı ve robotun kendi konum tahminini birlikte
    kaydırır, yani kapanır. Robotun kendi sensörüyle kurduğu harita **değildir**;
    onun hatası canlı lokalizasyon hatasıdır ve kapanmaz, ayrıca menzil sensörü
    ister (envanterde yok). Ayrım `nav2.yaml`'ın başlığında da yazılı:
    **survey edilmiş** engeli dolanır, **survey edilmemiş** engeli göremez ve
    içine sürer.

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

## A3c'de bulunan tuzaklar (costmap'lere ilk kez gerçek engel konunca)
1. **StaticLayer, harita gelmezse costmap'i hiç hazır etmiyor.** "Engel yoksa
   harita yayınlama" kararı engelsiz her koşuda Nav2'yi tamamen öldürdü — her
   hedef `status 6` ile abort. Boş harita **her zaman** yayınlanmalı: sessizlik
   "engel yok" demek değil, "planlayıcı hiç başlamadı" demek. Bunu yakalayan
   şey **karşıt testti**; kabul testi tek başına yemyeşildi.
2. **Inflation hiç engelle sınanmamıştı.** `inflation_radius: 0.55`
   (robot_radius + 0.15) NavFn için yeterli ama RPP için değil: 1.5 m lookahead
   ile viraj kesiyor ve planlanan yolun **içinden** geçiyor. Ölçülen: robot
   duvar ucuna tam 0.70 m temas mesafesinde sıkıştı. Marj robotun gövdesini
   değil **kontrolcünün viraj kesmesini** de kapatmalı → `1.0` /
   `cost_scaling_factor 2.0`.
3. ⚠️⚠️ **SIKIŞAN ROBOT İÇİN NAV2 "SUCCEEDED" DİYOR.** Çarpışmada tekerlekler
   dönmeye devam ediyor → hall sensörleri komut hızını bildiriyor → odometri
   entegre ediyor → EKF inanıyor (yaw gyro'dan geliyor ama **vx tekerlekten** ve
   onu yalanlayan hiçbir şey yok) → goal checker vardığını görüyor. Sadece
   ground truth hareket etmediğini biliyor. **Gerçek robot da aynen böyle
   davranacak** — kayaya dayanmış hub motorlar sağlıklı 0.5 m/s bildirir.
   `test_an_unsurveyed_obstacle_jams_the_robot_and_nav2_claims_success` bunu
   kalıcı belgeliyor; **düzeltildiği gün test patlar, amacı bu.**
   Çözüm "robot gerçekten hareket ediyor mu" sorusuna ikinci bir görüş ister ve
   adayların hepsi donanıma bağlı: GPS (yaw sorunu bu yığında `ekf_global`'ı
   dışarıda tutuyor), INA228 akımı (SP1 yazılımı hazır, sensör alınmadı),
   tampon (sadece temas, bu duvara dik giriliyor).
4. **`use_collision_detection: false`'ın gerekçesi çürüdü.** Kapalı olma sebebi
   "costmap boş, kontrol her zaman geçer" idi — ve geçmeyen bir kontrol,
   olmayandan **kötüdür**, çünkü kontrol gibi okunur. Costmap dolunca açıldı.
5. **`wait_for_server` bir hazır olma kontrolü DEĞİL.** Nav2 action server'ını
   CONFIGURE'da açıyor ama ACTIVATE'e kadar hedefi reddediyor; ikisi client'tan
   ayırt edilemiyor. Sabit uykuyla kurulan test yükte kararsızlaştı ("Nav2
   rejected the goal" — navigasyon hatası gibi okunur, aslında yarıştır).
   Kabul edilene kadar tekrar denenmeli.

## A3'te bulunan tuzaklar (fizik ilk kez GERÇEKTEN koşunca)
A3a "yazılmış" görünüyordu: kod, SDF, launch, birim testleri, hepsi yeşil. Fizik
bir kez bile koşturulmamıştı ve **aşağıdakilerin tamamı sessizce yanlıştı.**

1. ⚠️⚠️ **Gazebo DiffDrive'ın `/odometry`'si GROUND TRUTH DEĞİL.** Tekerlek
   joint açılarından yaptığı ölü hesap — yani tam olarak hall sensörlerinin
   muadili. Tekerlekler boşta dönerken de mesafe saymaya devam eder.
   **Ölçüldü:** eski modelde `/odometry` **5.47 m** derken gerçek poz
   **3.8e-9 m** idi; robot hiç kımıldamamıştı. Önceki oturumun elle bakıp
   "model GERÇEKTEN SÜRÜYOR (x: 0 → 14.2 m)" notu **bu yüzden yanlıştı**.
   Ground truth artık ayrı bir `OdometryPublisher`'dan geliyor. İkisi tek
   topic'e düşerse `/ground_truth` ile `/odom` **tanım gereği** uyuşur ve bu
   dünyada ölçülen her lokalizasyon sayısı totolojiye döner.
2. **`<pose>`'suz bir `<link>` model orijininde durur ve joint'in `<pose>`'u
   çocuğunu TAŞIMAZ.** İki tekerlek de orijinde, gövde kutusunun içine gömülü,
   hiçbir şeye değmiyordu; okuyanın "tekerlek yerleşimi" sandığı `<pose>`'lar
   joint'lerin üzerindeydi. Gazebo yine de onları hız kontrolüyle döndürdü,
   DiffDrive yine de odometri yayınladı. Yüzeydeki her işaret ("sim koşuyor,
   model spawn oluyor, odometri artıyor") çalışan bir robota benziyordu.
3. **`bridge.yaml` ile SDF farklı topic isimleri söylüyordu** ve köprü bunu
   şikâyet etmez: GZ→ROS tarafı sessiz kalır, ROS→GZ tarafı her komutu yutar.
   Köprünün tamamı hiçbir şeye bağlı değildi. Artık `test_gazebo_config.py` iki
   dosyayı birlikte ayrıştırıp karşılaştırıyor — **Gazebo'suz, 0.06 saniyede.**
4. **`/clock` köprülenmemişti VE `use_sim_time` hiçbir yerde set edilmiyordu.**
   İkisi aynı anda eksik olduğu için ikisi de görünmedi: `/clock` olmadan sim
   zamanına ayarlı bir düğüm asılı kalır ve biri fark ederdi. `use_sim_time`
   argümanı description/localization/nav2'de vardı, **`robot.launch.py` onu hiç
   tanımlamıyordu**; şimdi köprü ve battery dahil her düğüme geçiyor.
   ⚠️⚠️ **GERÇEK ROBOTTA ASLA `true` OLMAZ** (varsayılan `false`). Sebebi
   **ölçüldü** (`use_sim_time=True`, `/clock` yayınlayan yok, 4 sn):
   **rclpy timer'ı 0 kez tetiklendi.** Yani `_tx_tick` hiç koşmaz — köprü
   komut da göndermez, odometri de, diagnostics de. Motorlar durur (ESP32'nin
   200 ms watchdog'u devreye girer), *yani güvenli tarafa düşer*; ama
   drivetrain düğümü **sessizce tamamen ölmüştür** ve sebebini söyleyen hiçbir
   log yoktur. Robot durur ve neden durduğunu kimse bilmez.
   ⚠️ Not: burada bir kez "son `/cmd_vel` gönderilmeye devam eder" diye
   yazılmıştı — **yanlıştı, ölçülmeden yazılmıştı.** Timer koşmadığı için
   tekrar gönderim de yok. Sonuç (robotta asla açma) aynı, gerekçe değil.
   Ölçüm artık test:
   `test_sim_time_without_a_clock_silences_the_bridge_instead_of_freezing_it`
   (hoverboard_bridge). Karşıt kontrol de yapıldı — `use_sim_time` kapalıyken
   aynı timer 4 sn'de **80 kez** tetikleniyor, yani test boş değil. İleride
   rclpy donmuş saatte timer koşturmaya başlarsa test patlar, ve o gün bu uyarı
   yorumdan çıkıp `bridge_node`'a gerçek bir korumaya dönüşmek zorunda.
5. **"RTF 2-3" iddiası yanlıştı.** `/stats`'tan ölçüldü: `<physics>` elementi
   hiç yokken bile step **1 ms**, gerçekleşen RTF **1.0000**. `empty.sdf`'e
   yine de açık `<physics>` konuldu — davranışı değiştirmiyor, sayıyı
   Gazebo sürüm yükseltmesinin sessizce kaydıramayacağı hale getiriyor.
6. **gz-sim, contact sensörünün topic'ini KENDİ üretir ve SDF'teki `<topic>`'u
   yok sayar.** Gerçek isim `/world/<dünya>/model/<model>/link/<link>/sensor/
   <sensör>/contact`, yani dünya veya model adını değiştirmek
   `/collision_truth`'u sessizce kör eder. `test_gazebo_config.py` ismi
   kopyalamak yerine yeniden kuruyor.
7. **İki `gz` sunucusu hata değil, sessiz felakettir.** İkisi de
   `/world/empty/...` üzerinde cevap verir: engel bir dünyaya spawn olur, robot
   ötekinde sürer, köprü hangisini bulduysa ona bağlanır. Bu yaşandı — test
   "x=4.93'te durdu, 2.15 bekleniyordu" dedi ve bozuk fizik gibi okundu.
   Fixture artık aynı anda tek dünyaya izin veriyor ve önce kalıntı arıyor.
8. **`/odom` ile `/ground_truth`'un en yeni mesajları AYNI ANA ait değil.**
   `/odom` pty gidiş-dönüşünün gecikmesini taşır; 1 m/s'de iki örnek ~10 ms ve
   ~10 mm ayrı düşüyordu — ölçülmek istenen patinajla aynı büyüklükte ve
   makul görünen bir işarette. İkisi tek damgaya interpole ediliyor; bu
   düzeltme patinajsız kontrol koşusunu **−19 mm'den −2 mm'ye** taşıdı.
   (Bu arada köprüde sistematik bir odometri sapması olduğu şüphesi de böyle
   çürüdü: sapma benim ölçüm kusurumdu.)

### A3'te ölçülen sayılar (Gazebo, fizik VAR)
Tam zincir: `/cmd_vel` → `hoverboard_bridge` → pty → `Esp32Sim` →
`GazeboBackend` → Gazebo → hall → `/odom`, `/ground_truth`'a karşı.

| Ölçüm | Değer |
|---|---|
| Durdan **adım** komutla 2.5 m'de patinaj, v=0.25 / 0.5 / 1.0 m/s | **+8.4 / +35.0 / +143.4 mm** |
| Elle türetilen alt sınır `v²/(2·μ·g)`, μ=0.5 | 6.4 / 25.5 / 102.0 mm |
| Ölçülenin sınıra oranı | 1.31 / 1.37 / 1.41 (her hızda aynı) |
| Aynı 2.5 m, aynı 1.0 m/s, ama **rampalı** (karşıt test) | **−2.4 mm** |
| 3 tekrarın yayılımı | < 2 mm |
| 8 yönde gerçek rota ile burun yönü farkı | **0.02°** |
| Engele çarpma noktası (elle: 3.0 − 0.5 − 0.35) | **2.1500 m** |
| Sıkışmışken 6 sn'de: gerçek / `/odom` | **0.0000 m / +2.97 m** |

> **Patinajın neden alt sınırı elle türetilebiliyor:** DiffDrive joint'leri hızla
> sürdüğü için adım komutta temas yüzeyi anında `v`'ye fırlar, gövde ise hâlâ
> durur — patinaj başlar. Sürtünme gövdeyi en fazla `a = μ·g` ile hızlandırabilir
> (bu, tahrikli tekerleklerin ağırlığın TAMAMINI taşıdığı hal; bizimkiler
> taşımıyor, hızlanınca yük arka caster'a biner). Patinaj gövde `v`'ye ulaşınca
> biter: tekerlek `v·t` rapor ederken gövde `v·t/2` gitmiştir, fark `v²/(2a)`.
> `μ·g` bunu **bir tavan hızlanma** yaptığı için sonuç bir **alt sınır**, uydurma
> bir eğri uydurması değil. Ölçülenin sınıra oranının her hızda 1.4 çıkması da
> tam olarak yük transferi açığının imzası.
>
> **Karşıt test neden ŞART:** patinaj tek başına "odometri fazla sayıyor"
> demekten ibaret ve bunu bir tekerlek yarıçapı / birim-başına-rpm ölçek hatası
> da yapar. Ayıran şey **sabit mesafede** ölçmek: ölçek hatası her hızda AYNI
> fazlalığı verir, sürtünme patinajı `v²` ile büyür. Ölçülen oranlar 4.17 ve
> 4.10 (beklenen 4.0); ölçek hatası 1.0 verirdi. İkinci karşıt test rampa: aynı
> hız, aynı mesafe, sadece lastikten sahip olmadığı kuvvet istenmiyor → patinaj
> 60 kat küçülüyor ve işareti dönüyor.

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
ros2/src/robot_sim/         iki dünya (kinematik + Gazebo fiziği) + ground truth + sahte IMU/mag/GPS + engel haritası + yığın testleri
ros2/src/robot_sim/gazebo/  empty.sdf (dünya + fizik), hoverbot.sdf (model), bridge.yaml (ROS-GZ eşlemesi)
ros2/src/qmc5883l_driver/   manyetometre sürücüsü + sahte I2C (mutlak yönün tek kaynağı)
ros2/src/ina228_driver/     INA228 register seviyesi akım sürücüsü + sahte I2C
ros2/src/battery_manager/   SoC tahmini + yetkili /battery yayıncısı
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

### Batarya izleme (SP1)
`ina228_driver` ve `battery_manager` yazılımı donanımsız tamamlandı. INA228,
köprünün ham `/battery_raw` voltajını ve Pi I2C akımını birleştirerek tek yetkili
`/battery` mesajını yayınlar; INA228 yoksa düğüm voltaj-yalnız moda düşer
(`current` ve `percentage` = NaN, diagnostics WARN). SoC açılışta dinlenim
voltajından başlar, akımı coulomb sayar ve tam şarj kuyruğunda %100'e sıfırlanır.

Gerçek INA228 ve şönt henüz alınmadı. `shunt_ohms`, `invert_current` ve
`capacity_ah` gerçek paket üzerinde kalibre edilmelidir; şöntün BMS içindeki
ortak eksi hattı multimetreyle doğrulanmadan bağlanmamalıdır. SP1, motor komut
yoluna ve ESP32 seri protokolüne dokunmaz.

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
| `robot_sim/sim_node.py` | dünyayı koşturur; `/ground_truth` (frame **`sim_world`**), sahte `/imu/data_raw`, `/imu/mag`, `/gps/fix`, latch'li `/obstacle_map` (frame **`map`** — karar 12) ve `/collision_truth` yayınlar. GPS hatası Ornstein-Uhlenbeck (beyaz değil — beyaz gürültü EKF'te fazla güzel ortalanırdı). `ros2 run robot_sim sim_node` |
| `robot_sim/obstacle_map.py` | `GridSpec` + `build_obstacle_grid` + `parse_obstacle_params`. Grid geometrisinin **tek kaynağı**. Harita **çıplak** engeli işaretler; robot yarıçapını costmap inflation ekler, iki kez şişirilmez. |
| `mpu6050_driver/mpu6050.py` | register seviyesi MPU6050 (ROS'suz, I2C bus enjekte edilir) |
| `mpu6050_driver/imu_node.py` | **`/imu/data_raw`** + `/imu/temperature` + `/diagnostics`; açılışta gyro bias kalibrasyonu. ⚠️ `data_raw`, `data` değil: ROS geleneğinde `/imu/data` orientation taşır, bu düğüm taşımıyor (`orientation_covariance[0] = -1`). |
| `mpu6050_driver/fake_bus.py` | **sahte I2C chip** — register seviyesinde, config register'larını geri çözüp ölçekliyor |
| `qmc5883l_driver/qmc5883l.py` | register seviyesi QMC5883L. ⚠️ **little-endian** — aynı bus'taki MPU6050 big-endian. Karıştırmak hata vermez, sadece yönü döndürür; testle çivili. |
| `qmc5883l_driver/mag_node.py` | `/imu/mag`; hard/soft iron uygular, OVL örneklerini düşürür, kalibrasyonsuzken bağırır. **Yön HESAPLAMAZ** — çıplak pusula okuması sadece robot düzken yöndür; eğim telafisi madgwick'in işi. |
| `qmc5883l_driver/fake_bus.py` | sahte I2C chip; hard iron + 12-bit kuantizasyon modeller. Dünya alanını **hesaplamaz**, `earth_field`'dan alır. |
| `qmc5883l_driver/earth_field.py` | Simüle dünya alanının **tek tanımı** (`field_in_body_frame`). A6'nın kalan borcu buydu; `sim_node` ve `fake_bus` ikisi de kendi kopyasını taşıyordu. Sürücü paketinde duruyor çünkü sürücü sim'siz de deploy edilebilmeli. |
| `robot_bringup/config/ekf.yaml` | çift-EKF + navsat_transform, gerekçeleri yorumda |
| `robot_bringup/config/nav2.yaml` | RPP + rotation shim + NavFn; **static_layer** `/obstacle_map`'i okur. Başlıkta survey edilmiş / edilmemiş engel ayrımı. |
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
- ✅ **A2. Nav2 config** — **BİTTİ, GPS waypoint takibi uçtan uca doğrulandı**
  - `config/nav2.yaml` + `launch/nav2.launch.py`: RPP + rotation shim, NavFn,
    rolling costmap. Testte doğrulandı: hedef verilince robot **gerçekten gidiyor**
    (ground truth ile ölçüldü, `test_nav2.py`).
  - **GPS waypoint ölçüldü:** L rotası (6 m doğu + 4 m kuzey) → `status 4
    SUCCEEDED, missed=0, gerçek hata 0.73 m`. Uzun süre "bloke" görünüyordu;
    sebep Nav2 değil, **simülatörün aynalı manyetometresiydi** (A6).
  - ⚠️ ~~**Costmap'ler boş** → Nav2 bir yol takipçisi, engelden kaçınıcı
    değil~~ — **A3c'de değişti.** Costmap'ler artık bir **a priori survey
    haritası** taşıyor ve Nav2 ondaki engelleri gerçekten dolanıyor (ölçüldü:
    2.00 m sapma). Ama menzil sensörü hâlâ yok, yani **survey edilmemiş** engel
    görünmez: haritaya girilmemiş ağaca hâlâ güvenle sürer. Ayrım kritik.
- ✅ **A3. Gazebo arka ucu — BİTTİ, fizik uçtan uca ölçüldü (2026-09-08).**
  Aynı `fake_esp32`'nin arkasına takılıyor (mimari kararı aşağıda): fizik,
  patinaj ve engel dünyası. Detaylar aşağıdaki A3 dilimlerinde.
- ✅ **A4. Manyetometre sürücüsü** — **yazıldı** (`qmc5883l_driver` + madgwick),
  yaw sorunu çözüldü. Chip hâlâ alınmadı; sahte I2C'ye karşı doğrulandı.
- ✅ **A6. Aynalı manyetometre hatası** — sim'deki tek eksi işareti; A2'yi ve
  A4'ün "kanıtını" sahte kılmıştı. Düzeltildi + tam-tur regresyon testi eklendi.
- ✅ **SP1. Batarya izleme yazılımı** — INA228 register sürücüsü, sahte I2C,
  coulomb sayan SoC ve sensör yokken voltaj-yalnız `/battery` modu yazıldı;
  launch'ta köprü `/battery_raw`, `battery_monitor` `/battery` yayınlıyor.
- ✅ **A3a. Gazebo arka uç ilk dilim** — `GazeboBackend`, `hoverbot.sdf`,
  `bridge.yaml`, `gazebo.launch.py` ve birim testleri yazıldı.
  ⚠️ Bu dilim **hiç fizik koşturmadan** yazılmıştı ve içindeki her şey
  sessizce yanlıştı (aşağıdaki A3 tuzakları). Ders: "kod yazıldı" ile "çalışıyor"
  arasındaki farkı sadece kabul testi kapatır.
- ✅ **A3b. Slip modeli (kinematik)** — `KinematicWorld` `slip_factor` ile
  tekerlek hız kaybını modelleyebiliyor; varsayılan `0.0`.
  ⚠️ Bu **elle verilen bir sayı**, fizikten çıkmıyor: ne kadar patinaj olacağını
  sen söylüyorsun. Gazebo backend'inde `slip_factor` **reddediliyor** — orada
  patinaj sürtünme, kütle ve yük transferinden çıkar ve tek düğme
  `hoverbot.sdf`'teki tekerlek `mu`'su.
- ✅ ~~Gazebo `gz-sim-diff-drive-system` plugin'i container'da kurulu değil~~ —
  **YANLIŞ, düzeltildi (2026-09-08):** plugin
  `/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins/` altında **kurulu**,
  `gz sim` 8.11.0 headless koşuyor.
- ✅ **A3d. Fizik kabul testi — BİTTİ (2026-09-08).** `/cmd_vel` → köprü → pty →
  `Esp32Sim` → Gazebo → `/odom`, ground truth'a karşı ölçüldü. Model geometrisi
  baştan yazıldı (tekerlekler gerçekten yere değiyor), ground truth ayrı bir
  `OdometryPublisher`'dan geliyor, `/clock` + `use_sim_time` zinciri kuruldu,
  engeller Gazebo'ya spawn ediliyor. **`test_gazebo_physics.py` (6 kabul testi,
  ~2.5 dk) + `test_gazebo_config.py` (8 statik test, 0.06 sn).**
  Ölçülen sayılar ve bulunan tuzaklar aşağıda ayrı bölümde.
- ✅ **A3c. Obstacle/collision — BİTTİ, uçtan uca ölçüldü (2026-09-08).**
  `KinematicWorld` dairesel engelde duruyor; `obstacle_map.py` engelleri
  OccupancyGrid'e çeviriyor; `nav2.yaml`'ın her iki costmap'inde `static_layer`
  `/obstacle_map`'i okuyor; NavFn duvarı dolanıyor.
  **Ölçülen (ground truth):**
  | senaryo | sapma | en yakın geçiş | hedef hatası |
  |---|---|---|---|
  | survey edilmiş duvar | **2.00 m** | **1.06 m** (temas 0.70) | 0.69 m |
  | engelsiz (karşıt test) | **0.29 m** | — | — |
  Karşıt test şart: sapmanın sebebinin engel olduğunu, kontrolcünün kendi
  salınımı olmadığını ayırt eden tek şey o. Bulunan tuzaklar aşağıda ayrı
  bölümde — biri Nav2'yi engelsiz durumda tamamen öldürüyordu.
  ✅ ~~**Gazebo'da engel YOK**~~ — **A3d'de kapandı:** engeller artık
  `gazebo_obstacles.py` ile aynı ayrıştırılmış listeden hem OccupancyGrid'e hem
  Gazebo geometrisine dönüyor, `sim_node` gazebo backend'inde engeli reddetmiyor.
  Ölçülen: robot elle hesaplanan **2.1500 m**'de duruyor (3.0 − 0.5 − 0.35).
- **A5. CI** (GitHub Actions: colcon build + testler + `pio run`) — ertelendi

### İz B — Donanım (sıra atlanmaz)
1. **ST-Link al → anakartı flash'la** (STM32, EFeru FOC) — **tüm izin kilidi**, en riskli adım
2. **ESP32 tezgah testi** — protokol + çarpma refleksi gerçek kartla
3. **Şasi** — en sıkıcı, en uzun
4. **KALİBRASYON** — `cmd_per_rpm`, `steer_sign`, `invert_left/right`,
  `wheel_separation`, `battery_scale`, `BUMP_BLOCKS_POSITIVE_SPEED`,
  `shunt_ohms`, `invert_current`, `capacity_ah`; INA228 şönt yerleşimi de
  multimetreyle doğrulanacak
5. **Gerçek teleop** — burada sürülebilir robot olur
6. **Gerçek sensörler** — IMU montajı + `imu_joint` rpy ölçümü, GPS, mag
7. **Sahada Nav2 + engelden kaçınma**

### İzlerin birleştiği yer: B4
Sim'in parametreleri şu an **tahmin**. B4'te ölçülen sabitler sim'e girince
sim'in tahminleri anlamlı olur. Sim'i şimdi kurmak = kalibrasyon günü hazır olmak.

### ⚠️ Simülasyonun kanıtlamadığı şeyler (fazla güvenme)
Bu projenin kullanıcı profili notu şunu diyor: *"Yazılım tarafı risksiz; riskler
fizik/elektrik/RF tarafında."* **Simülasyon bu risklerin hiçbirini azaltmaz.**
Belgesiz TXTY kartının kaprisleri, GPS multipath, hub motorlarının
manyetometreyi bozması, UART gürültüsü — hiçbiri sim'de yok.

⚠️ **Patinaj artık Gazebo backend'inde VAR, ama sayı hâlâ tahmin.** A3d
patinajın *mekanizmasını* getirdi (sürtünme limiti, yük transferi) ve bunu
yazılımın patinaja nasıl davrandığını sınayacak kadar gerçek kıldı. Ne kadar
patinaj olacağını belirleyen `hoverbot.sdf`'teki `mu = 0.5` ise `robot_radius`
gibi **belgelenmiş bir tahmin** — çim/toprak üstünde kauçuk. Gerçek sayı B4'ten
gelir. Yani: "yığın patinajla başa çıkıyor" denebilir, "bahçede %3 odometri
hatası olur" denemez. Sim zaten düşük
riskli olan yazılımı sağlamlaştırır ve kalibrasyon gününü hızlandırır.
**İz A ne kadar ilerlerse ilerlesin, B1 projenin darboğazı olarak kalır.**

⚠️ **Dahası: sim'in KENDİSİ hata kaynağı.** A6 bunu pahalıya öğretti — simülatör
dünyanın manyetik alanını aynalıyordu ve tüm testler yeşilken A2'yi haftalarca
"bozuk" gösterdi. Sim bir yalan söylediğinde, ölçtüğün her şey o yalanı ölçer.
Korunma yolları (A6'dan çıkanlar):
- **Fikstürü kendi matematiğiyle test etme.** Testin yardımcısı fikstürün
  hatasını tersine çeviriyorsa, ikisinin birbiriyle uyuştuğunu kanıtlarsın.
  Elle türetilmiş sayılara çivile (`test_field_matches_hand_computed_physics`).
- **Tek bir çalışma noktasında test etme.** Ayna, 90° montaj ofseti ve yanlış
  declination — üçü de tek bir yönde kusursuz görünür. **Süpür** (tam tur).
- **Elle koşulan ölçüm kanıt değildir.** A4'ün tablosu elle alınmıştı ve yanlıştı.
  Ölçebiliyorsan teste çevir; çeviremiyorsan kanıtlamış sayma.

### Sim mimarisi kararı: tek sahte ESP32, iki arka uç
```
        /cmd_vel
            ↓
   hoverboard_bridge     ← GERÇEK kod, her iki dünyada da devrede
            ↓ pty / 0xABCD çerçeveleri
      fake_esp32 (protokol + watchdog + E-stop + çarpma vetosu)
         ╱        ╲
  --backend=kinematic   --backend=gazebo
   (hızlı, CI, ground    (kütle, sürtünme,
    truth, patinaj YOK)   PATİNAJ, engel)
```
**A3d'den sonra ikisi de gerçek.** Gazebo tarafında topic ayrımı kritik:
```
  DiffDrive       → /model/hoverbot/wheel_odometry   TEKERLEK ne dedi (hall muadili)
  OdometryPublisher → /model/hoverbot/ground_truth   robot NEREDE (cevap anahtarı)
  contact sensor  → .../chassis_contact/contact      gövde bir şeye DEĞDİ mi
```
İlk ikisi **ayrı eklentiler olmak zorunda**; aradaki fark patinajın ta kendisi.
Tek kaynağa düşerlerse `/ground_truth` ile `/odom` tanım gereği uyuşur.
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
*(son güncelleme: 2026-09-08)*

Yazılım İz A'da: **A1, A2, A3 (a–d, tamamı), A4, A6 (borcu dahil) ve SP1 bitti**;
donanım B1'de (ST-Link) kilitli. Tam workspace doğrulaması **140 test**
geçti (robot_sim 59, hoverboard_bridge 24, mpu6050 16, qmc5883l 23,
ina228 8, battery_manager 10), atlanan yok, **7 paketin tamamı** temiz build ediyor.
Ölçülen süre: **9 dk 15 sn**.

⚠️ **Suite artık ~9.5 dakika** — dört Nav2 yığını ve iki Gazebo dünyası
sırayla kalkıyor. Elle koşma alışkanlığı bu süreyle zayıflar; **A5 (CI) artık
listedeki en yüksek getirili iş.** Ara adım olarak hızlı süzgeç:
`pytest src/robot_sim/test -q -k "not gazebo_physics and not nav2"` (saniyeler,
ve A3'ün asıl hatasını yakalayan `test_gazebo_config.py` bunun içinde).

Sıradaki iş seçenekleri:
- **A5 (CI)** — en yüksek getirili. A6, "engelsiz Nav2 tamamen ölüyordu"
  regresyonu ve A3'ün "köprü hiçbir şeye bağlı değil"i — üçü de tam olarak
  CI'ın yakalayacağı tür.
- **Sıkışma tespiti** — A3c tuzak 3, artık **gerçek fizikle de kanıtlı**:
  engele dayanmış robotta ground truth 6 sn'de 0.0000 m ilerlerken `/odom`
  +2.97 m saydı. Yazılımla çözülebilir kısmı yok gibi: ikinci görüş GPS,
  INA228 akımı ya da tampon; **üçü de donanıma bağlı.** Bu, İz B'nin İz A'yı
  ilk kez gerçekten bloke ettiği yer.
- **Gazebo'da EKF'i patinaja karşı ölçmek** — A3d zinciri kurdu ama bu soruyu
  sormadı. Kinematik dünyada ~8 m karede EKF hatası 0.083 m'di ve patinaj
  YOKTU; aynı kareyi Gazebo'da koşup sayıyı almak artık kısa bir iş ve gerçek
  odometri hatasının en büyük kaynağına ilk dürüst bakış olur.
- **SP3 → SP5** — batarya/docking zinciri; SP1'in ölçüm katmanı hazır.

### ✅ A4 (manyetometre) yazıldı — yaw sorunu ÇÖZÜLDÜ, ölçüldü
`qmc5883l_driver` + `imu_filter_madgwick` devrede. Zincir:
```
mpu6050  ──► /imu/data_raw ─┐
                            ├──► imu_filter_madgwick ──► /imu/data (orientation VAR)
qmc5883l ──► /imu/mag ──────┘                               │
                                          ekf_global (yaw AÇIK) + navsat_transform
```
**Ölçüldü — tam tur döndürerek, her yönde (A6 düzeltmesinden SONRA):**
| truth | /imu/data (madgwick) | ekf_global | hata |
|---|---|---|---|
| +0.0° | +1.3° | +1.3° | **1.3°** |
| +76.9° | +78.5° | +78.5° | **1.6°** |
| +156.4° | +156.3° | +156.0° | **0.5°** |
| -126.8° | -128.1° | -127.7° | **0.9°** |

Artık **testle korunuyor**: `test_absolute_yaw_holds_through_a_full_turn`.

⚠️ **A4'ün ilk "kanıt" tablosu YANLIŞTI** — aynalı bir simülatörde ölçülmüştü
(A6'ya bak). Ders: elle bir kez koşulan ölçüm, kanıt değil. Testine çevir.

`ekf_local` hâlâ sapıyor (8→10°) — **kasıtlı**: mutlak yaw sadece `ekf_global`'a
veriliyor, `ekf_local`'a asla (kötü heading base_link'i savurmasın; karar 4'ün
zaten söylediği ayrım).

⚠️ **Chip HÂLÂ ALINMADI** (~100 TL). Sürücü `mpu6050_driver` deseniyle yazıldı
ve sahte I2C'ye karşı doğrulandı: kalibrasyonsuz hard iron **>15° yön hatası**
veriyor, kalibrasyonla **<0.5°**. Gerçek çipte `i2cdetect -y 1` ile 0x0D'yi gör.

### ✅ A6: GPS waypoint SORUNU ÇÖZÜLDÜ — sebep simülatörün aynalı manyetometresiydi
**Semptom:** waypoint 0 başarılı, waypoint 1'de robot hedefin 40 m ötesine sürüp
"Goal failed". Kontrolcü bozuk gibi görünüyordu. **Değildi.**

**Kök neden — tek bir eksi işareti**, `sim_node.py` ve `qmc5883l_driver/fake_bus.py`:
```python
bx = -EARTH_NORTH_T * math.sin(yaw)   # YANLIŞ — dünyanın alanını AYNALIYOR
bx =  EARTH_NORTH_T * math.sin(yaw)   # doğru: R(-yaw) @ (0,N) = (N sin, N cos)
```
Zincir: mag "ters yöne dönüyorsun" dedi → madgwick gyro'yla kavga edip **gerçek
dönüşün 0.897'sini** gördü → `ekf_global` yaw'ı **~120° şaştı** → bu `map→odom`'a
düştü → Nav2 her hedefi 120° çevirdi → robot kaçan havucu kovaladı.

**Bu bir SİM hatasıydı, robot kodunda değil.** `mag_node` yön hesaplamıyor, alanı
kalibre edip geçiriyor; gerçek çipten gerçek alan gelir. Ama A2 ve A4'ün
doğrulaması bu yalan söyleyen sim üzerinden yapıldığı için ikisi de sahte sonuç
verdi.

**Neden aylarca hayatta kaldı — asıl ders bu:**
1. **İki hata birbirini götürdü.** `fake_bus` aynalıydı, testin `heading_of`
   yardımcısı da (`atan2(-x, y)`) — çarpımları kimlik. Testler kendi içinde
   tutarlı, dünyaya göre aynalı bir evreni doğruluyordu.
2. **Her test yaw=0'da başlıyordu.** `sin(0)=0` — aynalı ve doğru alan orada
   **birebir aynı**. Hata tam olarak görünmez olduğu noktada aranıyordu.
3. **`ekf_global`'ın yaw'ına bakan test YOKTU.** `SimStack` `use_gps:=false` ile
   kalkıyordu, `Nav2Stack` `map→odom`'u kimliğe sabitliyordu → manyetometrenin
   var olma sebebinin sıfır kapsaması vardı. A4'ün "kanıtı" elle bir kez
   koşulmuş, düz süren, yaw 0'da başlayan bir ölçümdü.

**Şimdi korunuyor:** `test_absolute_yaw_holds_through_a_full_turn` — robotu
**tam tur döndürüp** her yönde madgwick + `ekf_global` yaw'ını ground truth'a
karşı ölçer. Bozuk işaretle koşulup **53.8° ile patladığı doğrulandı**. Yeni
`GlobalStack` (conftest) GPS + madgwick + ekf_global'ı gerçekten kaldırır.

**Kabul testi geçiyor** (`measure_gps_wp.py`, L rotası 6 m doğu + 4 m kuzey):
`status 4 SUCCEEDED, missed=0, gerçek hata 0.73 m` — 27 sn'de. Yani **A2 artık
gerçekten bitti**.

✅ ~~**Kalan borç:** dünya alanı modeli hâlâ iki yerde~~ — **KAPATILDI
(2026-09-08).** `qmc5883l_driver/earth_field.py` tek tanım;
`sim_node` ve `fake_bus` ikisi de oradan alıyor, hiçbiri yeniden türetmiyor.
Yön kararı: `robot_sim` → `qmc5883l_driver`, tersi değil — sürücünün
simülatörsüz de deploy edilebilmesi gerekiyor.
`test_earth_field.py` modeli **elle pusula muhakemesiyle** çiviliyor (kuzeye
bakınca alan ÖNDE, +x), tam tur süpürüyor, ve aynalı modelin **yaw=0'da doğru
modelle birebir aynı** olduğunu ayrıca test ediyor — yani tek noktada test
etmenin neden hiçbir şey kanıtlamadığını dosyanın içine yazıyor.

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
manevrası + `use_odometry_yaw: true`. ~~**Manyetometre artık İz A'nın da
blokeri.**~~ — **iptal (2026-07-17)**: A4 sürücüyü yazdı, A6 sim'deki aynayı
düzeltti; İz A artık bloke değil, GPS waypoint uçtan uca geçiyor. Manyetometre
**İz B'nin** blokeri olmayı sürdürüyor (chip alınmadı, B6'da gerçek montaj).

Tamamlananlar:
- ✅ ROS 2 workspace iskeleti, sahte ESP32'ye karşı doğrulandı
- ✅ IMU sürücüsü (`mpu6050_driver`). Kalan: gerçek chip gelince `i2cdetect -y 1`
  ile 0x68'i gör, `imu_joint` rpy'ını gerçek montaj yönüne göre ölç/yaz (B6).
- ✅ Çarpma refleksi — yönlü veto. Ultrasonik **bilinçli yapılmadı**.
- ✅ **A1: kinematik dünya + kalıcı entegrasyon testleri**

### Testleri koşmak
```bash
cd ros2 && source install/setup.bash
python3 -m pytest src/hoverboard_bridge/test -q   # 24 (~54 sn)
python3 -m pytest src/robot_sim/test -q           # 59; 4'ü Nav2, 6'sı Gazebo (~490 sn)
python3 -m pytest src/mpu6050_driver/test -q      # 16 (~0.1 sn)
python3 -m pytest src/qmc5883l_driver/test -q     # 23 (~0.1 sn)
python3 -m pytest src/ina228_driver/test -q       #  8 (~0.1 sn)
python3 -m pytest src/battery_manager/test -q     # 10 (~18 sn)
# hepsi: 140 test, ~9.3 dk

# Yığın kaldırmayan hızlı süzgeç (saniyeler). test_gazebo_config.py bunun
# içinde ve A3'ün asıl hatasını — köprünün hiçbir şeye bağlı olmaması — tam
# olarak o yakalıyor, Gazebo koşturmadan.
python3 -m pytest src/robot_sim/test -q -k "not gazebo_physics and not nav2"
```
⚠️ **Gazebo testleri `gz` yoksa temizce SKIP eder** (`-rs` ile görürsün) ve
aynı anda tek `gz` sunucusuna izin verirler; elle bir sim açık bırakırsan
fixture "a gz server is already running" diye patlar. Bu bilerek: ikinci
sunucu hata vermez, iki dünyayı tek okunmaz dünyaya karıştırır (A3 tuzak 7).
**ROS'suz da koşarlar:** protokol ve dünya birim testleri saf Python (bilinçli
tasarım); entegrasyon testleri `importorskip` ile temizce atlanır. Hook bunu
her düzenlemede zorluyor.

⚠️ **`source install/setup.bash` YAPMAZSAN yığın testleri sessizce ATLANIR** —
"10 passed, 2 skipped" görürsün ve her şey yolunda sanırsın. A6'yı yakalayan
testler tam da atlanan o testler. `-rs` ile atlananları listele.

### Gazebo (fizik) dünyası — üç komut
```bash
# 1. fizik + model + ROS-GZ köprüsü (headless; ekranı olan makinede gui:=true)
ros2 launch robot_bringup gazebo.launch.py

# 2. sahte ESP32, fizik arka ucuyla. Engel istersen buraya ekle:
#    -p obstacle_centers:="[3.0,0.0]" -p obstacle_radii:="[0.5]"
ros2 run robot_sim sim_node --ros-args \
    -p backend:=gazebo -p use_sim_time:=true -p link:=/tmp/fake_esp32_gazebo

# 3. gerçek robot yığını, SİM ZAMANINDA
ros2 launch robot_bringup robot.launch.py \
    esp32_port:=/tmp/fake_esp32_gazebo use_sim_time:=true \
    use_localization:=true use_imu:=false use_gps:=false
```
⚠️ **`use_sim_time:=true` 2. ve 3. adımda OPSİYONEL DEĞİL.** Gazebo `/clock`
yayınlar ve onun üzerinde koşar; duvar saatinde kalan bir düğüm her `dt`'yi
fiziğin ilerlettiğinden başka bir saate göre hesaplar. Hata vermez — sadece
türettiği bütün hız ve ivmeler yanlış olur.

⚠️ **Aynı anda tek `gz` sunucusu.** İkincisi hata vermez, iki dünyayı tek
okunmaz dünyaya karıştırır (A3 tuzak 7). Elle koştururken `pgrep -f "^gz sim"`.

Gerçek topic isimlerini görmek için: `gz topic -l`. Ground truth'a doğrudan
bakmak için `gz topic -e -t /model/hoverbot/ground_truth -n 1` —
`/model/hoverbot/wheel_odometry` **ground truth değildir** (A3 tuzak 1).

### Donanımsız tam yığın (kinematik dünya)
```bash
ros2 run robot_sim sim_node

# engelli dünya — survey edilmiş (haritada VAR, Nav2 dolanır) ve
# survey edilmemiş (haritada YOK, Nav2 içine sürer) ayrı parametreler.
# Değerler mutlaka ondalıklı: `3` int gider ve DOUBLE_ARRAY reddeder.
ros2 run robot_sim sim_node --ros-args \
    -p obstacle_centers:="[3.0,0.0]" -p obstacle_radii:="[0.5]" \
    -p unsurveyed_obstacle_centers:="[5.0,1.0]" -p unsurveyed_obstacle_radii:="[0.3]"

# yerel yarı (odom->base_link): sadece tekerlek + gyro
ros2 launch robot_bringup robot.launch.py esp32_port:=/tmp/fake_esp32 \
    use_localization:=true use_imu:=false

# TAM yığın — GPS waypoint'in gerçekten koştuğu yapılandırma:
# madgwick + ekf_global + navsat_transform + Nav2
ros2 launch robot_bringup robot.launch.py esp32_port:=/tmp/fake_esp32 \
    use_localization:=true use_gps:=true use_nav2:=true \
    use_imu:=false use_mag:=false use_imu_filter:=true
```
`use_imu:=false use_mag:=false`: `sim_node` `/imu/data_raw` ve `/imu/mag`'i
kendisi yayınlıyor; gerçek sürücüleri de açmak topic için kavga ettirir.
`use_imu_filter:=true` **şart** — `/imu/data`'yı (orientation'lı) üreten o.

⚠️ **Neden A1 ilk işti:** E-stop, çarpma vetosu ve EKF doğrulamaları scratchpad'de
(`/tmp`) yazılmıştı; devcontainer rebuild'i sildi, aynı test iki kez yazıldı.
Artık repodalar.

## Bilinen blokerler / bekleyen alımlar
- **ST-Link V2 klon (~150 TL)** — adım 1 için şart, henüz alınmadı
- **Magnetometer QMC5883L (~100 TL)** — listedeki en yüksek getirili harcama
- **INA228 modülü + 1.5 mΩ / ≥50 A şönt (~150-300 TL)** — SP1 yazılımı hazır;
  gerçek sensör takılınca işaret, şönt değeri ve paket kapasitesi kalibre edilecek
- Multimetre doğrulaması (UART pinout) — kart elde olunca
- Caster ×2 (125-150 mm kauçuk), mantar E-stop + kontaktör, sigorta
- ✅ ~~`ros-jazzy-nmea-navsat-driver` Jazzy apt'de olmayabilir~~ — **VAR**
  (2.0.1-3noble, apt'te doğrulandı). Bloker değil.
- `camera_ros` apt'te **var** (0.6.0-1noble) ama Dockerfile'a eklenmedi
  (`sensors.launch.py` `use_camera:=true` ile onu çağırıyor) — kamera işine
  gelince Dockerfile'a ekle, engel yok.
- Gazebo: `ros-jazzy-ros-gz` 1.0.22 apt'te **var** → A3 için engel yok.
  Makine: 4 çekirdek / 7 GB RAM (WSL2) — fizik simülasyonu koşar ama hızlı değil.
- ✅ ~~**Devcontainer rebuild bekliyor.**~~ — **yapıldı, doğrulandı (2026-07-17)**:
  `sudo -n true` ✓, `import smbus2` ✓, `imu_filter_madgwick` ✓ (/opt/ros/jazzy).
  İmajdakiler: `python3-smbus2` (IMU sürücüsü), **parolasız sudo**
  (`/etc/sudoers.d/ubuntu` — taban imajın `ubuntu` kullanıcısı sudo grubundaydı
  ama parolası kilitliydi; `visudo -c` build guard'ı var, detay
  `docs/devcontainer.md`), `ros-jazzy-imu-filter-madgwick` (A4).
