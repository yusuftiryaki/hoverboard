# Öğrenme modu: taklit ile öğrenme için veri kaydı

> Durum: **5c yapıldı** (2026-10-18) — kaydedici, sayfa, ölçüldü (aşağıda
> "5c: ne yapıldı"). 5d (dışa aktarma) tasarım. 5a/5b `docs/missions.md`
> "5. aşama"da.

## Ne istiyoruz
Operatör sayfada "şu görevi yapacağız, öğrenme modundasın" der, robotu sürer
(ya da otonom görevi izleyip gerektiğinde araya girer); robotun **tüm durumu ve
sensörleri** bir bölüm (episode) olarak kaydedilir. Bölümler sonra laptopa
alınıp bir taklit ile öğrenme (IL) algoritmasına verilir.

**Öğrenilecek şey (kullanıcı, 2026-10-18): tüm görev** — kenar dibinde biçme,
ağacın etrafından dolaşma, yabani ota yönelme gibi davranışlar. Yani politika
büyük ihtimalle **kamera + durum → hız komutu**. Bu yüzden kamera kaydın
merkezinde; algoritma bugün seçilmiyor, kayıt ona bağlı değil.

## Kararlar
1. **Ham kayıt rosbag2 + MCAP, eğitim formatı ayrı ve çevrimdışı.** MCAP
   container'da ve Pi'da apt'te var, ROS olmadan da okunuyor (`mcap` Python
   kütüphanesi). Bag kayıpsız ana kaynak: bugün hangi gözlemi, hangi frekansı
   isteyeceğimizi bilmiyoruz; dışa aktarma (5d) her seferinde bag'den yeniden
   üretilir. Pi'da dönüştürme yapılmaz.
2. **Kaydedici robotu asla etkilemez.** Yalnızca abone olan ayrı bir süreç
   (`ros2 bag record`, `episode_recorder` düğümü başlatır/durdurur, SIGINT ile
   düzgün kapatır). Disk eşiğin altına inerse kayıt durur — robot durmaz.
   Motor komut yolunda hiçbir şey değişmez.
3. **Aksiyon etiketi = kim, ne istedi, ne oldu — üçü ayrı.** IL verisinin en
   sık bozulduğu yer burası: "operatör ne istedi" ile "tekerleğe ne gitti"
   arasında mux, hız limitleri, tampon vetosu, batarya kilidi var.
   | topic | anlamı |
   |---|---|
   | `/cmd_vel_teleop` | operatörün niyeti (**birincil etiket**) |
   | `/cmd_vel_nav` | Nav2'nin önerisi (otonom bölümlerde etiket) |
   | `/cmd_vel` | mux'tan çıkan, köprüye giden |
   | `teleop/state` | o an kira var mı, teleop aktif mi (5a) |
   | `nav_lock`, `bumper`, `diagnostics` (E-stop) | komutu kim engelledi |
   | `odom`, `joint_states` | tekerleğin gerçekte yaptığı |
4. **Üç bölüm modu:** `demo` (operatör sürüyor), `intervention` (otonom görev
   koşuyor, operatör gerektiğinde araya giriyor — DAgger tarzı yöntemlerin en
   değerli verisi, araya girdiği anlar `teleop/state`'ten çıkar), `autonomous`
   (sadece izleniyor). Mod bölümün meta verisinde.
5. **Bağlam bölümle birlikte saklanır** (`episode.yaml`, bag dizininde). B4'te
   kalibrasyon sabitleri değişecek; hangi sabitlerle kaydedildiği bilinmeyen
   bölüm o gün çöpe döner. İçerik: görev etiketi, mod, operatörün notu, sonuç
   (başarılı / başarısız / iptal, bitişte sorulur), saha dosyasının kopyası, git
   sha, `robot_description`, köprü/EKF/kamera parametreleri, sim mi gerçek mi
   (sim'deyse dünya tohumu).
6. **Bölüm içi işaretler:** kayıt sırasında sayfadaki "işaret" düğmesi
   `learning/marker`'a etiketli bir an yazar ("ağaç başladı", "yabani ot").
   "Tüm görev" tek bir bölümde birçok beceri içeriyor; işaretler sonradan
   bölütlemeyi (segmentation) mümkün kılar.
7. **Sim de aynı kaydediciyi kullanır.** Orada uzman bedava: planlayıcı + Nav2
   istendiği kadar bölüm üretir, ground truth da kayda girer (`ground_truth`,
   `collision_truth`). Algoritma denemeleri gerçek veri beklemeden başlar.
   ⚠️ Kinematik dünyanın kamerası (`sim_camera.py`) gerçekçi değil; görüntüden
   öğrenen bir politika ancak Gazebo kamerasıyla ya da gerçek veriyle anlamlı.
8. **Kamera varsayılan açık**, gizlilik kısıtı yok (kullanıcı, 2026-10-18).

## Kaydedilen topic'ler
Profil bir yaml'da (`config/learning_topics.yaml`); yeni sensör = yeni satır.
- **Kamera:** `camera/image_raw/compressed` (JPEG). Ham görüntü kaydedilmez:
  640×480 rgb8 30 Hz ~27 MB/s. JPEG 640×480 ~50 KB × 15 Hz ≈ **0.75 MB/s ≈
  2.7 GB/saat**. Kaynağı: `compressed_image_transport` eklentisi (camera_ros
  kendiliğinden `/compressed` yayınlar) — sim kameraları (Gazebo,
  `sim_camera.py`) ham yayınlıyor, onlara `image_transport republish`.
- **Durum:** `imu/data_raw`, `imu/data`, `imu/mag`, `gps/fix`, `gps/filtered`,
  `odometry/filtered/local`, `odometry/filtered/global`, `odometry/gps`, `odom`, `joint_states`, `/tf`,
  `/tf_static`, `battery`, `bumper`, `diagnostics`.
- **Aksiyon ve bağlam:** karar 3'teki tablo + `mission_runner/status`,
  `battery_supervisor/state`, `obstacle_map` (latched), `learning/marker`.
- **Sim'de ek:** `ground_truth`, `collision_truth`.
- İleride: misina motoru komutu (karar 2'deki ikinci aktüatör) bir aksiyon
  olarak eklenecek — biçme açık/kapalı da öğrenilecek şeyin parçası.

## Depolama ve dayanıklılık
- **USB 3 SSD** (SATA SSD + UASP'li USB 3 kutu, ≥ 256 GB ≈ 90 saat kamera).
  SD kart hem yavaş hem yazmayla aşınıyor; kayıt dizini SSD'de, OS SD'de kalır.
  ⚠️ Pi 4'ün USB portları toplam ~1.2 A veriyor; SSD'nin çekişi ölçülmeli,
  gerekirse ayrı beslemeli hub. Pi powerbank'ten beslendiği için (karar 13)
  E-stop Pi'ı kapatmaz, kayıt E-stop'tan etkilenmez.
- MCAP `--storage-preset-profile zstd_fast` (parça içi sıkıştırma),
  `--max-bag-duration 60`: güç kesilirse en fazla son parça kaybolur. Bu bir
  testle kanıtlanacak (SIGKILL ortasında → önceki parçalar okunur).
- Boş alan eşiği (ör. 5 GB) altında yeni bölüm başlamaz, süren bölüm düzgün
  kapatılır; sayfada kalan süre gösterilir.

## Donanım notu (kullanıcı: "istediğimiz donanımı takabiliriz")
- Tek Pi Camera V2 (62° yatay) docking için 5° aşağı bakıyor; zemin 41 cm
  ileriden başlıyor. **Kenar dibinde biçme ve yabani ota yönelme yakın zemini
  ister** — muhtemelen ikinci, geniş açılı, aşağı bakan bir kamera gerekecek.
  Pi 4'te tek CSI var → ikincisi USB kamera. Karar ilk gerçek bölümlere bakarak
  verilmeli; kaydedici profili kamera sayısından bağımsız.
- Derinlik kamerası (missions.md, "Robotun kendi tespit ettiği engeller")
  gelirse aynı profile bir satır olarak girer.

## Sayfada (5c)
"Öğrenme" sekmesi: görev etiketi (serbest + son kullanılanlar), mod, not →
**Kaydı başlat**; kayıt sürerken kırmızı REC, süre, boyut, kalan disk,
**İşaret** düğmesi; **Bitir** → sonuç + not. Bölüm listesi: indir (zip), sil.

## ✅ 5c: ne yapıldı (2026-10-18)
- **`robot_learning` paketi:** `episode.py` (ROS'suz: bölüm dizini,
  `episode.yaml`, `ros2 bag record` sürecini başlatma/durdurma, disk koruması,
  çökme tespiti), `episode_recorder` düğümü (`~/command` =
  `robot_missions_interfaces/Episode` start/stop/mark, `~/status` latched,
  `learning/marker`), `image_compressor` (`camera/image_raw` → `camera/jpeg`,
  ham karenin damgasıyla, **yalnız biri abone olunca**, ≤ 15 Hz).
- Profil: `robot_bringup/config/learning_topics.yaml` (32 topic; `sim`
  grubunda ground truth). `robot.launch.py`: `use_learning` (varsayılan açık,
  boştayken iş yapmaz), `episodes_dir`, `episode_split_s` (60),
  `learning_sim`. Pi'da `episodes_dir` SSD'de (deployment.md adım 8).
- Sayfada **Öğrenme** sekmesi (görev, mod, not → başlat; işaret; sonuç + not →
  bitir; liste: indir zip / sil) ve her sekmede görünen **REC** rozeti.
- Bölüm dizini: `episode.yaml` (görev, mod, sonuç, işaretler, `ready_after_s`,
  yazılım sürümü: git sha + kirli mi), `bag/` (MCAP, 60 sn'de bölünmüş),
  `recorder.log`, `site.geojson` kopyası, `params/<düğüm>.yaml`
  (`ros2 param dump`: köprü, EKF'ler, navsat, twist_mux, batarya… — B4'ten
  sonra hangi sabitlerle kaydedildiği buradan okunur).

Yolda bulunanlar (hepsi testli):
- ⚠️⚠️ **Kaydedici saniyelerce kör.** `ros2 bag record` başladıktan sonra
  konulara abone olması bu makinede **3–4 sn** sürüyor; ilk denemede
  ground truth'un ilk kaydı robot 0.55 m gitmişken geldi — gösterimin başı
  yoktu. Artık düğüm ROS grafiğinden "o an yayıncısı olan her profil konusuna
  kayıt süreci abone mi" diye bakıyor; olana kadar sayfa **○ HAZIRLANIYOR —
  henüz sürme** (+ beklenen konular) diyor, sonra **● REC**. `ready_after_s`
  `episode.yaml`'da. Pi'da bu süre daha uzun olacak — ölçülmeli.
- ⚠️ **Kayıt sürecinin çıktısı boşaltılmayan bir PIPE'a gitseydi** (ilk
  taslak) rosbag2 her bölmede log yazdığı için 64 KB dolunca saatler sonra
  sessizce donardı. Çıktı `recorder.log`'a.
- Kayıt süreci düğümden bağımsız (kendi oturumu) ama sahipsiz kalmaz: düğüm
  kapanırken SIGINT (sonuç `shutdown`), düğüm öldürülürse `setpriv
  --pdeathsig INT` ile çekirdek SIGINT yolluyor. (`preexec_fn` çok iş
  parçacıklı süreçte güvenli değil.)
- `ros2 param dump` düğüm başına ~1–2 sn CLI açılışı → paralel (4), bölümden
  bağımsız iş parçacığında; bölüm o sırada biterse sonuç dosyaya sonradan
  yamanıyor.

**Ölçülen** (`test_learning_stack.py`, gerçek yığın + kinematik dünya):
- Sayfadan başlat → **hazır 3–4 sn**; hazır beklenince sürüşün tamamı bag'de
  (ilk ground truth x = 0.000).
- 10 sn'lik bölümde 23 topic: `/cmd_vel_teleop` 100 (tam 0.30 m/s ve 0.5
  rad/s), `/cmd_vel` 100, `/teleop/state` ~80, `/odom` ~300–440, kamera JPEG
  ~30 (5 Hz sim kamerası; damgalar artan, çözülebilir, varıştan < 0.5 sn eski),
  işaret 1 (hem bag'de hem `episode.yaml`'da), `/tf_static` ve
  `/robot_description` (latched) 1'er.
- **Çökme:** kayıt süreci 10 sn'de SIGKILL → bölüm `crashed`, `metadata.yaml`
  yok (beklenen); `Reindexer` 3 bölmeden **5.7 sn**'yi geri getirdi (kapanmış
  bölmeler), sonraki bölüm normal başladı. Elle: `ros2 bag reindex <bag> -s mcap`.
- Tarayıcıda uçtan uca: başlat → HAZIRLANIYOR → REC → Sür sekmesinden sür →
  işaret → bitir → listede → zip 556 KB → sil. JS hatası yok.

Bilinmeyenler / sahada ölçülecek:
- Pi 4'te hazır olma süresi, CPU yükü (kayıt + JPEG kodlama 15 Hz) ve SSD yazma
  hızı. Gerçek kamera 30 Hz ham + 15 Hz JPEG.
- `systemctl stop` SIGTERM'ü tüm cgroup'a yollar; rosbag2'nin SIGTERM'de de
  dosyalarını kapattığı doğrulanmadı (kapatmazsa `reindex`).

## 5d: dışa aktarma (sonra, laptopta)
MCAP → sabit frekansta senkron gözlem/aksiyon tablosu (LeRobot v2 benzeri:
parquet + mp4) ya da HDF5 (robomimic). Algoritma seçilince öğrenilen politika
twist_mux'a teleop ile Nav2 arasında **ayrı bir girdi** olarak takılır — kendi
hız tavanı ve kendi kapısıyla. Bugünün işi değil.
