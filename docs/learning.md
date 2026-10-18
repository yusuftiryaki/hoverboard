# Öğrenme modu: taklit ile öğrenme için veri kaydı

> Durum: **5c ve 5d yapıldı** (2026-10-18) — kaydedici, sayfa, dışa aktarma;
> ölçüldü (aşağıda). Sıradaki: algoritma seçimi ve politikanın twist_mux'a
> takılması. 5a/5b `docs/missions.md` "5. aşama"da.

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

## ✅ 5d: dışa aktarma (2026-10-18)
```bash
# bölümleri laptopa al (sayfadan zip ya da rsync), sonra dev container'da:
ros2 run robot_learning episode_export ~/episodes -o ~/dataset --fps 10 --image-size 320x240
#   --include-aborted   "iptal" sonuçlu bölümleri de al (varsayılan: alma)
#   --reindex           çökmüş bölümün bag'ini önce onar (metadata.yaml yoksa)
```
Çıktı (`format: hoverbot-episodes-v1`):
| dosya | içerik |
|---|---|
| `meta.json` | fps, sütun adları (`state_names`, `action_names`, `action_sources`), aksiyon kuralı; bölüm başına görev, mod, sonuç, işaretler (**kare numarası**), teleop oranı, yazılım sürümü |
| `episodes/episode_NNNNNN.npz` | `timestamp`, `state` (N×15, eksik = NaN), `action` (N×2 etiket), `action_source` (yok/teleop/nav), `action_executed` (`/cmd_vel`), `teleop` (insan tutuyordu), `image_age` |
| `videos/episode_NNNNNN.mp4` | satır başına bir kare (o andaki en son kamera karesi), mp4v |

Kurallar (`dataset.py`, elle hesaplanmış testlerle):
- Kareler **READY'den** başlar (öncesi kısmi). ⚠️ Gazebo'da bag'in saati sim
  saati, `started_unix` duvar saati — saatler uyuşmuyorsa bag'in ilk mesajı.
- Durum: her serinin kareden önceki **son** değeri; hiç gelmemişse NaN (0 değil).
- **Aksiyon = (t, t+1/fps] aralığında verilen komut** — gözlem k'ya verilen
  cevap; o aralıkta komut yoksa hâlâ taze (0.5 s, twist_mux'ın süresi) olan
  son komut. Teleop Nav2'yi ezer (mux gibi).
- Kamera karesi: damgası kareden önceki en son kare (damga = ışığın sensöre
  geldiği an, ham karenin damgası).

Neden parquet/LeRobot/HDF5 değil: algoritma seçilmedi (karar 1) ve
hiçbiri burada yeni bağımlılık olmadan yazılamıyor (Noble apt'inde pyarrow
yok). npz + mp4 her yerde açılır; seçilen eğiticinin formatına dönüştürücü
eğitim makinesinde küçük bir adım, ve bag kaynak olarak kalır — başka
seçimlerle istendiği an yeniden dışa aktarılır.

**Ölçülen** (`test_learning_stack.py`, kaydedilen gerçek bölüm): 68 kare (~70
beklenen), `action` tam 0.30 m/s ve 0.5 rad/s, kaynak yalnız teleop/yok,
**etiket tekerlekten önce**: komutun geldiği kare ile odometrinin hızlandığı
kare arası 0–5 kare (asla negatif değil — hizalama ters olsaydı politika
geleceği "görerek" öğrenirdi), işaret doğru karede, video kare sayısı = satır
sayısı.

## 6. aşama: beceri öğrenme — önce kenar takibi (plan, 2026-10-18)
Kullanıcıyla kararlaştırıldı: "tüm görev" tek politikaya verilmiyor. **Rota
planlayıcı + Nav2'de kalır** (şeritler, eve dönüş); öğrenme **yerel
becerilere** gider: kenar dibi takibi, engel etrafı, yabani ota yönelme.
`mission_runner` beceriyi kısa süreliğine çağırır, sonra kontrolü geri alır.
İlk beceri **kenar dibi takibi**: ölçütü en net olan, ve NEO-6M'le planlayıcının
kenar turu metrelerce ıskalıyor — kamera kenarı GPS'ten bağımsız görür.
Eğitim **bulutta** (Colab / Kaggle, GPU); robot ve dev container yalnız
veri üretir, dışa aktarır, çıkarım yapar.

Algoritma: önce **basit BC** (boru hattı taban çizgisi), sonra **ACT**
(aksiyon blokları, Pi'da koşma şansı), sol/sağ ortalaması sorun olursa
**Diffusion Policy** (çok modluluk; çıkarımı pahalı). Hepsi LeRobot'ta hazır.

**Sim'in uzmanı Nav2 değil, ayrıcalıklı betik:** ground truth'u ve gerçek
sınırı bilen bir kontrolcü (politika bunları görmez; kamera + durum görür).
Faydası: sınırsız gösterim ve **otomatik DAgger** — politika sürerken uzman
her duruma etiket verir (gerçekte bunu insan yapar: "müdahale" modu).

| dilim | ne | ölçüt / kanıt |
|---|---|---|
| **6a** | Ayrıcalıklı kenar uzmanı + kenar ölçütü (ground truth) + uzman bölümü kaydı; dışa aktarmada `action_expert` sütunu | uzman kendi ölçütünde: yanal hata cm'ler |
| **6b** | Kenar **görünür**: kinematik sim kamerasında çim / çim dışı (hızlı); rastgele sahalarla toplu bölüm üretimi | kamerada kenar; N bölüm |
| **6c** | npz → LeRobot dönüştürücü + Colab/Kaggle defteri (BC, ACT) | defter uçtan uca koşar |
| **6d** | `policy_node` + `policy_gate` (twist_mux önceliği 20) + sim'de kapalı çevrim değerlendirme + otomatik DAgger | politika kendi ölçütüyle puanlanır |
| **6e** | Gazebo render'lı dünyada gerçekçilik; sonra gerçek robotta **gölge mod** (komut üretir, tekerleğe gitmez) | sim→gerçek açığı |

### ✅ 6a: ayrıcalıklı kenar uzmanı (2026-10-18)
- `robot_sim/edge_expert.py` (ROS'suz): yol = sınırın `offset_m` (0.4) içeri
  çekilmişi; pure pursuit, ama **hedef keskin köşeyi geçmiyor** → robot köşeye
  girip yerinde dönüyor. Yalnız pure pursuit köşeleri yuvarlıyordu: dikdörtgen
  kenarın %16'sı hiç ulaşılmıyordu. `label(x, y, yaw)` saf: politikanın
  ulaştığı HER duruma etiket (6d DAgger). Tur, yay uzunluğuyla işaretli
  sayılıyor (ileri-geri titreme sıfır).
- `edge_metrics`: işaretli sınır uzaklığının `offset`'ten hatası (ortalama,
  p95, max), tolerans içi oranı, dışarı çıkma oranı, **sınırın ulaşılan oranı**.
  Uzman ve politika aynı ölçütle puanlanır.
- `edge_expert` düğümü: `/ground_truth` + sahanın alanı (sim'in GPS formülünün
  tam tersiyle sim_world'e — sim'in kendi gerçeği), `expert/cmd_vel` (her
  zaman, etiket) + `drive:=true` ise `cmd_vel_teleop` (insan gibi sürer),
  `~/start`, `~/stop`, `~/status`. Profil ve dışa aktarma: `action_expert`
  sütunu (sustuğu yerde NaN).
- **Ölçülen** (`test_edge_expert_stack.py`, gerçek mux + köprü + ESP32 sim +
  kinematik dünya, 6 × 4 m çim, offset 0.4): ortalama hata **1.9 cm**, p95
  6.5 cm, max 10 cm, dışarı 0, sınırın **%94**'ü (köşelerin kalanı yuvarlak
  misinanın erişemeyeceği yer: 0.4·√2 = 0.57 m > 0.5). Kayıt dışa
  aktarıldı: 652 kare, %99'unda uzman etiketi.

### ✅ 6b: kenar kamerada + rastgele sahalarla toplu bölüm (2026-10-18)
- `sim_camera`: `lawns` verilirse çim poligonunun içi çim, dışı **toprak**;
  `palette(seed)` renkleri ve dama boyunu sahaya göre değiştiriyor (domain
  randomization; çim yeşil, toprak kahve kalıyor — testli). `sim_node`:
  `lawn_file` (sahanın alanları), `ground_seed`.
- `site_world.py`: lat/lon ↔ sim_world, sim'in GPS formülünün tam tersi —
  uzman, kamera ve üreteç aynı "sim gerçeğini" kullanıyor (`EARTH_RADIUS_M`
  artık tek yerde).
- `lawn_gen.py`: orijin etrafında 4–7 köşeli, dışbükey ya da değil, rastgele
  dönük sahalar; **uzman ideal modelde temiz tur atamıyorsa saha reddediliyor**
  (sivri köşe < 55°, dar boğaz, robot kenara < 1 m, alan 15–70 m²) — kötü
  gösterim veriye sessizce girmesin.
- `ros2 run robot_sim edge_batch --lawns 20 --out ~/.robot/sim_episodes`:
  her saha için taze sim + Pi'daki `robot.launch.py` (öğrenme modu açık),
  uzman her yöne bir tur, bölüm kaydı; notunda seed, yön, GPS, sahanın WKT'si.
  Tur ~1–2 dk gerçek zaman: 20 saha × 2 yön ≈ 1.5 saat (gece işi).
- **Ölçülen** (`test_edge_batch_stack.py`, seed 8): bölüm başarılı, 317 kare,
  **kenar karelerin 312'sinde kamerada**, uzman etiketi %100; araç arkasında
  süreç bırakmıyor.
- ⚠️ **Kamera kenar takibi için iyi yerde değil.** Yanaşma için 5° aşağı
  bakıyor: karenin üst ~%40'ı gök, misinanın çalıştığı yakın zemin alt
  şeritte. Kenar ilerideki haliyle görünüyor, yanındaki haliyle zor. İkinci,
  aşağı bakan geniş açılı kamera (donanım notu, yukarıda) burada ölçülebilir:
  aynı sim'de ikinci kamera + aynı uzman, iki politikanın ölçütü yan yana.

⚠️ Kinematik kameranın kenarı (6b) yapısal olarak doğru ama gerçekçi değil;
o veriden öğrenen politika boru hattını ve yöntemi kanıtlar, sahayı değil.
Gerçek beceri gerçek gösterimler + müdahale verisiyle gelir.

**Güvenlik (6d):** `policy_gate` = kendi hız tavanı (ilk sahada 0.3 m/s) ve
ivme sınırı, kendi ölü adamı (kamera/politika > 0.3 s eskiyse sıfır),
coğrafi çit (görev alanı dışı = dur), **yalnız `mission_runner` beceriyi
başlattığında açık** (varsayılan kapalı). twist_mux'ta 20: teleop (100) ve
batarya kilidi (50) onu ezer; altındaki köprü/ESP32/E-stop/tampon zinciri
aynen kalır.

## Sonra: politika robota nasıl takılır
Algoritma seçilince (adaylar: ACT, Diffusion Policy — ikisi de kamera +
durum → aksiyon dizisi) öğrenilen politika twist_mux'a teleop ile Nav2
arasında **ayrı bir girdi** olarak takılır — kendi hız tavanı, kendi ölü
adamı ve kendi kapısıyla; teleop onu her zaman ezer. Pi 4'te çıkarım hızı
ayrı bir ölçüm konusu (gerekirse laptopta çıkarım, Wi-Fi üzerinden — o
zaman gecikme ve kopma güvenlik zincirinin parçası olur). Bugünün işi değil.
