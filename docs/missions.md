# Görevler: saha, görev tipleri, planlama, yürütme

> Durum: **tasarım + 1. aşama** (2026-10-16). Bu dosya görev katmanının tek
> kaynağıdır; handoff yalnızca buraya bağlanır.

## Ne istiyoruz
- Laptoptan **haritada bir alan seçip** ona bir görev vermek: "bu alanı biç",
  "bu alanda devriye gez", "bu rotayı sür", "şuraya git".
- Alanları ve rotaları **iki yoldan** tanımlamak: masada uydu haritası üstüne
  **çizerek**, ya da sahada robotu sürüp **öğret-kaydet** ile.
- Görev tipleri **çoğalacak ve değişecek** — yeni tip eklemek ne yürütücüye ne
  arayüze dokunmayı gerektirmemeli.
- Batarya düşünce görev kesilir (SP5, `battery_supervisor`), şarjdan sonra
  **kaldığı yerden** devam eder.

## Alınmış kararlar
1. **RTK yok, NEO-6M ile yaşanacak** (2026-10-16, kullanıcı kararı). Bedeli
   açıkça: GPS hatası 2.5–5 m ve dakikalar içinde geziniyor. Kaplama görevinde
   bitişik şeritler bazen üst üste biner, bazen arada biçilmemiş bant kalır;
   robot çizilen alanın sınırını GPS hatası kadar taşabilir. Kaplama bu yüzden
   **"en iyi gayret"** bir görevdir ve kalitesi sim'de ground truth'a karşı
   **ölçülür** (kaplama yüzdesi, sınır ihlali), varsayılmaz.
2. **Kesim aparatı misinalı biçme kafası** (bıçak değil). Kesim genişliği şasiye
   göre değişecek → `cut_width` **parametre**; şerit aralığı
   `cut_width × (1 − overlap)`. GPS hatası yüzünden `overlap` varsayılanı
   cömert tutulacak (sayı 3. aşamada ölçülerek seçilecek).
   ⚠️ Misina motoru **ikinci tehlikeli aktüatör**: tekerlek motorları gibi
   kendi güvenlik zinciri olmalı (E-stop keser, tampon/devrilmede durur, yalnızca
   görev sırasında döner). Ayrı bir iş, bu katmanın kapsamı dışında.
3. **Her şey tek bir saha dosyasında, WGS84 lat/lon olarak** (GeoJSON).
   Çizilen de, kaydedilen de, elle yazılan da aynı dosyaya gider.
4. ⚠️ **lat/lon → `map` dönüşümünün TEK yolu `navsat_transform`'un
   `/fromLLArray` servisi.** Robotun kendi GPS'ini `map`'e çeviren dönüşüm bu;
   saha geometrisi başka bir formülle (düz-dünya, kendi UTM'imiz) çevrilirse iki
   model olur — A6'nın ta kendisi. Ölçek: İstanbul'da UTM grid yakınsaması
   ~1.3°, 50 m'de ~1.1 m; şerit genişliğinden büyük.
5. ⚠️ **`/fromLL` dönüşüm hazır değilken HATA VERMİYOR.** robot_localization
   3.8.3'te servis geri çağrısı `false` döndürüyor ama rclcpp bunu hata olarak
   iletmiyor: `/fromLL` her koordinat için **(0, 0, 0)**, `/fromLLArray` **boş
   dizi** cevaplıyor (kaynak: `navsat_transform.cpp`, `fromLL()` →
   `throw "Invalid transform."` → yakalanıp `return false`). Bu yüzden
   `/fromLLArray` kullanılıyor ve **cevap uzunluğu istekle eşleşmezse hazır
   değil** sayılıyor. Tek noktalık `/fromLL` hiçbir yerde kullanılmaz.
6. **Survey haritası her zaman yayınlanır**, saha dosyası yoksa ya da boşsa da
   (A3c tuzak 1: harita gelmezse costmap hazır olmaz, her hedef abort).
   Saha dosyasında geometri varsa harita **yalnızca dönüşüm hazır olunca**
   yayınlanır — öncesinde `map` frame'i zaten yok (map→odom'u ekf_global,
   ekf_global'ın GPS girdisini navsat_transform üretiyor), yani "boş harita
   aralığı" diye bir pencere de yok.

## Saha dosyası (`site.geojson`)
GeoJSON `FeatureCollection`. ⚠️ GeoJSON koordinat sırası **[boylam, enlem]**
(lon, lat) — tersi değil. Leaflet/QGIS doğru yazar; elle yazarken klasik tuzak.

Her `Feature`'ın `properties.kind`'ı ne olduğunu söyler:

| `kind` | Geometri | Haritada | Ne için |
|---|---|---|---|
| `area` | Polygon (delikli olabilir) | — (serbest) | Görevin alanı: kaplama, devriye |
| `keepout` | Polygon | **dolu** | Girilmez bölge: çiçeklik, havuz, yol |
| `obstacle` | Point + `radius_m`, ya da Polygon | **dolu** | Survey edilmiş engel: ağaç, kaya |
| `station` | Point + `heading_deg` | istasyon gövdesi **dolu** | Şarj istasyonu |
| `route` | LineString | — | Kaydedilmiş/çizilmiş waypoint dizisi |

- `name` her feature'da zorunlu ve kendi `kind`'ı içinde eşsiz; görevler alanlara
  ve rotalara adla bağlanır.
- `station`: nokta **robotun yanaşmış hâldeki** `base_link` konumu (docks.yaml ile
  aynı anlam), `heading_deg` yanaşmış robotun baktığı **pusula yönü**
  (0 = kuzey, saat yönü). Öğret-kaydet'te robot istasyondayken kaydedilir.
  Gövde haritaya `station_body_offset` ileride, `station_body_radius`
  yarıçapında çizilir (sim'in `DockStation.stop_obstacle`'ıyla aynı geometri).
- Doğrulama gevşek değil: bilinmeyen `kind`, eksik `name`, kapanmamış halka,
  ≤ 0 yarıçap, aralık dışı enlem/boylam, birbirinden 5 km'den uzak geometri
  (büyük ihtimalle lat/lon yer değiştirmiş) → dosya **reddedilir**, yarım okunmaz.

## Mimari
```
Laptop tarayıcı (uydu haritası)        Pi
 ├ alan / yasak bölge / istasyon çiz ─► site.geojson
 ├ görev: kapla / devriye / rota / git ► planner (ROS'suz)       ─► waypoint'ler (map)
 ├ öğret-kaydet (teleop + "kaydet") ──► mission_runner ─► Nav2
 └ canlı konum, ilerleme, batarya ◄───   ├ battery_supervisor/state dinler: kesinti → yerini sakla → şarjdan sonra devam
                                         └ site_map_server: keepout + obstacle + station → /obstacle_map
```

| Tip | Ne yapar | Parametreler |
|---|---|---|
| `coverage` | Şerit şerit tarama (boustrophedon); keepout/obstacle delik sayılır | `cut_width`, `overlap`, `angle_deg`, `boundary_margin` |
| `patrol` | Çevre turu ya da alan içinde rastgele noktalar, tekrar | `mode`, `laps` (0 = süresiz), `dwell_s` |
| `route` | Kaydedilmiş/çizilmiş waypoint dizisi | `repeat` |
| `goto` | Tek nokta | — |

- Görev tipi = "alan + parametreler → `map`'te yol" üreten saf bir fonksiyon.
- Kaplama planlayıcısı kendimiz (boustrophedon): robot diferansiyel sürüşlü,
  yerinde dönebiliyor, yani Fields2Cover'ın asıl gücü (dönüş eğrileri) burada
  işe yaramıyor. `opennav_coverage` Jazzy apt'de yok; Fields2Cover 2.1.1 var,
  arayüz sonradan takılabilecek şekilde kurulur.
- Arayüz: Pi'ın sunduğu web sayfası (Leaflet + çizim araçları). Laptopta ROS/DDS
  gerekmez, telefonda da açılır.

## Aşamalar
1. ✅ **Saha dosyası + `site_map_server` → `/obstacle_map`** (bu dal). Sahada
   Nav2'yi sessizce öldürecek açığı (gerçek robotta haritayı yayınlayan kimse
   yoktu) kapatır.
2. ✅ **Planlayıcı**: coverage, patrol, route, goto. ROS'suz, elle hesaplanmış
   testler (aşağıda).
3. **mission_runner** — 3a ✅ (yürütme, disk, şarjdan sonra devam, ölçüm);
   kalan 3b (istasyon sahadan) ve 3c (tampon temas haritası). Kapsam:
   yürütme, ilerlemeyi diske yazma, şarjdan sonra devam;
   istasyon pozunu docks.yaml yerine sahadan alma; **tampon temas haritası**
   (aşağıda). Sim'de uçtan uca: alan →
   düşük batarya → dock → devam. **Kaplama yüzdesi ve sınır ihlali** ground
   truth'tan, GPS gürültüsü açık/kapalı.
4. **Web arayüzü**: çizim, görev formu, canlı durum; tespit edilen engelleri
   onaylama/etiketleme.
5. **Öğret-kaydet**: web teleop + sınır/waypoint/istasyon kaydı.

## 1. aşama: ne yapıldı, nasıl kullanılır
- **`robot_missions` paketi** (yeni): `site.py` (okuyucu + doğrulama),
  `occupancy.py` (şekil → hücre, sim'in `obstacle_map.py`'siyle aynı kurallar),
  `site_map.py` (hangi noktalar çevrilecek + şekilleri geri kurma, ROS'suz),
  `site_map_server.py` (düğüm).
- **`robot.launch.py`**: `site_map_server` varsayılan olarak koşar
  (`use_site_map:=true`), `site_file:=` boşsa tamamen serbest harita yayınlar.
  Sahada `robot.service`'in `ExecStart`'ına `site_file:=/home/enes/site.geojson`
  eklenir. ⚠️ `/obstacle_map`'in **tek** yayıncısı olmalı: sim'de `sim_node`
  yayınlıyor, o yüzden testler `use_site_map:=false` geçiyor (`conftest.py`
  `SITE_ARGS`); sahadan gelen bir site dosyasıyla sim koşmak için `sim_node`'a
  `publish_obstacle_map:=false`.
- `/diagnostics`'te `site_map_server: survey map`: WARN = navsat_transform
  bekleniyor, OK = harita yayınlandı (boyut, dolu hücre sayısı), ERROR =
  istasyon docks.yaml'la uyuşmuyor.
- **Ölçüldü** (`test_site_map_stack.py`, gerçek navsat_transform, GPS gezinmesi
  kapalı): pusulanın 8 yönünde 12 m'deki ağaçlar `map`'te sim_world
  konumlarından en çok **0.065 / 0.035 / 0.074 / 0.061 m** (4 koşu) sapıyor.
  `test_site_nav2.py`: lat/lon'la çizilmiş 6 m'lik yasak şerit → Nav2 etrafından
  dolanıyor (max |y| **4.09 m**, gövde merkezinin şeride en yakın geçişi
  **0.94 m**, yarıçap 0.4); karşıt test (şerit yok) düz çizgi.
- ⚠️ **Yolda bulunan sim hatası:** `sim_node`'un sahte GPS'i **base_link'in**
  konumunu bildiriyordu, antenin değil (direk base_link'in 0.15 m arkasında).
  navsat_transform olmayan bir ofseti çıkarıyordu → `map` ilk fix'teki yöne
  doğru 0.15 m kaymış, sonraki her fix robotla dönen 0.15 m hatalı. Aynı
  süpürme hatayla **0.105–0.182 m** okuyordu; eşik (0.12 m) bunun altında.
  Düzeltme: GPS anteni tf'ten okuyor (kamera montajı gibi).

## 2. aşama: planlayıcı (`planner.py`, `mission.py`)
Görev tanımı (JSON; web formu 4. aşamada bunu üretecek):
```json
{"type": "coverage", "area": "bahce", "params": {"overlap": 0.3, "angle_deg": 90}}
{"type": "patrol",   "area": "bahce", "params": {"mode": "random", "laps": 0}}
{"type": "route",    "route": "tur",  "params": {"repeat": 2, "back_and_forth": true}}
{"type": "goto",     "target": [29.0001, 41.0002], "params": {"heading_deg": 90}}
```
- **Görev tipleri kayıtlı** (`TASK_TYPES`): her tip neyin üstünde koştuğunu
  (alan / rota / nokta), parametrelerini (varsayılan, tip, sınır) ve planlayıcısını
  bildirir. Yeni tip = tek kayıt. ⚠️ **Bilinmeyen parametre hata:**
  `"overlab": 0.3` sessizce yutulsa görev varsayılanla koşar ve istendiği gibi
  koşmuş görünürdü.
- **Makine görevden ayrı:** `cut_width`, `robot_radius`, `obstacle_margin`
  `RobotSpec`'te (⚠️ `cut_width` 0.30 yer tutucu, B4'te ölçülecek). Görev bunları
  değiştiremez; şerit aralığı `cut_width × (1 − overlap)`.
- **Serbest bölge tek yerde:** alan `boundary_margin` kadar (varsayılan
  `cut_width/2`; köşeler kare kalsın diye mitre) daraltılır; keepout, engel ve
  istasyon gövdesi `robot_radius + obstacle_margin` büyütülüp çıkarılır. Her tip
  bunu kullanır — kendi "serbest" fikrini çizen bir planlayıcı Nav2'ye engelin
  şişirmesi içinde hedef verirdi.
- **Kaplama:** satırlar bölgenin minimum döndürülmüş dikdörtgeninin uzun
  kenarı boyunca (en az dönüş) ya da verilen pusula açısında; satırlar ortalı
  (iki kenar boşluğu eşit). Engelin böldüğü satırlar parçalanır, parçalar en
  yakın uçtan zincirlenir (dikdörtgende tam yılankavi). Sonda isteğe bağlı
  **kenar turu** (dış sınır + her deliğin çevresi).
- **Devriye:** bölgenin çevresi (saat yönü tersine, robota en yakın köşeden,
  kapalı) ya da tohumlu rastgele noktalar (ardışık noktalar `min_separation`
  ayrı). **Rota:** gidiş ya da gidiş-dönüş. `repeat`/`laps` = 0 → durdurulana kadar.
- Her waypoint `work` bayrağı taşır: varan bacak iş mi (biçme), geçiş mi.
  3. aşamada misina buna göre açılır, ilerleme iş bacaklarıyla sayılır.
- **Ölçülen (elle hesaplanan şekiller, ızgara örneklemesiyle):** 20 × 6 m alan
  12 farklı eğimde: merkez çizgisi bölgesinin %100'ü süpürülüyor. Üçgen çimde
  kenar turu kaplamayı **%95.7 → %99.9**'a çıkarıyor. Ağaçlı 10 × 10 m çim:
  %97.4 → %98.8 (kalan, ağacın etrafında robotun giremediği halka).
- ⚠️ **Yolda bulunan:** kenar turu için halkalar sadeleştiriliyor (her köşe bir
  Nav2 hedefi olur). Sadeleştirme deliğin halkasında kirişleri **engele doğru**
  kesiyor: 0.1 m toleransla robotun gövdesi ağaca 2 cm'e kadar yaklaşıyordu.
  Tolerans 0.02 m (`obstacle_margin` 0.1'in çok içinde); test bunu tutuyor.
- ⚠️ Kaplama ölçümünde shapely'nin birleşimi kenar paylaşan şeritleri float
  gürültüsüyle düşürüyordu (tamamını kaplayan planda 112.3 m²'nin 82.7'si).
  Ölçüm artık ızgara örneklemesi.

## 3a: `mission_runner` (yürütme, şarj dönüşü, ölçüm)
```
~/start (StartMission: görev JSON)   ~/pause ~/resume ~/abort (Trigger)   ~/status (JSON, latched)
battery_supervisor/state ──► NORMAL / RETURNING / ... / DOCKED_IDLE / SAFE_STOP
battery_supervisor/resume ◄── robot dolunca bir kez (undock)
navigate_to_pose ◄── devriye, rota, nokta, uzun geçişler        follow_path ◄── kaplama satırları (odom'da)
```
- **Kararlar ROS'suz** `runner_policy.py`'de (olay olay test), düğüm yapıştırıcı.
- ⚠️ **İptal edilen hedef asla kendiliğinden yeniden gönderilmez.** Supervisor
  eve dönmeden önce tüm görev hedeflerini iptal ediyor ve kendi staging hedefi
  aynı `navigate_to_pose` sunucusuna gidiyor — yeni hedef çalışanı *preempt*
  eder. "İptal → tekrar gönder" diyen bir runner şarja gidişi kaçırırdı.
  Supervisor'ın açıklamadığı bir iptal (3 sn) görevi PAUSED'a alır.
- Hedef yalnızca supervisor'ın son sözü NORMAL iken gönderilir. Robot dolunca
  (DOCKED_IDLE, CHARGING'de değil) runner supervisor'dan **bir kez** undock
  ister; NORMAL'e dönünce kesilen waypoint'ten devam eder.
- Ulaşılamayan waypoint 1 kez tekrar, sonra **atlanır** ve kaydedilir. ⚠️ Ama
  **üst üste 3 atlama görevi durdurur**: bu bir waypoint sorunu değil, Nav2
  sürmüyor demek. Gerçekten görüldü: Nav2 sunucularını henüz keşfetmemiş bir
  runner 23 waypoint'lik çimi **1 saniyede** "bitirdi" (hepsi atlandı, DONE).
  Artık sunucu 15 sn beklenir, sonra sayılır.
- **Disk:** her adımda `~/.robot/mission_state.json` (atomik yazım): görev,
  index, tur, atlananlar, turun planlandığı başlangıç (lat/lon, `/toLL` —
  `map` yeniden açılışta değişir, plan başlangıca bağlı). Yeniden açılışta
  görev **PAUSED** gelir — kimseye söylemeden kendiliğinden sürmeye başlayan bir
  robot kimsenin kazara alması gereken bir varsayılan değil. `~/resume` aynı
  başlangıçtan yeniden planlar ve kaydedilen index'ten devam eder.
- ⚠️ `/toLL` de dönüşüm hazır değilken (0, 0) döndürüyor — aynı tuzak.
- ⚠️ **tf hazır değil ≠ waypoint başarısız.** Yeni açılmış bir runner'ın tf
  tamponu boş: her hedef "frame does not exist" ile anında düşüyor, üç
  waypoint 20 ms'de atlanıp görev hiç kıpırdamadan PAUSED oluyordu (mission
  testlerinde iki kez, log yakalanana kadar sebebi bilinmedi). Artık tf de
  sunucular gibi 15 sn bekleniyor.
- **Biçme bayrağı** (`~/status` `work`) yalnızca bir satırın kendi
  `follow_path`'i koşarken açık — satıra gidişte (yaklaşma, birleşme) kapalı.
  Uzaktan satıra (dock dönüşü, atlanmış geçiş) önce `navigate_to_pose` (engel
  dolanır), sonra birleşme çizgisi. Düz çizgiyle dönüşte robot istasyondan
  çime 1.94 m çim dışında "biçiyordu" ve istasyon gövdesinin içinden geçerdi.

### Kaplama satırları neden `navigate_to_pose` ile sürülmüyor (ölçüldü)
4 × 3 m çim, 0.5 m kesim, ground truth (`test_mission_coverage.py`):

| tasarım | sessiz GPS | NEO-6M |
|---|---|---|
| `navigate_to_pose`, genel profil (1.0 m tolerans) | **%63.4**, 0.71 m dışarı | — |
| `navigate_to_pose`, 0.15 m tolerans | %90.9–92.7, 0.00 m | ❌ bir satır sonunu **15 dk** kovaladı |
| `follow_path` satırları, tüm çime tek `map→odom` | %90.0, 0.36 m (satırlar ~15° eğik) | %56–58 |
| `follow_path`, **chain** (varsayılan) | **%95.0–96.2, 0.00 m** | %18.6 / %69.5 |
| `follow_path`, leg (her satır yeni `map→odom`) | %97.4, 0.77 m (birleşme çizgisinden önce) | %44.9–47.8, 3.6–4.7 m dışarı |

- **Genel profil**: 1 m tolerans her satır sonundan bir metre önce "varıyor",
  0.4 m'lik satır geçişi ise robot kıpırdamadan toleransın içinde.
- **Sıkı tolerans `map`'te**: bt_navigator hedefi `map`'te sabitliyor ve GPS
  gezinmesi `map`'i robotun altında 0.15 m'den fazla oynatıyor → hedef hep
  kayıyor.
- Bu yüzden **satır = controller_server'a doğrudan `follow_path`**: satırın
  başından sonuna düz bir çizgi, gönderildiği an `odom`'a çevrilir
  (FollowRow + precise_goal_checker, nav2.yaml). `odom` sıçramaz; GPS
  düzeltmesi iki satır arasına düşer, satırın içine değil. Satır başına uzak
  geçişler `navigate_to_pose` ile (engel dolanır), ardından her zaman hassas
  bir **birleşme** çizgisi — gevşek hedefin bıraktığı yerden değil, satırın
  çizildiği yerden başlansın diye (birleşme yokken 0.77 m dışarı kesiyordu).
- **chain:** her waypoint = bir öncekinin `odom` hedefi + plandaki adım,
  `map→odom`'un **o anki yaw'ıyla** döndürülmüş. Desen kendi geometrisini
  korur (satırlar tam bir aralık ayrı), sadece yön `map`'ten (manyetometre)
  gelir — `odom`'un yalnız-jiro yaw'ı jiro bias'ıyla kayıyor (tek snapshot
  200 sn'de satırları ~15° eğdi). GPS konumu her (yeniden) başlangıçta **bir
  kez** girer.
- ⚠️ **NEO-6M'in bedeli (karar 1), açıkça:** chain'de satırlar birbirine
  doğru, ama bütün desen görev başladığında GPS `map`'i nereye koyduysa oraya
  düşüyor; 4 × 3 m'lik bir çimde o kayma çimin çoğu (%18.6 ile %69.5 arası,
  şansa bağlı). leg daha istikrarlı ama hiçbir zaman iyi değil ve metrelerce
  dışarı kesiyor. Büyük alanlarda kayma sadece kenar şeritlerini götürür —
  ama hassas kaplama bu GPS'le mümkün değil; büyük alan ölçümü yapılmadı.
- `supervisor` artık `follow_path`'i de iptal ediyor (`MISSION_ACTIONS`):
  doğrudan controller'a giden bir satır eve dönüşte biçilmeye devam ederdi.
- ⚠️ İki goal checker olunca **boş `goal_checker_id` FollowPath'i düşürüyor**
  (controller_server.cpp); stok BT'ler boş bırakıyor. `robot_bringup/
  behavior_trees/` stok ağaçların kopyaları + checker adı, bt_navigator
  varsayılanları onlara bakıyor. `test_nav2_profiles.py` kuralları tutuyor.

## 3b: ev = sahanın istasyonu
- Sahada `kind: station` adı supervisor'ın `dock_id`'si (varsayılan `home`)
  olan bir istasyon varsa, **ev oradan gelir**: lat/lon → `/fromLLArray` (survey
  haritasındaki istasyon gövdesiyle aynı dönüşüm), `docks.yaml`'ın `map` pozu
  **yok sayılır**. Sebep: `map`'in orijini ilk GPS fix'i, yani docks.yaml'daki
  sabit poz ancak robot istasyonda — o yöne bakarak — açılırsa doğru.
- Dönüşüm hazır olana kadar ev bilinmez; o arada LOW gelirse staging adımı
  "home is not known yet" ile düşer → SAFE_STOP (zaten `map` yokken robot
  eve gidemezdi). `/diagnostics`'te `home: site | docks.yaml`.
- `DockRobot` artık **pozla** (`use_dock_id: false`, `dock_pose`) — docking
  server'ın kendi veritabanı çalışma anında dönüştürülen pozu bilemez.
  docks.yaml yalnızca dock tipi ve sahasız çalışma için kalıyor.
- `site_map_server` artık iki pozu karşılaştırmıyor; sahada istasyon yokken
  docks.yaml'da varsa **WARN**: o istasyon survey haritasında çizili değil
  (çizilmemiş istasyona robot çarpıyordu — `test_docking_sweep`).
- **Kabul testi** (`test_mission_dock.py`, sessiz GPS): kaplama görevi → batarya
  LOW → supervisor satırı iptal eder, **sahanın istasyonuna** gider (docks.yaml
  hâlâ orijinde, robot orada başlamadı) → yanaşır, şarj, FULL → mission_runner
  undock ister → kaldığı yerden devam → DONE.
- ⚠️ **Yolda bulunan (gerçek robotu da etkilerdi):** SoC tahmincisi FULL'ü
  yalnızca sayılan SoC < 1.0 iken bildiriyordu. Coulomb sayımı şarj
  incelmeden 1.0'a dayanmışsa (kapasite düşük girilmiş, INA228 yüksek okuyor —
  `capacity_ah` hâlâ yer tutucu) FULL **hiç** gelmiyor, supervisor CHARGING'de
  sonsuza kadar bekliyor, robot istasyondan **hiç** ayrılmıyordu. İlk koşu 20 dk
  şarjda kaldı. Artık bekleme süresi ilk aşıldığında bir kez bildiriliyor.

## Robotun kendi tespit ettiği engeller (2026-10-16, kullanıcı onayı)
Bugünkü donanımda menzil sensörü yok (handoff karar 8): survey edilmemiş bir
taşı robot ancak **tamponla değince** fark ediyor, Nav2 de sıkışmayı göremiyor
(A3c tuzak 3). Kamera tek başına mesafe ölçemiyor; hazır nesne tanıma
modellerinde (COCO) "ağaç" ve "taş" sınıfı yok. Plan:

1. **Temas haritası (3. aşama, `mission_runner` ile):** tampon her tetiklendiğinde
   temas noktası (robotun `map`'teki kendi tahmini + tamponun base_link'teki
   yeri) "tespit edilen engel" olarak kaydedilir ve **canlı haritaya hemen**
   girer — aynı görevde robot aynı taşa ikinci kez gitmez. Kayıt diske yazılır,
   saha dosyasına **kendiliğinden girmez**. Aynı sinyal sıkışma tespitine de
   girdi: tampon tetikli ama tekerlek odometrisi "ilerliyorum" diyor.
2. **İnsan onayı ve etiket (4. aşama, web arayüzü):** tespit edilen noktalar
   "onay bekleyen" olarak görünür; kullanıcı "ağaç, 0.4 m" der → saha dosyasına
   kalıcı `obstacle` olarak geçer, ya da reddeder. Robot "burada bir şey var"
   der, **ne olduğunu insan söyler** — otomatik sınıflandırma hedef değil.
3. **Gerçek algılama ertelendi:** menzil sensörü kararı ilk saha denemelerinden
   sonra (çim boyu ve taş yüksekliği sahada görülünce). Kullanıcı ileride
   **derinlik kamerasını** (OAK-D Lite sınıfı) değerlendirecek. Not: 2D LiDAR'ın
   çim biçen robotta tuzağı — alçak monte edilirse uzun çimi engel sanar, yüksek
   monte edilirse alçak taşı görmez; derinlik kamerası ikisini ayırabilir.
   Hangisi gelirse gelsin Nav2'nin `obstacle_layer`'ına girer, temas haritası
   yedek olarak kalır.

⚠️ Bedel: temas haritası öğrenmek için önce **çarpmayı** gerektirir. Yavaş hız
ve misinalı aparatla kabul edilebilir; bıçakla olmazdı.

## Bilinen açıklar
- ✅ ~~İstasyon iki yerde~~ — **3b'de kapandı** (aşağıda).
- Survey edilmemiş engel hâlâ görünmez (menzil sensörü yok, karar 8).
