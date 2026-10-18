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
| `poi` | Point + `label` (+ `note`, `radius_m`) | yalnız `radius_m` varsa **dolu** | Etiketli nokta: kuyu, sınır taşı, direk… `goto` hedefi |

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
   3b ✅ (istasyon sahadan), 3c ✅ (tampon temas haritası). Kapsam:
   yürütme, ilerlemeyi diske yazma, şarjdan sonra devam;
   istasyon pozunu docks.yaml yerine sahadan alma; **tampon temas haritası**
   (aşağıda). Sim'de uçtan uca: alan →
   düşük batarya → dock → devam. **Kaplama yüzdesi ve sınır ihlali** ground
   truth'tan, GPS gürültüsü açık/kapalı.
4. ✅ **Web arayüzü** (4a): çizim, görev formu, canlı durum; tespit edilen
   engelleri onaylama/etiketleme.
5. **Öğret-kaydet** (tasarım 2026-10-18, aşağıda "5. aşama"): 5a web teleop ✅,
   5b sınır/rota/istasyon/etiketli nokta kaydı ✅, 5c öğrenme modu kaydedici ✅
   (taklit ile öğrenme verisi, `docs/learning.md`), 5d dışa aktarma.

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

## 3c: tampon temas haritası (yapıldı)
```
tampon basıldı ─► ESP32 ileriyi keser ─► site_map_server: temas noktası (base_link + 0.45 m ileri)
   ─► /obstacle_map'e hemen (0.25 m daire) + diske lat/lon (`detected_obstacles.json`, /toLL)
   ─► ~/detected (JSON) ─► mission_runner: satırlar tespitlerin etrafından kesilir,
                            yeni planlar da onları engel sayar
```
- **Siteye yazılmaz**: tampona köpek, top, ayak da basar; onay/etiket 4. aşamada.
- Yeniden açılışta tespitler lat/lon'dan geri çevrilip haritaya çizilir.
- Bir basış ancak bilinen bir tespitin **dairesinin içine** düşerse aynı temas
  sayılır. ⚠️ Önce 0.5 m içindeki her basış birleştiriliyordu: 0.4 m'lik kaya
  yakın kenarında tek bir 0.25 m daire olarak kaldı, robot ortasına **27 kez**
  çarptı (her yeni basış eski dairenin içinde kayboldu).
- ⚠️ Çarpan robot, tespit çizildiği an onun şişirmesinin **içinde** duruyor:
  her düz çizgi "collision ahead", `navigate_to_pose` letal alandan plan
  yapamıyor; behavior_server'ın `backup`'ı da kendi izini çarpışma sayıp 30 ms'de
  5 kez düştü. Çözüm: runner önündeki tespitten temizlenecek kadar **düz geri**
  sürer — `cmd_vel_nav` üstünden (kapı, twist_mux kilidi, deadman hâlâ geçerli),
  0.1 m/s, yalnız az önce geçtiği zeminde; firmware'in tampon vetosu geri
  hareketi zaten hiç kesmiyor.
- Durdurulan satır robotun bulunduğu yerden devam eder (satırın başından değil).
- Geri çekilme yalnızca robot tespite **değerken** (robot yarıçapı + 15 cm).
  ⚠️ Önce satırların geniş boşluğu kullanılıyordu: çimin kenarını biçen, tespite
  bir metre uzaktaki robot her bacaktan önce geri gitti, kenar turu üç kez
  üst üste düşüp görev durdu (CI, PR #16).
- Tespitlerin etrafında `robot_radius + obstacle_margin + detection_margin
  (0.3)` boşluk. Daha dar iken satır uçları costmap'in inscribed bölgesine düştü,
  RPP satırları reddetti (3 satır atlandı).
- **Ölçülen** (`test_mission_rock.py`, sessiz GPS, 4 × 3 m çimde survey edilmemiş
  0.2 m kaya, 3 koşu): **0 çarpışma** (sim tamponu temastan 3 cm önce basıyor),
  1–2 tespit (merkezden 0.17–0.55 m — temas noktası, robotun o anki ~0.35 m'ye
  varan konum hatasıyla), hiç atlama yok, **%75–80** kesim (kayasız
  %95–96: küçük bir kaya ~1 m yarıçapında çim götürüyor; gerçek boyutu bir insan
  etiketleyince — 4. aşama — geri gelir).
- Sim'de tampon artık engellerden basılabiliyor (`bumper_from_obstacles`,
  varsayılan kapalı; yalnızca ön ±60°, istasyon gövdesi hariç — gerçek tamponun
  istasyona değip değmeyeceği SP6 sorusu).

## 4. aşama: web arayüzü (`web_ui`, `web_api.py`, `web/`)
Laptoptan (ya da telefondan): **`http://<robot>:8080/`** — tarayıcı yeter, ROS
gerekmez. Uydu görüntüsü için laptopun internete çıkması lazım (Esri World
Imagery; "Harita" katmanı OpenStreetMap).
- **Saha** sekmesi: tür seç (Alan, Girilmez, Engel, İstasyon, Rota), haritada
  çiz; bir şekle tıkla → ad, yarıçap (engel), yanaşma yönü (istasyon, pusula °);
  köşeleri sürükle. **Sahayı kaydet** → robot kendi okuyucusuyla doğrular
  (bozuksa reddedilir, dosya değişmez), eski hâli `site.geojson.bak`, sonra
  `site_map_server` ve `battery_supervisor` dosyayı yeniden okur (`~/reload`,
  `~/reload_site`). Supervisor eve dönüş/yanaşma sırasında yeniden yüklemeyi
  reddeder (staging bir yere, dock başka yere gitmesin). İlk istasyon adı
  `home` — supervisor'ın yanaştığı istasyon o.
- **Görev** sekmesi: form `TASK_TYPES`'tan üretiliyor (yeni görev tipi = sayfada
  kendiliğinden yeni seçenek). Başlat / Duraklat / Devam / İptal; durum canlı.
- **Tespitler** sekmesi: tamponun buldukları; **Onayla** (ad + gerçek yarıçap) →
  sahaya kalıcı `obstacle` olarak girer ve tespitlerden çıkar — fazladan boşluk
  kalkar, kaybedilen çim geri gelir; **Sil** → unutulur.
- Canlı: robot konumu `/gps/filtered`'dan, yönü `map→base_link`'ten (pusula),
  görev durumu, supervisor, batarya; saniyede bir.
- ⚠️ **Giriş/şifre YOK.** Robotun ağındaki herkes görev başlatabilir. Robotu
  kontrol etmediğin ağlara bağlama. `use_web:=false` ile kapanır.
- Doğrulama: `test_web_api.py` (18 test, gerçek HTTP, sahte robot),
  `test_web_stack.py` (gerçek yığın: konum/yön doğru, sayfadan çizilen yasak
  bölge `/obstacle_map`'te). Tarayıcıda (headless Chromium) elle de denendi:
  çizim → kaydet → robotun kendi okuyucusu kabul etti, JS hatası yok.

## Kamera: tespit fotoğrafı ve canlı görüntü
- **Tespit fotoğrafı:** tampon basılınca `site_map_server` kameranın son
  karesini (≤ 2 sn eski) JPEG olarak tespitle birlikte saklar
  (`detected_images/` tespit dosyasının yanında). Tespitler sekmesinde onay
  kutusunun üstünde görünür — "taş mı, köpek mi" sorusunun cevabı. Fotoğraf
  hata verirse tespit yine kaydedilir (fotoğraf engeli asla kaybettirmez).
  Tespit silinince/onaylanınca fotoğraf da silinir.
- **Canlı görüntü:** sayfadaki 📷 düğmesi → `/api/camera/stream` (MJPEG). `web_ui`
  kameraya **yalnızca biri izlerken** abone olur ve ≤ 5 Hz kodlar; kutu
  kapanınca akış kesilir (unutulan sekme 10 dk sonra kendiliğinden).
- Ham `sensor_msgs/Image` okunuyor (`camera/image_raw`, rgb8/bgr8/rgba8/bgra8/
  mono8), JPEG'e burada çevriliyor — image_transport'un compressed eklentisi
  kurulu değil.
- Robotta kamera `use_camera:=true` ile açılır. ⚠️ Pi'da Ubuntu libcamera'nın
  Pi Camera V2'yi tanıdığı hâlâ doğrulanmadı.
- **Simülatörde** (`sim_camera.py`): kinematik dünyanın render'ı yok, o yüzden
  ucuz bir ışın izleyici — çim (dama), gök, engeller gri silindir; poz
  URDF'ten (tf), görüş alanı Pi Camera V2. Yalnızca **görüntü hattını**
  (fotoğraf, canlı görüntü) denemek için; hiçbir görüntü algoritması buna göre
  ayarlanamaz. `camera_rate_hz` (varsayılan 0 = kapalı), demo 5 Hz açar.
  Gazebo kamerayı destekliyor (Sensors sistemi + ros_gz_bridge), ama render
  motoru (Ogre2) ister; bu container ve CI'da GPU yok, ve kurulumumuzda Gazebo
  yalnızca fizik için. Gerçek render'lı bir Gazebo kamerası, `track_marker_node`'u
  (AprilTag) ilk kez piksel üzerinde sınamak için ayrı bir iş olabilir.

## Simülatörde elle denemek (`scripts/sim_demo.sh`)
```bash
./scripts/sim_demo.sh                 # sonra tarayıcıda http://localhost:8080/
./scripts/sim_demo.sh --quiet-gps     # NEO-6M gezinmesi olmadan
./scripts/sim_demo.sh --rock 9,0      # başlangıcın 9 m doğusunda survey edilmemiş taş
```
Simülatör + Pi'daki `robot.launch.py`'nin aynısı (Nav2, supervisor, site_map_server,
mission_runner, web_ui). Robot Yıldız Parkı'nda (`--lat/--lon` ile değişir)
doğuya bakarak başlar; sayfada yanına alan çiz → kaydet → Görev → Başlat.
Saha `~/.robot/sim_site.geojson`'da kalır. Demo bataryası 200 Ah (stok 4.4 Ah
~3.5 saatte LOW → CRITICAL → SAFE_STOP oluyordu; istasyon simüle edilmiyor).

Bu demoyla elle denerken bulunanlar (ikisi de düzeltildi):
- ⚠️ **Henüz kaydedilmemiş `site_file` açılışta iki düğümü düşürüyordu**
  (site_map_server, battery_supervisor: FileNotFoundError) → survey haritası
  yok, Nav2 her hedefi reddeder. Artık olmayan dosya = boş saha.
- ⚠️ **Yandan sıkışan robot görevi sonsuza kadar askıda bıraktı** (3.5 saat).
  Ön tampon yandan temasta basmıyor, tekerlekler dönüyor → odometri
  "ilerliyorum", GPS geri çekiyor, Nav2'nin ilerleme denetçisi bu çekişmeyi
  ilerleme sanıyor. Gerçek sıkışma tespiti hâlâ açık (A3c tuzak 3); altına ağ:
  her waypoint'e **süre bütçesi** (60 sn + 10 sn/m), aşan bacak iptal →
  başarısız sayılır (tekrar, atla, üst üste 3'te PAUSED). Aynı senaryo
  ağdan sonra: DONE, atlama yok, iki tespit, 380 sn.
- ⚠️ **Sayfa hiç açılmadı (2026-10-17): ölü yığınların DDS dosyaları.**
  Öldürülen (SIGKILL) ROS süreçleri `/dev/shm/fastrtps_*` dosyalarını geride
  bırakıyor; ~1800 tanesi birikince her yeni düğüm `rcl_node_init`'te
  **sessizce** takılıyor (Fast DDS ölü portları tek tek zaman aşımıyla
  yokluyor). Nav2 "active" dedi ama web_ui, mission_runner, site_map_server ve
  batarya düğümleri `Node.__init__`'i geçemedi → :8080 cevap vermedi, logda tek
  satır yok. Temizlikten sonra aynı yığın sayfayı 15 sn'de açtı.
  `scripts/dds_shm_clean.sh` artık `sim_demo.sh` ve `ci.sh` başında koşuyor:
  hiçbir süreç bu dosyaları kullanmıyorsa hepsini siler, kullanan varsa yalnız
  `fastdds shm clean` (canlı katılımcılarınkine dokunmaz).

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

## 5. aşama: öğret-kaydet (tasarım, 2026-10-18)
Kullanıcı istekleri: sahada robotu sayfadan sürerek **sınır, rota, istasyon ve
etiketli özel nokta** kaydetmek; ayrıca **öğrenme modu** — operatör "şu görevi
yapacağız" deyince robotun tüm durumu ve sensörleri ileride bir taklit ile
öğrenme (IL) algoritmasını eğitmek için kaydedilsin. 5c/5d'nin tasarımı
`docs/learning.md`'de; burada 5a ve 5b.

Dilimler, sırayla (her biri ayrı PR, yığınsız):
| dilim | ne | neden bu sırada |
|---|---|---|
| **5a** | Sayfadan teleop | 5b ve 5c'nin ikisi de buna dayanıyor; motor komut yolunda olduğu için önce ve tek başına |
| **5b** | İz/nokta kaydı → saha dosyası; `poi` türü | |
| **5c** | Bölüm kaydedici (MCAP) + sayfa kontrolleri | 5a'dan sonra 5b ile paralel gidebilir |
| **5d** | MCAP → eğitim formatı (laptopta, çevrimdışı) | algoritma seçilince |

### ✅ 5a: sayfadan teleop (2026-10-18)
Sayfada **Sür** sekmesi: sürüklenen top (dokunmatikte de) ya da ok tuşları /
WASD, hız sürgüsü (%10–100, tavanın oranı). Kod: `teleop.py` (kapı, ROS'suz),
`web_api.py` (`/api/teleop*`), `web_ui.py` (20 Hz yayın), `web/drive.js`.
- Tarayıcı sürüş modundayken **10 Hz** `POST /api/teleop` yollar (çubuk
  bırakılmışsa da: sıfır). `web_ui` `/cmd_vel_teleop`'a 20 Hz yayınlar; twist_mux
  önceliği 100 → Nav2'yi ezer. Batarya kilidi (öncelik 50) yalnız Nav2'yi
  maskeler, teleop'u değil (twist_mux.yaml): bitik bataryalı robot elle eve
  sürülebilir.
- **WebSocket değil, düz HTTP:** sunucu stdlib `http.server`; WebSocket ya yeni
  bağımlılık ya elle çerçeveleme ister. Güvenliği taşıyan şey taşıma değil,
  sunucu tarafındaki **ölü adam** — iki yolda da aynı.
- **Ölü adam (sunucuda, saat `web_ui`'da):**
  | son komuttan beri | `web_ui` ne yayınlar |
  |---|---|
  | < `deadman_s` (0.3 s) | komut (kırpılmış) |
  | 0.3 s – `lease_s` (2 s) | **sıfır** — bağlantı hıçkırığında robot hemen durur, köprünün 0.5 s'sine kalmaz |
  | > `lease_s` | **hiçbir şey**, kira düşer → mux Nav2'ye döner |
  Altındaki zincir aynen duruyor: mux zaman aşımı 0.5 s, köprü `cmd_timeout`
  0.5 s, ESP32 watchdog 200 ms, E-stop, tampon vetosu.
- **Hız tavanı sunucuda:** `max_linear` 0.6 m/s, `max_angular` 1.0 rad/s
  (köprünün 1.0 / 1.5'inin altında). Tarayıcının yolladığı sayı ±1'lik oran; m/s
  değil — sayfa ne yollarsa yollasın tavanı aşamaz. NaN/sonsuz/yanlış tip → 400.
- **Kira (tek sürücü):** giriş yok, iki sekme aynı anda süremesin. `POST
  /api/teleop/acquire` bir jeton verir; komut jetonsuz/yanlış jetonla → 409.
  Kira `lease_s` boyunca komut gelmezse kendiliğinden düşer (kapanan sekme
  robotu kilitlemez). Başkası sürerken alma isteği reddedilir.
- **Görev sırasında sürmek serbest** (öğrenme modunun "müdahale" verisi bu,
  learning.md). Mux zaten teleop'u öne alıyor; 20 sn'den uzun tutarsan Nav2'nin
  ilerleme denetçisi hedefi düşürür, mission_runner bunu başarısız bacak sayar.
  Elle uzun süreceksen önce görevi **duraklat**.
- `web_ui` `teleop/state` (String JSON: kira var mı, aktif mi, komut) yayınlar —
  bölüm kaydında "o an kim sürüyordu" sorusunun cevabı.
- Sayfa sekme gizlenince sürüşü bırakır: gizli sekmenin zamanlayıcısı ~1 Hz'e
  düşer, robot komut ile ölü adam arasında sarsılırdı.
- **Ölçülen** (`test_web_stack.py`, gerçek mux + köprü + kinematik dünya, 3
  koşu): yarım çubuk 3 sn → `/cmd_vel` tam 0.30 m/s, robot 0.86–0.88 m düz;
  sayfa susunca ilk sıfır **0.32 s**'de (köprünün 0.5 s'si değil, bizim 0.3 s),
  robot **0.12 m**'de duruyor; kira bitince `/cmd_vel_teleop` susuyor. Tam
  hızda (0.6 m/s) kopan Wi-Fi'ın bedeli bunun yaklaşık iki katı.
  Headless Chromium'da da sürüldü (tuş, top, ikinci sekme reddi, JS hatası yok).
- Testler: `test_teleop.py` (kapı, adım adım saat), `test_web_api.py` (HTTP).

### ✅ 5b: sınır, rota, istasyon, etiketli nokta (2026-10-18)
"Sür" sekmesinin altında **Öğret**: iz (alan / girilmez / rota) başlat → sür →
"Köşe bırak" (isteğe bağlı) → Bitir; nokta (istasyon / etiketli nokta / engel)
→ dur → Ölç (N sn). Çıkan şekil sahaya **aday** olarak eklenir, adı verilir,
düzeltilir, mevcut "Sahayı kaydet" yolundan geçer (robotun okuyucusu doğrular;
karar 3). Kod: `teach.py` (ROS'suz), `web_api.py` `/api/teach/*`, `web_ui.py`
(besleme + `/toLL`), `web/drive.js`.

- ⚠️⚠️ **İz, GPS örnekleri DEĞİL — tasarım ölçümle değişti.** İlk sürüm
  `/gps/filtered` örneklerini kaydediyordu. Sessiz GPS'te kusursuzdu (0.055 m),
  ama sim'in NEO-6M gezinmesiyle 6 × 4 m'lik kare **kendini kesti**, noktalar
  1–2 m zikzak çizdi, başı ile sonu 3 m ayrıydı — kullanılamaz.
  `/gps/filtered` = map→odom (GPS, zıplıyor) ∘ odom→base_link (pürüzsüz).
  Artık **şekil `odom`'da** kaydediliyor (tekerlek + IMU), **yer** kayıt boyunca
  **ortalanan map→odom** ile veriliyor, lat/lon'a `/toLL` çeviriyor (karar 4).
  Ortalama gürültüyü alır, kaymayı almaz (karar 1): alan **tek parça** olarak
  birkaç metre kayık olabilir, buruşmaz. Sayfa kayıt boyunca GPS'in ne kadar
  gezindiğini (`anchor_wander_m`) gösterir.
- Köşe artık ortalama almıyor, anında sabitleniyor (odom'da titreşim yok);
  sadeleştirme (Douglas–Peucker, 0.3 m) sabit köşeyi silemez. İz robotun
  durduğu yerde biter, son aralıklı örnekte değil.
- Reddedilen iz (kendini kesen, < 1 m² alan, < 3 köşe) kayda devam eder.
- **Noktalar** `/gps/filtered` ortalaması (durağan robotta map→odom zaten tek).
  İstasyon yönü `map→base_link`'ten pusulaya (`compass_from_yaw`, tam tur
  test edildi); 359.96° yuvarlanınca 360 olup saha okuyucusunca
  reddediliyordu — `% 360`.
- **`poi`:** etiket serbest metin + öneri listesi (sınır taşı, kuyu, direk,
  musluk, vana, kapı, ağaç), not; `radius_m` verilirse `site.py` onu
  `circle_obstacles`'a da koyar → harita ve kaplama planlayıcısı değişmeden
  dolanır. `goto` formunda hedef listesi: noktalar + istasyonlar (ya da haritaya
  tıkla).
- ⚠️ **Kilitlenme bulundu:** `finish()` `/toLL`'u Teacher kilidini tutarken
  çağırıyordu; servis cevabını teslim edecek yürütücü iş parçacığı aynı kilide
  takılıyordu → her bitiş zaman aşımıyla "navsat cannot convert". Çeviri
  kilidin dışında; regresyon testi var.
- Sayfa koordinatları 8 ondalıkla yazıyor (Leaflet varsayılanı 6 ≈ 10 cm
  yuvarlıyordu — çizilenler dahil).

**Ölçülen** (`test_web_stack.py`, gerçek yığın, ground truth'a karşı robot
6 × 4 m kare sürüldü, 2'şer koşu):
| GPS | yola uzaklık | **şekil** (merkezler hizalı) | kayma | istasyon konum / yön |
|---|---|---|---|---|
| sessiz | 0.12 m | 0.10 m | 0.03 m | 0.01 m / 0.8° |
| NEO-6M | 1.4–2.7 m | **0.11 m** | 1.4–2.7 m | 0.9–1.3 m / 0.7–0.8° |
Alan sürülenden ~%3 küçük (23.1 vs 23.8 m²; köşe kesme + sadeleştirme).
Pusula, yerinde tam turda 45°'lik adımlarla, her adımda durup ground truth'a
karşı süpürüldü: en kötü **1.65°**. (İlk sürüm dönerken ölçüyordu; yüklü
makinede 11° okudu — hata değil, tf + HTTP gecikmesiydi.) Tarayıcıda da uçtan uca: iz → aday rota, nokta → etiket
"kuyu" + yarıçap → kaydet → robot kabul etti, `goto` listesinde çıktı.

## Bilinen açıklar
- ✅ ~~İstasyon iki yerde~~ — **3b'de kapandı** (aşağıda).
- Survey edilmemiş engel hâlâ görünmez (menzil sensörü yok, karar 8).
