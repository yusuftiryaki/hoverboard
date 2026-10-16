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
2. **Planlayıcı**: coverage, patrol, route, goto. ROS'suz, elle hesaplanmış testler.
3. **mission_runner**: yürütme, ilerlemeyi diske yazma, şarjdan sonra devam;
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
- ⚠️ **İstasyon iki yerde:** opennav_docking ve supervisor hâlâ `docks.yaml`'ın
  `map` pozunu okuyor, harita ise sahadaki lat/lon'u çiziyor. `map`'in orijini
  ilk GPS fix'i (`wait_for_datum: false`) olduğu için docks.yaml'daki sabit poz
  ancak robot istasyonda açılırsa doğru — ve placeholder'ın yaw'ı 0 (doğuya
  bakan istasyon) olduğu için ancak istasyon doğuya bakıyorsa. `site_map_server`
  ikisi **1 m ya da 15°'den fazla** ayrışırsa `/diagnostics`'e ERROR basıyor;
  kalıcı çözüm 3. aşamada (poz çalışma anında sahadan).
- Survey edilmemiş engel hâlâ görünmez (menzil sensörü yok, karar 8).
