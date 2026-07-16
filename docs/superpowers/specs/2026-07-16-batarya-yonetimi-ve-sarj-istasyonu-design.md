# Batarya yönetimi ve şarj istasyonuna dönme — tasarım

> Durum: kullanıcıyla brainstorming'de bölüm bölüm onaylandı (2026-07-16).
> Kapsam: **tam otonom docking** — robot düşük bataryada görevi iptal eder,
> istasyona döner, fiziksel olarak yanaşır ve şarj olur.

## Hedef ve kapsam

- Robot batarya durumunu gerçek SoC ile (coulomb sayma) izler.
- SoC "DÖN" eşiğinin altına inince görev **anında iptal edilir**, robot Nav2 ile
  eve döner ve istasyona kendisi yanaşıp şarj olur.
- KRİTİK eşikte (her durumda, dönüş yolunda dahil) olduğu yerde güvenli durur.
- v1'de şarj bitince robot **dock'ta bekler**; göreve otomatik dönüş kapsam dışı
  (YAGNI). Enerji-bütçeli "göreve devam edebilir miyim" hesabı da kapsam dışı.
- Gece çalışması kapsam dışı (kamera tabanlı yanaşma karanlıkta çalışmaz —
  bilinçli kabul).

## Sorgulanan alternatifler ve seçimler (gerekçeleriyle)

| Karar | Seçilen | Elenen | Gerekçe |
|---|---|---|---|
| Şarj yolu | İstasyonda eldeki 42 V/2 A şarj aleti + robotta temas pedleri | Robotta şarj aleti (ağırlık + açık 220 V riski); endüktif (bütçeyi tek başına yer) | En ucuz, en az yeni donanım; BMS dengeleme/kesmeyi zaten yapıyor |
| Yanaşma algısı | Pi Camera V2 + fiducial işaretçi (AprilTag) | IR fener (güneşte boğulur, elektronik+firmware işi); sadece mekanik huni (5 m GPS hatasında güvenilmez) | Ek maliyet ~sıfır; cm-sınıfı poz; apt'te hazır algılayıcı |
| Batarya ölçümü | Akım sensörü + coulomb sayma | Sadece filtreli voltaj (kaba); ikisi-aşamalı | Kullanıcı tercihi: gerçek SoC ve "şarj başladı" tespiti donanımla |
| Düşük batarya davranışı | Anında görev iptali + eve dönüş; KRİTİK'te güvenli duruş | Enerji-bütçeli karar (aşırı mühendislik); sadece uyarı (otonomi hedefiyle çelişir) | Basit, öngörülebilir, v1 için doğru |
| Orkestrasyon | Nav2 `opennav_docking` (kurulu: 1.3.12) + `battery_manager` düğümü | Tam özel durum makinesi (staging/retry/kurtarma sıfırdan); karma (orta yol kazandırmıyor) | Pişmiş retry/staging bedava; A2'nin üstüne doğal biner; docking'in riskli kısmı yazılım değil |

> İşaretçi ailesi notu: konuşmada "ArUco" dendi; Jazzy apt'inde `aruco_opencv`
> yok, AprilTag algılayıcıları var (`ros-jazzy-apriltag-detector*`). Mimari
> aynı — "işaretçi" = AprilTag.

## Alt projelere ayrıştırma (her biri kendi spec → plan → uygulama döngüsü)

| # | Alt proje | Ne üretir | Bağımlılık |
|---|---|---|---|
| SP1 | **Batarya izleme** | `ina228_driver` paketi + SoC tahmini + tek yetkili `/battery` | yok — ilk yazılacak spec |
| SP2 | **Nav2 config** | yol haritasındaki A2; eve dönüşün taşıyıcısı | A1 ✅ |
| SP3 | **İşaretçi algısı** | kamera bringup (`camera_ros` Dockerfile'a) + AprilTag → `detected_dock_pose` | kamera devreye alma |
| SP4 | **Docking** | `opennav_docking` config; muhtemelen hazır `simple_charging_dock` eklentisi yeter (harici poz + BatteryState'ten şarj tespiti destekliyor) | SP2, SP3 |
| SP5 | **battery_manager davranışı** | eşik → iptal → dön → dock → şarj durum makinesi | SP1, SP4 |
| SP6 | **İstasyon donanımı** | kontaklar, pilot devresi + röle, V-huni, işaretçi montajı; `docs/charging-station.md` + `wiring-map.md` güncellemesi | İz B ile paralel |

Sıra: SP1 → SP2 → SP3 → SP4 → SP5; SP6 donanım izinde paralel.

**ESP32 protokolüne dokunulmuyor.** Akım sensörü Pi I2C'de (karar: yavaş akıllı
sensörler Pi'da). İki yerde tanımlı seri protokolün senkron yükü bu işe girmiyor;
motor komut yolu, watchdog, E-stop ve çarpma vetosu aynen kalıyor.

## Bileşenler

### Akım sensörü (SP1)
- **INA228** (85 V sınıfı) — INA226 OLMAZ: 10S paket dolarken 42 V, INA226'nın
  36 V bus sınırını aşar.
- I2C adres 0x40 (MPU6050 0x68 ile çakışmaz), Pi I2C bus'ına.
- **Şönt yerleşimi multimetreyle doğrulanacak** (BMS topolojisi belgesiz —
  tahmin yürütülmez): tercih **ortak ana eksi hat** (tek şönt hem deşarjı hem
  şarj akımını görür). Mümkün değilse: şönt deşarj hattına, şarj tespiti voltaj
  sıçramasından (`bat_voltage` kontak enerjilenince belirgin yükselir).
- Paket: `ros2/src/ina228_driver/` — `mpu6050_driver` deseninin kopyası:
  ROS'suz register-seviyesi sınıf + `fake_bus` + düğüm.

### SoC tahmini ve yetkili `/battery` (SP1)
- `battery_manager` paketi içinde tahminci: coulomb sayacı (mAh integrali) +
  açılışta dinlenim voltajından SoC başlatma + şarj sonu tespitiyle
  (akım kuyruğu düşük VE voltaj ≥ ~41.5 V) %100'e sıfırlama.
- Girdi: köprünün ham voltajı + INA228 akımı. Çıktı: **tek yetkili
  `BatteryState`** (voltaj, akım, `percentage` dolu). Köprünün mevcut yayını
  `/battery_raw`'a remap edilir; tüketiciler tek kaynaktan okur.

### Davranış durum makinesi (SP5)
```
NORMAL ──SoC<DÖN──▶ RETURNING ──staging'e vardı──▶ DOCKING ──şarj akımı──▶ CHARGING ──dolu──▶ DOCKED_IDLE
   │                    │ (Nav2 NavigateToPose)       (DockRobot aksiyonu)
   └──SoC<KRİTİK (her durumda)──▶ SAFE_STOP (görev iptal, sıfır cmd_vel, diagnostics ERROR)
```
- Eşikler histerezisli ve parametrik, config'te **CALIBRATE** işaretli
  (mevcut gelenek). Başlangıç tahminleri: DÖN ~%25, KRİTİK ~%10 — sahada
  kalibre edilir.
- `battery_manager` motor komut yoluna **yeni kapı eklemez**: Nav2'ye hedef
  verir / görevi iptal eder. KRİTİK duruşu = sıfır hız + görev iptali;
  E-stop insan/donanım kanalı olarak saf kalır.

### Docking akışı (SP3 + SP4)
- Ev pozu = harita çerçevesinde kayıtlı waypoint. Nav2 robotu staging pozuna
  getirir (~2-3 m, işaretçi görüş alanında).
- `opennav_docking` devralır: AprilTag düğümü → adaptör → `detected_dock_pose`,
  görsel geri beslemeyle yanaşma, retry hazır.
- Yanaşma **ileri yönde** (kamera önde). Kontaklar, istasyon gövdesi tamponu
  basmayacak şekilde konumlandırılır. Tampon basılırsa ileri veto yanaşmayı
  keser — bu korumanın çalışmasıdır (veto latch'siz, robot geri çekilip
  retry yapabilir).

### İstasyon tarafı (SP6)
- Eldeki 42 V şarj aleti istasyonda; ana kontaklar normalde **ölü**.
- Robotta iki küçük **pilot pedi** arası köprülü tel: robot doğru oturunca
  pilot devresi kapanır → röle ana kontakları enerjiler. Yanlış hizada açıkta
  gerilim yok.
- Mekanik V-huni son ~10 cm hatayı affeder.
- Çıktılar: `docs/charging-station.md`, `wiring-map.md` güncellemesi,
  İz B yol haritasına adımlar.

## Hata durumları ve güvenlik

- **Docking başarısız (retry'lar tükendi):** SAFE_STOP + `/diagnostics` ERROR.
  Robot istasyonun dibinde durur; insan müdahalesi v1 için kabul.
- **Dönüş yolunda KRİTİK:** olduğu yerde durur. "Eve yetişemedi" v1'de kabul
  edilen risk.
- **İşaretçi görünmüyor** (karanlık/kir/açı): dock sunucusu zaman aşımı →
  retry → başarısızlık yolu.
- **Oturdu ama akım yok** (oksit, röle çekmedi): ayrı teşhis mesajı —
  mekanik/elektrik ayrımı için.
- **INA228 arızası / I2C kopması:** `percentage` NaN olur; `battery_manager`
  muhafazakâr **voltaj-yedeğine** düşer + WARN (uzun pencerede filtreli voltaj;
  DÖN ~3.5 V/hücre, KRİTİK ~3.3 V/hücre — CALIBRATE). Sensör kaybı robotu
  köreltmez, temkinli yapar.
- **Değişmeyenler:** E-stop, watchdog, çarpma vetosu docking dahil her fazda
  aktif; docking normal `/cmd_vel` yolundan sürer.

## Test stratejisi (A1 düzeninin devamı, donanımsız)

**Birim:** coulomb sayacı matematiği · eşik histerezisi · `ina228_driver`
sahte I2C bus'a karşı · şarj-sonu tespiti.

**Sim entegrasyonu:** `robot_sim`'e sahte batarya modeli (hareketle boşalan,
dock toleransı içindeyken dolan) + ground truth'tan gürültülü sahte
`detected_dock_pose`. Repo testleri:
1. SoC eşiğin altına inince görev iptal edilip eve dönülüyor
2. Staging hatasıyla başlasa da yanaşma yakınsıyor
3. Kritik eşikte her durumda duruyor
4. Şarj akımı görülünce CHARGING'e geçip SoC %100'e sıfırlanıyor
5. Sensör kaybında voltaj-yedeğine düşüyor

**Simin kanıtlamadıkları (fazla güvenme):** kontak mekaniği, oksitlenme,
gerçek şarj eğrisi, güneşte AprilTag algısı, INA doğruluğu — hepsi İz B
tezgah/kalibrasyon işi. Simülasyon bu projede yazılımı sağlamlaştırır;
fizik/elektrik riskini azaltmaz.

## Bekleyen alımlar (bu özellik için)

- INA228 modülü + uygun şönt (~150-300 TL)
- Temas pedleri/yaylı kontak malzemesi (~100-200 TL)
- Röle modülü (~50 TL)
- AprilTag baskısı (~0), huni malzemesi (hurda olabilir)
