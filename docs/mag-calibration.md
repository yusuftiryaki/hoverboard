# Manyetometre Kalibrasyonu (QMC5883L)

> **Kalibrasyonsuz manyetometre, manyetometresizlikten KÖTÜDÜR.** Yön vermez;
> *kendinden emin şekilde yanlış* bir yön verir ve hata robotla birlikte döner.
> `ekf_global` ona mutlak ölçüm diye güvenir, `map→odom`'u ona göre kurar, Nav2
> da her hedefi ona göre çevirir.
>
> Ölçüldü (`test_qmc5883l.py`): dünyanın yatay alanının üçte biri kadar bir hard
> iron ofseti yön hatasını **>15°** yapıyor; ofset çıkarılınca **<0.5°**.

## Neden gerekli

Manyetometre dünyanın alanını değil, **bulunduğu yerdeki toplam alanı** ölçer.
Robot kendi alanını taşır:

- **Hard iron** — mıknatıslanmış çelik, kablolardaki DC akım. Sabit bir vektör
  ekler; sensörün çizdiği kürenin **merkezini kaydırır**. Robot dönünce bu ofset
  dönmez, o yüzden **ortalaması alınarak yok olmaz** — yönü, yöne bağlı bir
  miktarda kaydırır.
- **Soft iron** — yakındaki demir, dünyanın alanını büker. Küreyi **elipsoide**
  çevirir.

Düzeltme: `hard_iron` çıkar, sonra `soft_iron_scale` ile çarp.

## Ön koşul: montaj (karar 4)

Kalibrasyon **kötü montajı kurtarmaz**. Önce:
- Direğe, gövdeden **>30 cm** yukarı.
- Motor kablolarından ve hub motorlardan uzak.
- ⚠️ `/diagnostics`'te **`overflows` artıyorsa** ya da `field_uT` 25-65 aralığı
  dışındaysa: **sensörü taşı.** 8 G aralığına geçmek sorunu çözmez, gizler.

Hard iron akımla değişir, o yüzden kalibrasyonu **motorlar bağlıyken ve robot
normal çalışma durumundayken** yap. Tekerlekler havada olsun.

## Prosedür

```bash
# 1. Sensörü çalıştır (kalibrasyonsuz — uyarı verecek, normal)
ros2 launch robot_bringup robot.launch.py use_mag:=true

# 2. Ham alanı kaydet: robotu YAVAŞÇA en az iki tam tur döndür.
#    Sadece yatay dönmek x/y'yi kalibre eder; z için robotu eğmek gerekir
#    (yatay sürüş için x/y yeterli).
ros2 topic echo /imu/mag --field magnetic_field > /tmp/mag_raw.txt
```

Sonra min/max'tan ofset ve ölçek:

```
hard_iron[i]      = (max[i] + min[i]) / 2          # kürenin merkezi
soft_iron_scale[i]= ortalama_yarıçap / yarıçap[i]  # yarıçap[i] = (max[i]-min[i])/2
                    (ortalama_yarıçap = üç yarıçapın ortalaması)
```

Değerleri `ros2/src/robot_bringup/config/qmc5883l.yaml` içine yaz (**tesla**).

## Doğrulama

```bash
# Alan büyüklüğü dönerken SABİT kalmalı — dünyanın alanı sabittir, sadece yönü değişir.
# Değişiyorsa kalibrasyon eksik.
ros2 topic echo /diagnostics | grep field_uT   # 25-65 uT arası ve sabit olmalı

# Robotu bilinen bir yöne çevir, madgwick'in yaw'ıyla karşılaştır (REP-103: 0 = DOĞU)
ros2 topic echo /imu/data --field orientation
```

⚠️ **Manyetik sapma (declination) ayrı bir iş.** Kalibrasyon sensörü düzeltir;
sapma *manyetik kuzey* ile *gerçek kuzey* arasındaki açıdır ve
`ekf.yaml` → `navsat_transform` → `magnetic_declination_radians` içine girer.
Sahan için: https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
(İstanbul ~+6.4° = 0.112 rad). **Şu an 0.0** — simülatörün alanında sapma yok,
gerçek sayı sim testlerini sessizce bozardı. Saha çıkışından önce ayarla.

## Neden düğümde, madgwick'te değil

`imu_filter_madgwick`'in de `mag_bias_x/y/z` parametreleri var, ama kalibrasyonu
sürücüde yapıyoruz: ikisinde birden yaparsak ofset **iki kez** çıkarılır. Sürücü
ayrıca soft iron'ı da uygular, madgwick uygulamaz. Madgwick'inkiler 0.0'da kalsın.
