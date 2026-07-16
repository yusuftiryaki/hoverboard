# howerboard — hoverboard parçalarından dış mekan otonom yer robotu

## Dil ve oturum başlangıcı
- **Kullanıcı Türkçe konuşuyor — her zaman Türkçe yanıt ver.**
- Yeni oturumda önce `docs/handoff.md`'yi oku: projenin tüm kararları,
  donanım envanteri ve açık işler orada. Oturum sonunda `/handoff` ile güncelle.

## Proje yapısı
- `ros2/` — colcon workspace (ROS 2 **Jazzy**). Paketler `ros2/src/` altında:
  - `hoverboard_bridge/` — ESP32 seri köprü düğümü + protokol + sahte ESP32 (`fake_esp32.py`)
  - `mpu6050_driver/` — IMU düğümü + sahte I2C bus (`fake_bus.py`)
  - `robot_bringup/` — launch dosyaları, `config/ekf.yaml`, URDF
- `firmware/esp32_bridge/` — PlatformIO/Arduino ESP32 firmware'i
- `docs/` — Türkçe dokümantasyon (handoff, deployment, wiring, bringup checklist)
- `scripts/deploy.sh` — Pi'a SSH ile pull + build

## Komutlar
```bash
# ROS 2 build (ros2/ içinde)
cd ros2 && colcon build --symlink-install

# Testler (paket dizininde; donanım gerektirmez, sahte sınıflar kullanılır)
cd ros2/src/hoverboard_bridge && python3 -m pytest test -q
cd ros2/src/mpu6050_driver   && python3 -m pytest test -q

# ESP32 firmware derleme (donanım gerekmez)
cd firmware/esp32_bridge && pio run

# Pi'a deploy (repo remote'a bağlı olmalı)
PI_HOST=enes@robot.local ./scripts/deploy.sh
```

## Kurallar
- `ros2/build/`, `ros2/install/`, `ros2/log/` ve `firmware/*/.pio/` **üretilmiş
  dizinlerdir** — asla elle düzenleme (hook zaten engeller). Kaynak her zaman
  `ros2/src/` ve `firmware/*/src/` altında.
- Seri protokol **iki yerde** tanımlı: `firmware/esp32_bridge/src/main.cpp` (C++)
  ve `ros2/src/hoverboard_bridge/hoverboard_bridge/protocol.py` (Python).
  Birini değiştirirsen diğerini de senkron tut; testleri koş.
- **Donanım güvenliği:** Bu kod 36V bataryalı, fiziksel olarak hareket eden bir
  robotu sürüyor. Motor komut yolunu etkileyen değişikliklerde hız limitlerini,
  watchdog/deadman zaman aşımını ve bağlantı kopunca güvenli duruşu koru.
- Donanım pinout'ları (SWD pad sırası, USART rolleri, hall sırası) belgelenmemiş
  TXTY 6052 kartı için **tahmin edilmez** — `docs/wiring-map.md` ve multimetre
  doğrulaması esastır.
- Geliştirme makinesinde gerçek donanım yok; düğümler sahte sınıflarla
  (`fake_esp32.py`, `fake_bus.py`) test edilir. Gerçek donanım Pi üzerinde.
