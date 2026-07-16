# Raspberry Pi Deployment Planı

Kararlar: **sıfır Pi → Ubuntu Server 24.04 (64-bit)**, ROS 2 Jazzy **native** (apt),
kod **Pi'da git pull + colcon build**, dev makine ↔ Pi **aynı DDS ağında**.

```
[Dev makine / WSL2 devcontainer]         [Raspberry Pi 4 / Ubuntu 24.04]
  kod yaz, pio derle, colcon test          native ROS 2 Jazzy + gerçek donanım
  git push ──────────────────────────►     git pull + colcon build --symlink-install
  RViz / rqt / teleop  ◄── DDS (LAN) ──►    robot düğümleri (bringup)
                          (aynı ROS_DOMAIN_ID)
```

Repo yapısı:
```
howerboard/
  ros2/                 <- colcon workspace (build burada koşar)
    src/
      hoverboard_bridge/   ESP32 seri köprü düğümü + protokol + sahte ESP32
      robot_bringup/       launch + ekf config + urdf   (nav2 config henüz yok)
  firmware/esp32_bridge/
  docs/  .devcontainer/  scripts/
```
⚠️ Adım 3'teki `git clone` ve `deploy.sh`'ın `git push`'u için repo'nun
**git'e alınmış ve bir remote'a bağlanmış olması** gerekiyor — henüz yapılmadı.

---

## Adım 1 — Pi'a Ubuntu Server 24.04 (headless)

1. **Raspberry Pi Imager** → "Ubuntu Server 24.04 LTS (64-bit)".
2. Imager'da ⚙️ (gelişmiş) ayarlar — SD'yi takmadan hallet:
   - Hostname: `robot` (→ `robot.local` mDNS ile erişilir)
   - SSH: aç, public key ekle (parola yerine key öneririm)
   - WiFi SSID/parola + ülke
   - Kullanıcı adı (örn. `enes`)
3. SD'yi Pi'a tak, boot et. İlk boot cloud-init yüzünden birkaç dakika sürer.
4. `ssh enes@robot.local` ile bağlan. Sonra:
   ```bash
   sudo apt update && sudo apt full-upgrade -y && sudo reboot
   ```

## Adım 2 — ROS 2 Jazzy (native, apt)

```bash
# locale
sudo apt install -y locales && sudo locale-gen en_US en_US.UTF-8

# ROS 2 apt deposu
sudo apt install -y software-properties-common curl && sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $VERSION_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update

# ros-base (masaüstü yok — Pi headless) + araçlar
sudo apt install -y ros-jazzy-ros-base python3-colcon-common-extensions python3-rosdep
sudo rosdep init && rosdep update
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

## Adım 3 — Repo + build

```bash
cd ~ && git clone <repo-url> howerboard
cd ~/howerboard/ros2
rosdep install --from-paths src --ignore-src -r -y   # paket bağımlılıkları
colcon build --symlink-install
echo "source ~/howerboard/ros2/install/setup.bash" >> ~/.bashrc
```
`--symlink-install`: Python düğümlerde her değişiklikte yeniden build gerekmez.

## Adım 4 — ⚠️ Seri cihaz kararlılığı (ESP32 vs GPS çakışması)

ESP32 **ve** GPS'in USB-TTL'i ikisi de USB-serial → `/dev/ttyUSB0/1` numaraları
her boot'ta yer değiştirebilir. udev ile sabit isim ver:

```bash
# hangi cihaz hangisi — tak/çıkar yaparak VID:PID ve seri no bul
lsusb
udevadm info -a -n /dev/ttyUSB0 | grep -E 'idVendor|idProduct|serial'
```
```
# /etc/udev/rules.d/99-robot-serial.rules
# ESP32 (CP2102 örneği; CH340 ise 1a86:7523)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="esp32"
# GPS USB-TTL (FTDI örneği)
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="gps"
```
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG dialout $USER   # relogin gerekir
```
> **Çakışma uyarısı:** ESP32 ile GPS adaptörü **aynı çip**i kullanırsa (ikisi de
> CH340) VID:PID aynı olur, ayırt edilemez. O durumda `ATTRS{serial}` ile ya da
> fiziksel USB portuna göre (`KERNELS==`) ayır — ya da GPS'e farklı çipli
> (FTDI) bir adaptör kullan. En temizi: baştan farklı çip.

## Adım 5 — I2C (IMU + manyetometre) ve kamera

```bash
# I2C aç
sudo apt install -y i2c-tools
# /boot/firmware/config.txt içine:
#   dtparam=i2c_arm=on
#   camera_auto_detect=1
sudo usermod -aG i2c $USER
sudo reboot
# doğrula:
i2cdetect -y 1        # IMU (0x68) ve manyetometre (0x0D) adreslerini görmeli
```
Pi Camera V2: Ubuntu'da `libcamera` ile çalışır; ROS için `camera_ros` veya
`v4l2_camera` düğümü kullanırız (ROS skeleton adımında).

## Adım 6 — Dev döngüsü

`scripts/deploy.sh` push eder + Pi'da pull & build tetikler:
```bash
PI_HOST=enes@robot.local ./scripts/deploy.sh
```
Hızlı Python iterasyonunda (symlink-install sayesinde) çoğu zaman sadece düğümü
yeniden başlatman yeterli, build bile gerekmez.

## Adım 7 — ⚠️ ROS 2 DDS ağı: WSL2 tuzağı

**Sorun:** WSL2 varsayılan olarak NAT arkasında; kendi sanal ağı var. Bu yüzden
devcontainer'daki `--network=host` **yetmez** — WSL2 VM'inin kendisi LAN'daki
Pi ile DDS multicast keşfi yapamaz. "Düğümler Pi'da çalışıyor ama dev makineden
görünmüyor" bunun tipik belirtisidir.

**Çözüm — WSL2 mirrored networking** (Windows 11 22H2+):
```ini
# Windows: %USERPROFILE%\.wslconfig
[wsl2]
networkingMode=mirrored
```
```powershell
wsl --shutdown   # sonra WSL'i yeniden aç
```
Bu, WSL2'yi host ağıyla paylaştırır; artık LAN'daki Pi ile DDS keşfi çalışır.

Sonra **iki tarafta da** aynı domain:
```bash
# hem Pi'ın .bashrc'sinde hem dev makinede
export ROS_DOMAIN_ID=42
```
Test: Pi'da `ros2 run demo_nodes_cpp talker`, dev makinede `ros2 topic list` →
`/chatter` görünmeli.

> Mirrored networking yoksa/çalışmıyorsa alternatif: Fast DDS **Discovery Server**
> (Pi'da sunucu, dev makine ona bağlanır) — multicast'e gerek kalmaz. Gerekirse
> onu ayrıca kurarız.

## Adım 8 — Açılışta otomatik başlatma (systemd)

Robotun bringup launch'ı boot'ta otomatik kalksın:
```ini
# /etc/systemd/system/robot.service
[Unit]
Description=Robot bringup
After=network-online.target

[Service]
Type=simple
User=enes
# donanım hazır olsun diye küçük gecikme
ExecStartPre=/bin/sleep 5
ExecStart=/bin/bash -lc 'source /opt/ros/jazzy/setup.bash && \
  source /home/enes/howerboard/ros2/install/setup.bash && \
  ros2 launch robot_bringup robot.launch.py'
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
```
```bash
sudo systemctl enable --now robot.service
journalctl -u robot.service -f    # logları izle
```
> Geliştirme sırasında bunu **kapalı** tut (`systemctl stop`), yoksa senin elle
> başlattığın düğümlerle çakışır. Sadece "aç-çalış" saha modu için.

---

## Sıra / bağımlılıklar
1–2 (OS + ROS) donanımdan bağımsız, **hemen yapılabilir**. 3 ROS workspace'i
gerektiriyor (sonraki adım). 4–5 (udev, I2C) donanım Pi'a bağlanınca. 7 (DDS)
canlı iletişimi ilk kez isteyince. 8 en son, saha moduna geçerken.
