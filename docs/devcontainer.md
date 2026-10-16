# Geliştirme Ortamı — Devcontainer + Flashing İş Akışı

## Ne için?

Tek container iki tarafı da karşılıyor:
- **ROS 2 (Jazzy)** düğüm geliştirme + `colcon build` → Pi'a deploy edilir.
- **ESP32 firmware** derleme (PlatformIO) + opsiyonel flash.
- **STM32 flash** araçları (`st-flash`, OpenOCD) — hoverboard anakartı için.

Container **x86_64 dev makinesinde (native Ubuntu, WSL2 değil)** çalışır. Robotun **çalışma zamanı
ARM64 Pi'da**; gerçek donanım (seri, kamera, GPIO) burada yok. Bu ortam kod
**yazmak, derlemek ve mantığı test etmek** içindir — gerçek motorla konuşmak için değil.

## Başlangıç

1. VS Code + "Dev Containers" eklentisi (`ms-vscode-remote.remote-containers`) — bu
   host tarafında, container'ı çalıştıran eklenti.
2. Repo'yu aç → "Reopen in Container". İlk build ROS 2 + Nav2 paketleri yüzünden
   birkaç dakika sürer.
3. Container içi eklentiler (ROS, PlatformIO, serial monitor, YAML/XML/URDF…)
   `devcontainer.json` üzerinden otomatik kurulur.

## sudo

`ubuntu` kullanıcısı **parolasız sudo** ile geliyor (Dockerfile'daki
`/etc/sudoers.d/ubuntu` kuralı).

Neden açıkça yazmak gerekti: taban imajın `ubuntu` kullanıcısı zaten `sudo`
grubunda, ama **parolası kilitli** → sudo kimsenin bilmediği bir parola soruyor,
her `apt install` ölüyor. Devcontainer'ların `common-utils` feature'ı normalde
bunu kurar; onu kullanmadığımız için kuralı Dockerfile'a kendimiz yazdık —
böylece ne olduğu ima edilmek yerine görünür oluyor.

> Kapsam: bu **atılır bir geliştirme container'ı**. İçindeki hiçbir şey robotta
> koşmuyor; Pi `docs/deployment.md` ile ayrıca kuruluyor, bu imajdan değil.

⚠️ **Container'a elle `apt install` ettiğin şey bir sonraki rebuild'de kaybolur.**
Kalıcı olması gerekiyorsa Dockerfile'a ekle. sudo hızlı deneme için; kaynak
gerçeği Dockerfile.

## VS Code eklentileri (neden hangisi)

| Eklenti | Ne için |
|---------|---------|
| `ms-iot.vscode-ros` | ROS 2 build task'ları, launch, mesaj/topic gezme |
| `ms-python.python` + `pylance` | Python düğümler |
| `ms-vscode.cmake-tools`, `twxs.cmake` | ament_cmake / colcon |
| `platformio.platformio-ide` | ESP32 derle / flash / monitor |
| `ms-vscode.vscode-serial-monitor` | ESP32 ↔ Pi seri hattını canlı izle |
| `redhat.vscode-yaml` | ROS param, EKF/Nav2 config, launch.yaml |
| `redhat.vscode-xml` + `smilerobotics.urdf` | URDF/xacro + 3B önizleme |
| `ms-azuretools.vscode-docker` | container yönetimi |
| `yzhang.markdown-all-in-one` | `docs/` klasörü |
| `eamodio.gitlens` | git |

## Container içinde Claude Code (host login'i paylaşılıyor)

Container'da tekrar login olmana gerek yok — host'taki oturum mount ediliyor:
- Node.js `features` ile kuruluyor, `postCreateCommand` Claude Code'u
  (`@anthropic-ai/claude-code`) global kuruyor.
- `~/.claude` (OAuth token + ayarlar) ve `~/.claude.json` (hesap/config)
  read-write bind-mount ediliyor.
- Host kullanıcı (uid 1000) ile container `ubuntu` (uid 1000) eşleştiğinden
  0600'lük `.credentials.json` sorunsuz okunuyor.

Container terminalinde sadece `claude` yaz — mevcut oturumla açılır. Login/logout
iki tarafta da paylaşılır (aynı dosyalar).

> Notlar:
> - Bu ayar bu makineye özel: `~/.claude.json` yoksa bind-mount boş klasör
>   oluşturup bozar. Başka makinede kullanılacaksa mount satırlarını koşullu yap.
> - MCP sunucuları (Gmail/Calendar vb.) config'te taşınır ama bazıları
>   interaktif auth istediğinden headless container'da çalışmayabilir — normaldir.

## USB cihazları ve flashing (native Ubuntu host)

Dev makine **doğrudan Ubuntu** çalıştırıyor (2026-10-16'dan beri; eskiden
WSL2'ydi — usbipd, `.wslconfig` gibi adımlar artık **geçersiz**). Host USB'yi
doğrudan görür; container'a geçirmek `devcontainer.json`'daki `runArgs` işi.

İki tür cihaz var ve container'a farklı yollardan girerler:

| Cihaz | Linux'ta ne olarak görünür | Container'a nasıl girer |
|---|---|---|
| **ST-Link V2** (`0483:3748`) | `/dev/bus/usb/...` (libusb) — **tty oluşturmaz** | `-v /dev/bus/usb` bind + `--device-cgroup-rule=c 189:* rmw` → **hazır**, container açıkken tak-çıkar çalışır |
| **ESP32** (CP2102/CH340), **GPS FTDI** | `/dev/ttyUSB*` | `--device=/dev/ttyUSB0` satırı yorumda; açarsan cihaz container başlamadan **önce** takılı olmalı |

> ⚠️ `lsusb` cihazı gösteriyor ama `st-info` "Found 0 stlink programmers"
> diyorsa (`libusb error: -4`): container'da `/dev/bus/usb` yok demektir.
> `lsusb` sysfs'ten okur, açmak için düğüm gerekir. `runArgs` değişikliği
> **Rebuild Container** ister.

### İzinler
Container'daki düğümlerin sahibi/izni **host'un udev'inden** gelir. Host'ta
ST-Link kuralı yoksa düğüm `root:root 0664` olur ve container'da `sudo st-info
--probe` gerekir (parolasız sudo var). Kalıcı çözüm, **host'ta** bir kez:
```bash
sudo apt install stlink-tools   # udev kuralını getirir: GROUP=plugdev
```
Ubuntu'da `plugdev` GID'i varsayılan olarak 46, container'da da 46 ve `ubuntu` kullanıcısı container'da
`plugdev` + `dialout` grubunda → sudo'suz çalışır.

### ST-Link (STM32 anakartı) — container'dan
```bash
st-info --probe                          # programlayıcı + hedef çip
st-flash write firmware.bin 0x8000000    # flash
```

### ESP32 — iki yol
**Container'dan:** `devcontainer.json`'da `--device=/dev/ttyUSB0` satırını aç,
ESP32 takılıyken rebuild et, sonra `pio run -t upload`.

**Pi'dan (sahada pratik):** ESP32 robotta zaten Pi'a USB ile bağlı.
```bash
# container'da derle
cd firmware/esp32_bridge && pio run
# çıktı: .pio/build/esp32dev/firmware.bin

# Pi'a gönder ve Pi'dan flash'la
scp .pio/build/esp32dev/firmware.bin pi@<ip>:/tmp/
ssh pi@<ip> 'esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 921600 \
    write_flash 0x1000 /tmp/firmware.bin'
```
STM32 için de aynısı geçerli: ST-Link Pi'a takılıyken `st-flash` Pi'da koşar.

## Derleme her zaman donanımsız çalışır
Nereden flash'larsan flash'la, `pio run` (ESP32 derleme) ve `colcon build` (ROS 2)
hiçbir donanım olmadan container'da çalışır. Donanım yalnızca **flash** ve
**gerçek çalıştırma** için gerekir.
