# Geliştirme Ortamı — Devcontainer + Flashing İş Akışı

## Ne için?

Tek container iki tarafı da karşılıyor:
- **ROS 2 (Jazzy)** düğüm geliştirme + `colcon build` → Pi'a deploy edilir.
- **ESP32 firmware** derleme (PlatformIO) + opsiyonel flash.
- **STM32 flash** araçları (`st-flash`, OpenOCD) — hoverboard anakartı için.

Container **x86_64 dev makinesinde (WSL2)** çalışır. Robotun **çalışma zamanı
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
| `ms-vscode.cpptools` | C++ düğümler + firmware IntelliSense |
| `ms-vscode.cmake-tools`, `twxs.cmake` | ament_cmake / colcon |
| `platformio.platformio-ide` | ESP32 derle / flash / monitor |
| `ms-vscode.vscode-serial-monitor` | ESP32 ↔ Pi seri hattını canlı izle |
| `redhat.vscode-yaml` | ROS param, EKF/Nav2 config, launch.yaml |
| `redhat.vscode-xml` + `smilerobotics.urdf` | URDF/xacro + 3B önizleme |
| `ms-azuretools.vscode-docker` | container yönetimi |
| `yzhang.markdown-all-in-one` | `docs/` klasörü |
| `eamodio.gitlens` | git |

## Container içinde Claude Code (host login'i paylaşılıyor)

Container'da tekrar login olmana gerek yok — host WSL'deki oturum mount ediliyor:
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

## Flashing: WSL2 gerçeği

WSL2 USB cihazlarını doğrudan görmez. İki yol var.

### Yol A (önerilen) — Pi'ı flashing merkezi yap
ESP32 zaten Pi'a USB ile bağlı; ST-Link'i de Pi'a takabilirsin. Yani:

```bash
# container'da derle
cd firmware/esp32_bridge && pio run
# çıktı: .pio/build/esp32dev/firmware.bin

# Pi'a gönder ve Pi'dan flash'la
scp .pio/build/esp32dev/firmware.bin pi@<ip>:/tmp/
ssh pi@<ip> 'esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 921600 \
    write_flash 0x1000 /tmp/firmware.bin'
```
STM32 için de aynı mantık: ST-Link Pi'a takılı, `st-flash write firmware.bin 0x8000000`
komutu Pi üzerinde çalışır. WSL USB passthrough derdi yok, sahada da kullanışlı.

### Yol B — container'dan doğrudan flash (usbipd-win)
Windows tarafında bir kez kurulum:

```powershell
# Windows PowerShell (yönetici)
winget install usbipd            # bir kez
usbipd list                      # ESP32 / ST-Link'in BUSID'sini bul
usbipd bind   --busid <b-id>
usbipd attach --wsl --busid <b-id>
```
Sonra WSL'de `/dev/ttyUSB0` (ESP32) veya `/dev/ttyACM0` (ST-Link) belirir.
`devcontainer.json` içindeki ilgili `--device=` satırını aç, container'ı rebuild et.
Artık:
```bash
pio run -t upload            # ESP32
st-flash write firmware.bin 0x8000000   # STM32
```

> Cihazı her takışında `usbipd attach` tekrar gerekir (reboot sonrası da). Bu
> tekrar eden sürtünme, Yol A'yı çoğu zaman daha pratik yapıyor.

## Derleme her zaman donanımsız çalışır
Hangi yolu seçersen seç, `pio run` (ESP32 derleme) ve `colcon build` (ROS 2)
hiçbir donanım olmadan container'da çalışır. Donanım yalnızca **flash** ve
**gerçek çalıştırma** için gerekir.
