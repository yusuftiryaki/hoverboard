# SP1 — Batarya İzleme Uygulama Planı

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** INA228 akım sensörü sürücüsü + coulomb sayan SoC tahmini + tek yetkili `/battery` yayını (spec: `docs/superpowers/specs/2026-07-16-batarya-yonetimi-ve-sarj-istasyonu-design.md`, SP1).

**Architecture:** İki yeni ament_python paketi. `ina228_driver` = ROS'suz, register seviyesi çip sınıfı + sahte I2C bus (`mpu6050_driver` deseninin birebir kopyası). `battery_manager` = ROS'suz `SocEstimator` + `battery_monitor` düğümü: INA228'i sahiplenir, köprünün `/battery_raw`'a remap edilen ham voltajını dinler, yetkili `/battery`'yi (`percentage` dolu) yayınlar. Sensör yoksa/ölürse voltaj-yalnız moda düşer (percentage NaN + WARN) — bugünkü Pi'da (INA alınmadı) düğüm böyle çalışacak.

**Tech Stack:** ROS 2 Jazzy, rclpy, sensor_msgs/BatteryState, smbus2 (tembel import), pytest.

## Global Constraints

- Dokümantasyon ve log/kullanıcı metinleri Türkçe; **kod içi yorumlar İngilizce** (repo geleneği — mevcut dosyalara bak).
- Birim testler **ROS'suz koşmalı** (saf Python); entegrasyon testleri `pytest.importorskip("rclpy")` ile atlanmalı. Hook her düzenlemede testleri zorlar.
- `ros2/build/`, `ros2/install/`, `ros2/log/` üretilmiş dizinler — asla elle düzenleme.
- **ESP32 seri protokolüne ve motor komut yoluna dokunulmaz.** Bu plan yalnızca dinler ve yayınlar.
- Ölçülemeyen her sabit config'te **CALIBRATE** yorumu taşır (mevcut gelenek, `hoverboard_bridge.yaml`'a bak).
- INA228 donanımı henüz alınmadı; her şey `fake_bus` ve voltaj-yalnız modla doğrulanır.
- Test komutları: `cd ros2 && source install/setup.bash && python3 -m pytest src/<paket>/test -q`. Build: `cd ros2 && colcon build --symlink-install`.
- Commit mesajları Türkçe, mevcut git geçmişinin üslubunda.

## Sözleşmeler (tüm görevlerin ortak dili)

- **Akım işareti — BatteryState geleneği:** deşarj **negatif**, şarj **pozitif** (sensor_msgs/BatteryState böyle tanımlar). `SocEstimator.update()` ve `/battery.current` bu gelenektedir. Sürücünün ham işareti şönt kablolamasına bağlıdır; düğümdeki `invert_current` parametresi (CALIBRATE) düzeltir.
- **Düşük-taraf şönt:** şönt paketin ortak eksi hattında (spec kararı). Bu yüzden INA228'in VBUS pini ~0 V okur ve **kullanılmaz** — paket voltajı köprünün `/battery_raw`'ından gelir.
- Paket voltajı = 10 hücre; hücre başına değerler ×10 ölçeklenir.

---

### Görev 1: `ina228_driver` paket iskeleti + 24-bit register çözümü

**Files:**
- Create: `ros2/src/ina228_driver/package.xml`
- Create: `ros2/src/ina228_driver/setup.py`
- Create: `ros2/src/ina228_driver/setup.cfg`
- Create: `ros2/src/ina228_driver/resource/ina228_driver` (boş dosya)
- Create: `ros2/src/ina228_driver/ina228_driver/__init__.py` (boş)
- Create: `ros2/src/ina228_driver/ina228_driver/ina228.py`
- Test: `ros2/src/ina228_driver/test/test_ina228.py`

**Interfaces:**
- Consumes: —
- Produces: `ina228.py` modül sabitleri (`REG_*`, `*_LSB_*`, `DEFAULT_ADDRESS=0x40`, `DEVICE_ID_VALUE=0x228`) ve `_to_signed20(raw24: int) -> int` — Görev 2'nin sınıfı ve fake bus bunları import eder.

- [ ] **Step 1: Paket iskeletini oluştur**

`setup.cfg` (mpu6050_driver'ınkiyle aynı kalıp):

```ini
[develop]
script_dir=$base/lib/ina228_driver
[install]
install_scripts=$base/lib/ina228_driver
```

`package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>ina228_driver</name>
  <version>0.1.0</version>
  <description>
    INA228 high/low-side current monitor driver, register level, no ROS.
    Reads VSHUNT and computes current in Python (I = Vshunt / Rshunt) instead of
    programming SHUNT_CAL and the on-chip CURRENT register — we know the shunt
    value and do not need the chip's energy accumulators. The bus is injected,
    same pattern as mpu6050_driver.
  </description>
  <maintainer email="enesis@entes.com.tr">Enes</maintainer>
  <license>MIT</license>

  <!-- No smbus2 dependency on purpose: the bus is injected, this package never
       imports an I2C library. The node that opens the real bus (and therefore
       depends on python3-smbus2) is battery_manager. -->

  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

`setup.py`:

```python
from setuptools import find_packages, setup

package_name = "ina228_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Enes",
    maintainer_email="enesis@entes.com.tr",
    description="INA228 current monitor driver (register level, bus injected).",
    license="MIT",
    tests_require=["pytest"],
    # No console_scripts: this package is a library. The node that owns the
    # chip lives in battery_manager.
)
```

- [ ] **Step 2: Başarısız testi yaz**

`test/test_ina228.py`:

```python
"""INA228 driver tests against the simulated bus — no I2C, no ROS.

Like test_mpu6050.py these pin the silently-wrong failure modes: the 20-bit
left-aligned register layout, byte order, and LSB scale factors. Datasheet:
TI SBOSA20 (INA228), sections 7.5 and 8.1.
"""

import pytest

from ina228_driver.ina228 import _to_signed20


def test_signed20_decoding():
    # Registers are 24 bits on the wire, data is the top 20, bits 3..0 reserved.
    assert _to_signed20(0x000010) == 1          # 1 << 4
    assert _to_signed20(0x000000) == 0
    assert _to_signed20(0xFFFFF0) == -1
    assert _to_signed20(0x800000) == -524288    # most negative 20-bit value
    assert _to_signed20(0x7FFFF0) == 524287     # most positive
```

- [ ] **Step 3: Testin başarısız olduğunu doğrula**

Çalıştır: `cd ros2/src/ina228_driver && python3 -m pytest test -q`
Beklenen: FAIL — `ModuleNotFoundError: No module named 'ina228_driver.ina228'` (ya da import hatası)

- [ ] **Step 4: `ina228.py`'nin sabitler + decode kısmını yaz**

```python
"""Register-level INA228 driver. No ROS, no I2C library import.

The bus is injected (anything with smbus2.SMBus's read_i2c_block_data /
write_byte_data-style surface), so this module unit-tests on a dev machine with
no I2C at all — see fake_bus.py. Same pattern as mpu6050_driver.

Register map and LSB values: TI INA228 datasheet (SBOSA20), section 7.5.

Design choice: we read VSHUNT and divide by the known shunt resistance in
Python instead of programming SHUNT_CAL and reading the on-chip CURRENT
register. That path exists to feed the chip's internal energy/charge
accumulators, which we do not use — coulomb counting happens in
battery_manager where it can be unit tested.

⚠️ Registers are BIG-endian on the wire (MSB first), and the 24-bit data
registers carry a 20-bit value left-aligned (bits 3..0 reserved). Getting
either wrong yields plausible-looking garbage, so both are pinned in tests.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Protocol

# ---- Registers ---------------------------------------------------------------
REG_CONFIG = 0x00
REG_ADC_CONFIG = 0x01
REG_VSHUNT = 0x04      # 24-bit, signed 20-bit left-aligned
REG_VBUS = 0x05        # 24-bit, unsigned 20-bit left-aligned
REG_DIETEMP = 0x06     # 16-bit, signed
REG_MANUFACTURER_ID = 0x3E
REG_DEVICE_ID = 0x3F

DEFAULT_ADDRESS = 0x40         # A1=A0=GND
MANUFACTURER_TI = 0x5449       # "TI"
DEVICE_ID_VALUE = 0x228        # DEVICE_ID bits 15..4; bits 3..0 are the die rev

# LSBs (SBOSA20 section 7.5). ADCRANGE=0 (±163.84 mV) — a 40 A robot through a
# 1.5 mΩ shunt is 60 mV, comfortably inside; no need for the fine range.
VSHUNT_LSB_V = 312.5e-9
VBUS_LSB_V = 195.3125e-6
DIETEMP_LSB_C = 7.8125e-3

# CONFIG bits
BIT_RESET = 0x8000

# ADC_CONFIG: MODE[15:12]=0xF continuous shunt+bus+temp, conversion time code
# 0b101 (1052 µs) for all three, AVG code 0b010 (16 samples). Net: a fresh
# averaged reading every ~50 ms — plenty for a 10 Hz battery monitor, and the
# averaging kills motor-PWM ripple on the shunt.
ADC_CONFIG_VALUE = (0xF << 12) | (0x5 << 9) | (0x5 << 6) | (0x5 << 3) | 0x2


class I2CBus(Protocol):
    """The slice of smbus2.SMBus this driver uses."""

    def read_i2c_block_data(self, addr: int, register: int, length: int) -> List[int]: ...
    def write_i2c_block_data(self, addr: int, register: int, data: List[int]) -> None: ...


@dataclass(frozen=True)
class PowerSample:
    """One reading, in SI units.

    current sign follows the shunt wiring — the node's invert_current parameter
    (CALIBRATE) maps it onto the BatteryState convention (discharge negative).
    """

    bus_voltage: float   # V at the VBUS pin. ⚠️ ~0 in our low-side install — unused.
    current: float       # A
    temperature: float   # °C, the die


def _to_signed20(raw24: int) -> int:
    value = raw24 >> 4
    return value - (1 << 20) if value >= (1 << 19) else value


def _to_signed16(raw16: int) -> int:
    return raw16 - 65536 if raw16 >= 32768 else raw16
```

(`INA228` sınıfı Görev 2'de eklenecek — bu adımda dosya sabitler + decode ile biter.)

- [ ] **Step 5: Testin geçtiğini doğrula**

Çalıştır: `cd ros2/src/ina228_driver && python3 -m pytest test -q`
Beklenen: PASS (1 test)

- [ ] **Step 6: Commit**

```bash
git add ros2/src/ina228_driver
git commit -m "ina228_driver paket iskeleti: register haritası ve 20-bit çözüm"
```

---

### Görev 2: `FakeINA228Bus` + `INA228` sınıfı (probe/configure/read)

**Files:**
- Modify: `ros2/src/ina228_driver/ina228_driver/ina228.py` (sınıfı ekle)
- Create: `ros2/src/ina228_driver/ina228_driver/fake_bus.py`
- Test: `ros2/src/ina228_driver/test/test_ina228.py` (testleri ekle)

**Interfaces:**
- Consumes: Görev 1'in sabitleri ve `_to_signed20`.
- Produces:
  - `INA228(bus, address=0x40, shunt_ohms=0.0015)` — `.probe() -> int` (device id, eşleşmezse öylece döndürür; karar çağırana ait), `.reset() -> None`, `.configure() -> None`, `.read() -> PowerSample`.
  - `FakeINA228Bus(address=0x40, rshunt_ohms=0.0015, noise_current_a=0.0, seed=0)` — `true_current_a`, `true_bus_v`, `true_temp_c` alanları test/düğüm tarafından sürülür.
  - Görev 4'ün düğümü ikisini de import eder.

- [ ] **Step 1: Başarısız testleri yaz** (`test_ina228.py`'ye ekle)

```python
from ina228_driver.fake_bus import FakeINA228Bus
from ina228_driver.ina228 import (
    ADC_CONFIG_VALUE,
    DEVICE_ID_VALUE,
    REG_ADC_CONFIG,
    INA228,
)


def make(shunt_ohms=0.0015, **kwargs):
    bus = FakeINA228Bus(rshunt_ohms=0.0015, noise_current_a=0.0, **kwargs)
    ina = INA228(bus, shunt_ohms=shunt_ohms)
    ina.reset()
    ina.configure()
    return bus, ina


def test_probe_returns_device_id():
    bus, ina = make()
    assert ina.probe() == DEVICE_ID_VALUE


def test_wrong_address_raises_like_a_real_bus():
    bus = FakeINA228Bus(address=0x40)
    ina = INA228(bus, address=0x44)
    with pytest.raises(OSError):
        ina.probe()


def test_configure_writes_continuous_mode_with_averaging():
    bus, ina = make()
    assert bus.reg16(REG_ADC_CONFIG) == ADC_CONFIG_VALUE


def test_current_scale_round_trip():
    # The fake converts true amps -> VSHUNT counts with ITS OWN rshunt; the
    # driver converts counts -> amps with the configured shunt_ohms. They only
    # agree when the driver's value matches the physical resistor — a wrong
    # shunt_ohms parameter must show up here, not in the field.
    bus, ina = make()
    bus.true_current_a = -12.5   # discharging hard
    assert ina.read().current == pytest.approx(-12.5, rel=1e-3)
    bus.true_current_a = 1.8     # charging
    assert ina.read().current == pytest.approx(1.8, rel=1e-3)


def test_mismatched_shunt_value_reads_wrong_current():
    bus, ina = make(shunt_ohms=0.003)   # config says 3 mΩ, resistor is 1.5 mΩ
    bus.true_current_a = -10.0
    assert ina.read().current == pytest.approx(-5.0, rel=1e-3)


def test_bus_voltage_and_temperature_scale():
    bus, ina = make()
    bus.true_bus_v = 37.2
    bus.true_temp_c = 41.5
    sample = ina.read()
    assert sample.bus_voltage == pytest.approx(37.2, abs=0.001)
    assert sample.temperature == pytest.approx(41.5, abs=0.01)


def test_rejects_nonpositive_shunt():
    with pytest.raises(ValueError):
        INA228(FakeINA228Bus(), shunt_ohms=0.0)
```

- [ ] **Step 2: Testlerin başarısız olduğunu doğrula**

Çalıştır: `cd ros2/src/ina228_driver && python3 -m pytest test -q`
Beklenen: FAIL — `No module named 'ina228_driver.fake_bus'`

- [ ] **Step 3: `INA228` sınıfını `ina228.py`'ye ekle**

```python
class INA228:
    def __init__(self, bus: I2CBus, address: int = DEFAULT_ADDRESS,
                 shunt_ohms: float = 0.0015) -> None:
        if shunt_ohms <= 0.0:
            raise ValueError("shunt_ohms must be positive")
        self._bus = bus
        self._address = address
        self._shunt_ohms = shunt_ohms

    # ---- Setup ----------------------------------------------------------------
    def probe(self) -> int:
        """Device id (0x228 for an INA228). Caller decides what a mismatch means."""
        return self._read_u16(REG_DEVICE_ID) >> 4

    def reset(self) -> None:
        self._write_u16(REG_CONFIG, BIT_RESET)

    def configure(self) -> None:
        """Continuous shunt+bus+temp conversions with 16-sample averaging."""
        self._write_u16(REG_ADC_CONFIG, ADC_CONFIG_VALUE)

    # ---- Reading ---------------------------------------------------------------
    def read(self) -> PowerSample:
        vshunt = _to_signed20(self._read_u24(REG_VSHUNT)) * VSHUNT_LSB_V
        vbus = (self._read_u24(REG_VBUS) >> 4) * VBUS_LSB_V
        temp = _to_signed16(self._read_u16(REG_DIETEMP)) * DIETEMP_LSB_C
        return PowerSample(
            bus_voltage=vbus,
            current=vshunt / self._shunt_ohms,
            temperature=temp,
        )

    # ---- Wire format -------------------------------------------------------------
    def _read_u16(self, register: int) -> int:
        raw = self._bus.read_i2c_block_data(self._address, register, 2)
        if len(raw) != 2:
            raise OSError(f"short I2C read: {len(raw)} of 2 bytes")
        return (raw[0] << 8) | raw[1]

    def _read_u24(self, register: int) -> int:
        raw = self._bus.read_i2c_block_data(self._address, register, 3)
        if len(raw) != 3:
            raise OSError(f"short I2C read: {len(raw)} of 3 bytes")
        return (raw[0] << 16) | (raw[1] << 8) | raw[2]

    def _write_u16(self, register: int, value: int) -> None:
        self._bus.write_i2c_block_data(
            self._address, register, [(value >> 8) & 0xFF, value & 0xFF]
        )
```

- [ ] **Step 4: `fake_bus.py`'yi yaz**

```python
"""A simulated INA228 on a simulated I2C bus.

Same philosophy as mpu6050_driver's fake_bus: register-level, not a stub that
returns canned PowerSamples. The fake converts its ground-truth amps into
VSHUNT counts using ITS OWN physical shunt resistance, so a driver configured
with the wrong shunt_ohms reads a wrong current and the tests catch it.

What it does NOT model: shunt tempco, I2C glitches, PWM ripple aliasing,
contact resistance. Passing here means the maths is right, not that the
install is good — the multimeter check in docs/wiring-map.md is the truth.
"""

from __future__ import annotations

import random
from typing import Dict, List

from ina228_driver.ina228 import (
    BIT_RESET,
    DEFAULT_ADDRESS,
    DEVICE_ID_VALUE,
    DIETEMP_LSB_C,
    MANUFACTURER_TI,
    REG_CONFIG,
    REG_DIETEMP,
    REG_DEVICE_ID,
    REG_MANUFACTURER_ID,
    REG_VBUS,
    REG_VSHUNT,
    VBUS_LSB_V,
    VSHUNT_LSB_V,
)

_WIDTHS = {REG_VSHUNT: 3, REG_VBUS: 3}  # everything else is 2 bytes


def _clamp_s20(value: float) -> int:
    return max(-(1 << 19), min((1 << 19) - 1, int(round(value))))


class FakeINA228Bus:
    """Quacks like the slice of smbus2.SMBus that INA228 uses."""

    def __init__(self, address: int = DEFAULT_ADDRESS, rshunt_ohms: float = 0.0015,
                 noise_current_a: float = 0.0, seed: int = 0) -> None:
        self._address = address
        self._rshunt = rshunt_ohms
        self._noise_a = noise_current_a
        self._rng = random.Random(seed)  # seeded: the tests must be deterministic
        self._regs: Dict[int, int] = {}

        # Ground truth the test (or the node's fake mode) drives.
        self.true_current_a = 0.0   # sign: + charging, - discharging, as wired
        self.true_bus_v = 36.0     # ⚠️ low-side install reads ~0 here; tests set it
        self.true_temp_c = 30.0

    # ---- smbus2-compatible surface --------------------------------------------
    def read_i2c_block_data(self, addr: int, register: int, length: int) -> List[int]:
        self._check_addr(addr)
        if length != _WIDTHS.get(register, 2):
            raise OSError(f"unexpected read length {length} for reg 0x{register:02x}")
        if register == REG_MANUFACTURER_ID:
            value = MANUFACTURER_TI
        elif register == REG_DEVICE_ID:
            value = (DEVICE_ID_VALUE << 4) | 0x1  # rev 1 die
        elif register == REG_VSHUNT:
            amps = self.true_current_a + self._rng.gauss(0.0, self._noise_a)
            value = (_clamp_s20(amps * self._rshunt / VSHUNT_LSB_V) & 0xFFFFF) << 4
        elif register == REG_VBUS:
            value = (max(0, int(round(self.true_bus_v / VBUS_LSB_V))) & 0xFFFFF) << 4
        elif register == REG_DIETEMP:
            value = int(round(self.true_temp_c / DIETEMP_LSB_C)) & 0xFFFF
        else:
            value = self._regs.get(register, 0)
        width = _WIDTHS.get(register, 2)
        return [(value >> (8 * i)) & 0xFF for i in reversed(range(width))]

    def write_i2c_block_data(self, addr: int, register: int, data: List[int]) -> None:
        self._check_addr(addr)
        value = (data[0] << 8) | data[1]
        if register == REG_CONFIG and value & BIT_RESET:
            self._regs = {}   # reset clears config, as the real chip does
            return
        self._regs[register] = value

    def close(self) -> None:
        pass

    # ---- Test conveniences ------------------------------------------------------
    def reg16(self, register: int) -> int:
        return self._regs.get(register, 0)

    def _check_addr(self, addr: int) -> None:
        if addr != self._address:
            raise OSError(121, "Remote I/O error")  # what a real bus gives you
```

- [ ] **Step 5: Testlerin geçtiğini doğrula**

Çalıştır: `cd ros2/src/ina228_driver && python3 -m pytest test -q`
Beklenen: PASS (8 test)

- [ ] **Step 6: Commit**

```bash
git add ros2/src/ina228_driver
git commit -m "INA228 sürücüsü: probe/configure/read + register seviyesi sahte bus"
```

---

### Görev 3: `battery_manager` paketi + `SocEstimator`

**Files:**
- Create: `ros2/src/battery_manager/package.xml`
- Create: `ros2/src/battery_manager/setup.py`
- Create: `ros2/src/battery_manager/setup.cfg`
- Create: `ros2/src/battery_manager/resource/battery_manager` (boş dosya)
- Create: `ros2/src/battery_manager/battery_manager/__init__.py` (boş)
- Create: `ros2/src/battery_manager/battery_manager/soc_estimator.py`
- Test: `ros2/src/battery_manager/test/test_soc_estimator.py`

**Interfaces:**
- Consumes: —
- Produces: `SocEstimator(capacity_ah, cells=10, full_voltage_per_cell=4.15, taper_current_a=0.3, full_hold_s=30.0)`:
  - `.initialized: bool`, `.soc: float | None` (0..1)
  - `.initialize_from_rest_voltage(pack_voltage: float) -> float`
  - `.update(current_a: float, pack_voltage: float, dt: float) -> float` — **akım BatteryState geleneğinde** (deşarj negatif); initialize edilmeden çağrılırsa `RuntimeError`. Dönüş: güncel SoC.
  - `.just_reached_full: bool` — son `update()` şarj-sonu sıfırlaması yaptıysa True (düğüm FULL durumu için okur).

- [ ] **Step 1: Paket iskeletini oluştur**

`setup.cfg` (Görev 1'dekiyle aynı kalıp, `ina228_driver` yerine `battery_manager`).

`package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>battery_manager</name>
  <version>0.1.0</version>
  <description>
    Battery monitoring: coulomb-counting SoC estimator plus the battery_monitor
    node that owns the INA228 and publishes the single authoritative
    sensor_msgs/BatteryState on /battery. Falls back to voltage-only
    (percentage=NaN) when the current sensor is absent or dies. The behavior
    state machine (return-to-dock) is SP5 and lives here later.
  </description>
  <maintainer email="enesis@entes.com.tr">Enes</maintainer>
  <license>MIT</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>diagnostic_msgs</exec_depend>
  <exec_depend>ina228_driver</exec_depend>
  <!-- Ubuntu 24.04 universe, python3-smbus2 0.4.3-1 — same note as
       mpu6050_driver/package.xml. Imported lazily; absent on the dev machine
       is fine. -->
  <exec_depend>python3-smbus2</exec_depend>

  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

`setup.py` (Görev 1'deki kalıp; farklar):

```python
    description="Batarya izleme: SoC tahmini + yetkili /battery yayıncısı.",
    entry_points={
        "console_scripts": [
            "battery_monitor = battery_manager.battery_monitor_node:main",
        ],
    },
```

(entry point Görev 4'te gelecek dosyayı işaret ediyor — sorun değil, bu görevde `battery_monitor` çalıştırılmıyor; testler saf Python.)

- [ ] **Step 2: Başarısız testleri yaz**

`test/test_soc_estimator.py`:

```python
"""SoC estimator tests — pure Python, no ROS, no hardware.

Sign convention throughout: sensor_msgs/BatteryState — discharge NEGATIVE,
charge POSITIVE. The node maps the raw shunt sign onto this via invert_current.
"""

import pytest

from battery_manager.soc_estimator import SocEstimator


def make(capacity_ah=4.4, **kwargs):
    return SocEstimator(capacity_ah=capacity_ah, **kwargs)


def test_starts_uninitialized_and_update_refuses():
    est = make()
    assert not est.initialized
    assert est.soc is None
    with pytest.raises(RuntimeError):
        est.update(-1.0, 38.0, 0.1)


def test_rest_voltage_initialization_interpolates_the_ocv_table():
    est = make()
    # 38.0 V pack = 3.80 V/cell -> table says 0.45 exactly.
    assert est.initialize_from_rest_voltage(38.0) == pytest.approx(0.45, abs=0.01)
    assert est.initialized
    # 41.0 V = 4.10 V/cell -> 0.90; fresh estimator to avoid re-init semantics.
    assert make().initialize_from_rest_voltage(41.0) == pytest.approx(0.90, abs=0.01)


def test_rest_voltage_clamps_outside_the_table():
    assert make().initialize_from_rest_voltage(45.0) == pytest.approx(1.0)
    assert make().initialize_from_rest_voltage(25.0) == pytest.approx(0.0)


def test_discharge_integrates_down():
    est = make(capacity_ah=4.4)
    est.initialize_from_rest_voltage(41.0)   # 0.90
    # 2.2 A discharge for one hour = 2.2 Ah = half of 4.4 Ah capacity.
    for _ in range(3600):
        est.update(-2.2, 37.0, 1.0)
    assert est.soc == pytest.approx(0.90 - 0.50, abs=1e-6)


def test_charge_integrates_up_and_clamps_at_one():
    est = make(capacity_ah=4.4)
    est.initialize_from_rest_voltage(41.0)   # 0.90
    for _ in range(3600):
        est.update(2.2, 41.0, 1.0)           # +0.5 would overshoot 1.0
    assert est.soc == pytest.approx(1.0)


def test_discharge_clamps_at_zero():
    est = make(capacity_ah=4.4)
    est.initialize_from_rest_voltage(30.0)   # 0.0
    est.update(-10.0, 30.0, 60.0)
    assert est.soc == pytest.approx(0.0)


def test_charge_complete_resets_to_full():
    est = make(full_voltage_per_cell=4.15, taper_current_a=0.3, full_hold_s=30.0)
    est.initialize_from_rest_voltage(38.0)   # 0.45, deliberately wrong vs "full"
    # CV tail: pack at 41.6 V, current tapered to 0.2 A, held for 30 s. Exactly
    # 30 updates: the reset fires ON the update that reaches full_hold_s, and
    # just_reached_full is only true for that one update.
    for _ in range(30):
        est.update(0.2, 41.6, 1.0)
    assert est.soc == pytest.approx(1.0)
    assert est.just_reached_full


def test_brief_taper_does_not_reset():
    est = make(full_hold_s=30.0)
    est.initialize_from_rest_voltage(38.0)
    for _ in range(10):                      # only 10 s at full conditions
        est.update(0.2, 41.6, 1.0)
    est.update(-3.0, 37.0, 1.0)              # load returns -> timer must reset
    for _ in range(20):
        est.update(0.2, 41.6, 1.0)           # 20 s < 30 s again
    assert est.soc < 0.6
    assert not est.just_reached_full
```

- [ ] **Step 3: Testlerin başarısız olduğunu doğrula**

Çalıştır: `cd ros2/src/battery_manager && python3 -m pytest test -q`
Beklenen: FAIL — `No module named 'battery_manager.soc_estimator'`

- [ ] **Step 4: `soc_estimator.py`'yi yaz**

```python
"""Coulomb-counting state-of-charge estimator. No ROS.

Init: coulomb counting only measures CHANGE, so the absolute level comes from
an open-circuit-voltage lookup at startup — the robot boots resting, the pack
voltage is a usable OCV. Reset: the only other absolute reference is "the
charger finished" (CV tail: voltage high, current tapered), which snaps the
estimate back to 100% and stops integration drift from accumulating forever.

Sign convention: sensor_msgs/BatteryState — discharge negative, charge
positive. The caller (battery_monitor_node) owns mapping the shunt's raw sign
onto this.

The OCV table is generic Li-ion and the capacity is a guess until measured —
both are CALIBRATE items in battery_manager.yaml.
"""

from __future__ import annotations

# Per-cell open-circuit voltage -> state of charge, generic Li-ion (NMC).
# Good to maybe ±10% SoC; only used at rest, so load sag does not apply.
OCV_PER_CELL = [
    (3.00, 0.00), (3.30, 0.02), (3.50, 0.06), (3.60, 0.13), (3.70, 0.28),
    (3.80, 0.45), (3.90, 0.62), (4.00, 0.78), (4.10, 0.90), (4.20, 1.00),
]


def _interp_ocv(cell_voltage: float) -> float:
    if cell_voltage <= OCV_PER_CELL[0][0]:
        return OCV_PER_CELL[0][1]
    for (v0, s0), (v1, s1) in zip(OCV_PER_CELL, OCV_PER_CELL[1:]):
        if cell_voltage <= v1:
            return s0 + (s1 - s0) * (cell_voltage - v0) / (v1 - v0)
    return OCV_PER_CELL[-1][1]


class SocEstimator:
    def __init__(self, capacity_ah: float, cells: int = 10,
                 full_voltage_per_cell: float = 4.15,
                 taper_current_a: float = 0.3, full_hold_s: float = 30.0) -> None:
        if capacity_ah <= 0.0:
            raise ValueError("capacity_ah must be positive")
        self._capacity_ah = capacity_ah
        self._cells = cells
        self._full_voltage = full_voltage_per_cell * cells
        self._taper_a = taper_current_a
        self._full_hold_s = full_hold_s

        self._soc: float | None = None
        self._full_timer_s = 0.0
        self.just_reached_full = False

    @property
    def initialized(self) -> bool:
        return self._soc is not None

    @property
    def soc(self) -> float | None:
        return self._soc

    def initialize_from_rest_voltage(self, pack_voltage: float) -> float:
        self._soc = _interp_ocv(pack_voltage / self._cells)
        return self._soc

    def update(self, current_a: float, pack_voltage: float, dt: float) -> float:
        """Integrate one step. current_a: + charging, - discharging."""
        if self._soc is None:
            raise RuntimeError("call initialize_from_rest_voltage() first")
        self.just_reached_full = False

        self._soc += current_a * dt / 3600.0 / self._capacity_ah
        self._soc = max(0.0, min(1.0, self._soc))

        # Charge-complete: CV tail — voltage at the top, current tapered off.
        # abs() so a dead-idle robot sitting at a full pack also qualifies.
        if pack_voltage >= self._full_voltage and abs(current_a) <= self._taper_a:
            self._full_timer_s += dt
            if self._full_timer_s >= self._full_hold_s and self._soc < 1.0:
                self._soc = 1.0
                self.just_reached_full = True
        else:
            self._full_timer_s = 0.0
        return self._soc
```

- [ ] **Step 5: Testlerin geçtiğini doğrula**

Çalıştır: `cd ros2/src/battery_manager && python3 -m pytest test -q`
Beklenen: PASS (8 test)

- [ ] **Step 6: Commit**

```bash
git add ros2/src/battery_manager
git commit -m "battery_manager paketi: coulomb sayan SoC tahmini (OCV init + şarj-sonu sıfırlama)"
```

---

### Görev 4: `battery_monitor` düğümü

**Files:**
- Create: `ros2/src/battery_manager/battery_manager/battery_monitor_node.py`
- Test: `ros2/src/battery_manager/test/conftest.py`
- Test: `ros2/src/battery_manager/test/test_battery_monitor_integration.py`

**Interfaces:**
- Consumes: `INA228`, `FakeINA228Bus` (Görev 2); `SocEstimator` (Görev 3); köprünün `BatteryState`'i **`battery_raw`** konusunda (remap Görev 5'te — testte doğrudan yayınlanır).
- Produces: yetkili **`/battery`** (`sensor_msgs/BatteryState`: `voltage` köprüden, `current` BatteryState geleneğinde, `percentage` 0..1 ya da NaN, `power_supply_status` akımdan) + `/diagnostics`. Düğüm adı `battery_monitor`, çalıştırılabilir `battery_monitor`.

- [ ] **Step 1: Başarısız entegrasyon testini yaz**

`test/conftest.py` (hoverboard_bridge'in conftest kalıbı — sinyaller ve süreç grubu notları oradan):

```python
"""Harness for the battery_monitor integration tests.

Run with the workspace sourced:

    cd ros2 && source install/setup.bash
    python3 -m pytest src/battery_manager/test -q

Without ROS the integration tests SKIP; the estimator unit tests still run —
soc_estimator.py is deliberately ROS-free. Nothing here may import rclpy at
module level.

Process management notes are inherited from hoverboard_bridge/test/conftest.py:
`ros2 run` does not forward signals, so every process gets its own session and
is killed by process group.
"""

import os
import signal
import subprocess
import time

import pytest

STARTUP_S = 2.5


@pytest.fixture(scope="session")
def ros():
    rclpy = pytest.importorskip("rclpy", reason="ROS 2 not sourced")
    owner = not rclpy.ok()
    if owner:
        rclpy.init()
    yield rclpy
    if owner:
        rclpy.shutdown()


class MonitorHarness:
    """A battery_monitor node with a chosen parameter set."""

    def __init__(self, *params):
        args = []
        for p in params:
            args += ["-p", p]
        self._proc = subprocess.Popen(
            ["ros2", "run", "battery_manager", "battery_monitor", "--ros-args"] + args,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        time.sleep(STARTUP_S)

    def stop(self):
        os.killpg(os.getpgid(self._proc.pid), signal.SIGINT)
        try:
            self._proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(self._proc.pid), signal.SIGKILL)


@pytest.fixture
def monitor_harness():
    harnesses = []

    def spawn(*params):
        h = MonitorHarness(*params)
        harnesses.append(h)
        return h

    yield spawn
    for h in harnesses:
        h.stop()
```

`test/test_battery_monitor_integration.py`:

```python
"""battery_monitor against the fake INA228 — no hardware.

The test plays the bridge's role: it publishes BatteryState on battery_raw and
watches the authoritative /battery come out the other side.
"""

import math
import time

import pytest

rclpy = pytest.importorskip("rclpy", reason="ROS 2 not sourced")

from rclpy.node import Node
from sensor_msgs.msg import BatteryState


class BridgeStandIn(Node):
    """Publishes battery_raw like hoverboard_bridge does, collects /battery."""

    def __init__(self, voltage=38.0):
        super().__init__("bridge_stand_in")
        self.voltage = voltage
        self.received = []
        self._pub = self.create_publisher(BatteryState, "battery_raw", 10)
        self.create_subscription(BatteryState, "battery", self.received.append, 10)
        self.create_timer(0.1, self._tick)   # bridge feeds back at ~50 Hz; 10 is enough

    def _tick(self):
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = self.voltage
        self._pub.publish(msg)


def spin_for(ros, node, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        ros.spin_once(node, timeout_sec=0.05)


def test_fake_ina_yields_soc_and_signed_current(ros, monitor_harness):
    stand_in = BridgeStandIn(voltage=38.0)   # 3.8 V/cell -> OCV table says 0.45
    try:
        monitor_harness("use_fake_bus:=true", "fake_current_a:=-2.0",
                        "init_window_s:=1.0")
        spin_for(ros, stand_in, 6.0)
        assert stand_in.received, "no /battery published"
        last = stand_in.received[-1]
        assert last.voltage == pytest.approx(38.0, abs=0.1)
        assert last.current == pytest.approx(-2.0, abs=0.1)
        assert last.percentage == pytest.approx(0.45, abs=0.02)
        assert last.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
    finally:
        stand_in.destroy_node()


def test_no_sensor_falls_back_to_voltage_only(ros, monitor_harness):
    # use_fake_bus:=false on the dev machine = exactly today's Pi: no INA228.
    # The node must come up anyway and publish voltage with percentage=NaN.
    stand_in = BridgeStandIn(voltage=38.0)
    try:
        monitor_harness("use_fake_bus:=false", "init_window_s:=1.0")
        spin_for(ros, stand_in, 6.0)
        assert stand_in.received, "no /battery published in fallback mode"
        last = stand_in.received[-1]
        assert last.voltage == pytest.approx(38.0, abs=0.1)
        assert math.isnan(last.current)
        assert math.isnan(last.percentage)
        assert last.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
    finally:
        stand_in.destroy_node()
```

- [ ] **Step 2: Build + testlerin başarısız olduğunu doğrula**

Çalıştır:
```bash
cd ros2 && colcon build --symlink-install && source install/setup.bash
python3 -m pytest src/battery_manager/test -q
```
Beklenen: entegrasyon testleri FAIL (`no /battery published` — düğüm dosyası yok, `ros2 run` sessizce ölür); estimator birim testleri PASS kalır.

- [ ] **Step 3: `battery_monitor_node.py`'yi yaz**

```python
"""INA228 + bridge voltage -> the single authoritative /battery.

Inputs:  battery_raw (BatteryState) — the bridge's board-measured pack voltage,
         remapped away from /battery in robot.launch.py; and the INA228 on the
         Pi's I2C bus for current.
Output:  /battery (BatteryState) with voltage, signed current and percentage —
         the one topic every consumer (SP5's behavior node, dashboards) reads.

Degradation is the design here, not an afterthought: the INA228 is not even
purchased yet, so the node MUST be useful with no sensor. Absent or dead INA ->
percentage/current go NaN, a WARN lands on /diagnostics, and the voltage keeps
flowing. Losing the current sensor makes the robot cautious, not blind.

Why the INA228's own VBUS reading is ignored: the shunt sits in the pack's
common negative lead (low-side, docs/wiring-map.md), so the chip's bus pin
sits near ground and reads ~0 V. Pack voltage comes from the bridge instead.
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

from battery_manager.soc_estimator import SocEstimator
from ina228_driver.ina228 import DEVICE_ID_VALUE, INA228

# Below this magnitude the current is "idle" rather than charging/discharging —
# keeps power_supply_status from flapping on sensor noise.
IDLE_CURRENT_A = 0.05
# This many consecutive I2C failures = the sensor is gone, not glitching.
MAX_CONSECUTIVE_ERRORS = 20


class BatteryMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("battery_monitor")

        # ---- Parameters -------------------------------------------------------
        self.declare_parameter("i2c_bus", 1)        # same bus as the MPU6050
        self.declare_parameter("address", 0x40)     # A1=A0=GND
        self.declare_parameter("rate_hz", 10.0)
        # CALIBRATE: the actual shunt. Wrong value = every current and therefore
        # every SoC estimate is wrong by the same factor.
        self.declare_parameter("shunt_ohms", 0.0015)
        # CALIBRATE: flip so that DISCHARGE reads NEGATIVE (BatteryState
        # convention). Check with the robot idling on battery: current must be
        # a small negative number, not positive.
        self.declare_parameter("invert_current", False)
        # CALIBRATE: measure by running the pack from full to the 3.3 V/cell
        # cutoff once. 4.4 Ah is the typical 10S2P hoverboard pack.
        self.declare_parameter("capacity_ah", 4.4)
        self.declare_parameter("cells", 10)
        self.declare_parameter("full_voltage_per_cell", 4.15)
        self.declare_parameter("taper_current_a", 0.3)
        self.declare_parameter("full_hold_s", 30.0)
        # Rest-voltage averaging window at startup, before the estimator inits.
        self.declare_parameter("init_window_s", 5.0)
        # Dev aids: run against the fake chip with a fixed true current.
        self.declare_parameter("use_fake_bus", False)
        self.declare_parameter("fake_current_a", -2.0)

        p = self.get_parameter
        self._invert = bool(p("invert_current").value)
        self._init_window_s = float(p("init_window_s").value)

        self._estimator = SocEstimator(
            capacity_ah=float(p("capacity_ah").value),
            cells=int(p("cells").value),
            full_voltage_per_cell=float(p("full_voltage_per_cell").value),
            taper_current_a=float(p("taper_current_a").value),
            full_hold_s=float(p("full_hold_s").value),
        )
        self._capacity_ah = float(p("capacity_ah").value)

        # ---- Sensor -----------------------------------------------------------
        self._ina: Optional[INA228] = None
        self._bus = None
        self._setup_sensor()

        # ---- State ------------------------------------------------------------
        self._raw_voltage = float("nan")
        self._raw_stamp = None            # rclpy Time of the last battery_raw
        self._init_samples: list = []
        self._init_started = None
        self._current_a = float("nan")    # BatteryState convention
        self._read_errors = 0
        self._consecutive_errors = 0
        self._is_full = False

        # ---- ROS interfaces ----------------------------------------------------
        self.create_subscription(BatteryState, "battery_raw", self._on_raw, 10)
        self._battery_pub = self.create_publisher(BatteryState, "battery", 10)
        self._diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        rate = float(p("rate_hz").value)
        self._dt = 1.0 / rate
        self.create_timer(self._dt, self._tick)
        self.create_timer(1.0, self._diag_tick)

    # ---- Sensor setup ----------------------------------------------------------
    def _setup_sensor(self) -> None:
        p = self.get_parameter
        address = int(p("address").value)
        try:
            if p("use_fake_bus").value:
                from ina228_driver.fake_bus import FakeINA228Bus

                self.get_logger().warn(
                    "SİMÜLE INA228 kullanılıyor — bu bir geliştirme aracı, sensör değil"
                )
                self._bus = FakeINA228Bus(address=address)
                self._bus.true_current_a = float(p("fake_current_a").value)
            else:
                # Lazy import: the package must run on a machine with no smbus2.
                import smbus2

                self._bus = smbus2.SMBus(int(p("i2c_bus").value))
            ina = INA228(self._bus, address=address,
                         shunt_ohms=float(p("shunt_ohms").value))
            device_id = ina.probe()
            if device_id != DEVICE_ID_VALUE:
                raise OSError(f"0x{address:02x} cevap verdi ama device id "
                              f"0x{device_id:03x} != 0x{DEVICE_ID_VALUE:03x}")
            ina.reset()
            ina.configure()
            self._ina = ina
            self.get_logger().info("INA228 hazır — akım ölçümü ve SoC aktif")
        except (ImportError, OSError, ValueError) as exc:
            # No sensor is a supported configuration (it is today's reality on
            # the Pi), so this is a warning and a mode, not a crash.
            self._ina = None
            self.get_logger().warn(
                f"INA228 yok ({exc}) — voltaj-yalnız mod: percentage=NaN yayınlanacak"
            )

    # ---- Inputs -----------------------------------------------------------------
    def _on_raw(self, msg: BatteryState) -> None:
        self._raw_voltage = msg.voltage
        self._raw_stamp = self.get_clock().now()
        if not self._estimator.initialized:
            now = self.get_clock().now()
            if self._init_started is None:
                self._init_started = now
            self._init_samples.append(msg.voltage)
            elapsed = (now - self._init_started).nanoseconds * 1e-9
            if elapsed >= self._init_window_s:
                rest = sum(self._init_samples) / len(self._init_samples)
                soc = self._estimator.initialize_from_rest_voltage(rest)
                self.get_logger().info(
                    f"SoC başlatıldı: dinlenim {rest:.1f} V -> %{soc * 100.0:.0f}"
                )

    # ---- Main loop ----------------------------------------------------------------
    def _tick(self) -> None:
        if self._ina is not None:
            try:
                sample = self._ina.read()
                raw = -sample.current if self._invert else sample.current
                self._current_a = raw
                self._consecutive_errors = 0
            except OSError as exc:
                self._read_errors += 1
                self._consecutive_errors += 1
                self._current_a = float("nan")
                self.get_logger().warn(f"INA228 okuması başarısız: {exc}",
                                       throttle_duration_sec=5.0)
                if self._consecutive_errors >= MAX_CONSECUTIVE_ERRORS:
                    # Dead, not glitching. Fall back for good; systemd/launch
                    # respawn gives it a fresh chance if it was transient.
                    self._ina = None
                    self.get_logger().error(
                        "INA228 arka arkaya okunamadı — voltaj-yalnız moda geçildi"
                    )

        if (self._estimator.initialized and not math.isnan(self._current_a)
                and not math.isnan(self._raw_voltage)):
            self._estimator.update(self._current_a, self._raw_voltage, self._dt)
            if self._estimator.just_reached_full:
                self._is_full = True
                self.get_logger().info("şarj tamamlandı — SoC %100'e sıfırlandı")
            elif self._current_a < -IDLE_CURRENT_A:
                self._is_full = False

        self._publish()

    def _publish(self) -> None:
        soc = self._estimator.soc
        have_soc = soc is not None and not math.isnan(self._current_a)

        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = self._raw_voltage
        msg.current = self._current_a
        msg.percentage = float(soc) if have_soc else float("nan")
        msg.charge = float(soc) * self._capacity_ah if have_soc else float("nan")
        msg.capacity = self._capacity_ah
        msg.design_capacity = self._capacity_ah
        if math.isnan(self._current_a):
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        elif self._is_full:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL
        elif self._current_a > IDLE_CURRENT_A:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        elif self._current_a < -IDLE_CURRENT_A:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        else:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        msg.present = True
        msg.location = "hoverboard"
        self._battery_pub.publish(msg)

    # ---- Diagnostics -----------------------------------------------------------------
    def _diag_tick(self) -> None:
        status = DiagnosticStatus(name="battery_monitor: batarya", hardware_id="ina228")
        raw_age = None
        if self._raw_stamp is not None:
            raw_age = (self.get_clock().now() - self._raw_stamp).nanoseconds * 1e-9

        if raw_age is None or raw_age > 2.0:
            status.level = DiagnosticStatus.WARN
            status.message = "battery_raw gelmiyor — köprü çalışıyor mu?"
        elif self._ina is None:
            status.level = DiagnosticStatus.WARN
            status.message = "INA228 yok — voltaj-yalnız mod (SoC yok)"
        elif not self._estimator.initialized:
            status.level = DiagnosticStatus.OK
            status.message = "dinlenim voltajı toplanıyor (SoC başlatma)"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "izliyor"
        soc = self._estimator.soc
        status.values = [
            KeyValue(key="voltage_v", value=f"{self._raw_voltage:.2f}"),
            KeyValue(key="current_a", value=f"{self._current_a:.2f}"),
            KeyValue(key="soc", value=f"{soc:.3f}" if soc is not None else "nan"),
            KeyValue(key="read_errors", value=str(self._read_errors)),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._diag_pub.publish(array)

    def destroy_node(self) -> bool:
        try:
            if self._bus is not None:
                self._bus.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[BatteryMonitorNode] = None
    try:
        node = BatteryMonitorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Build + testlerin geçtiğini doğrula**

Çalıştır:
```bash
cd ros2 && colcon build --symlink-install && source install/setup.bash
python3 -m pytest src/battery_manager/test -q
```
Beklenen: PASS (8 birim + 2 entegrasyon). ROS'suz ortamda entegrasyonlar SKIP olmalı: `python3 -m pytest src/battery_manager/test -q` bare interpreter'da 8 passed, 2 skipped.

- [ ] **Step 5: Regresyon — diğer paketlerin testleri hâlâ geçiyor mu**

Çalıştır:
```bash
python3 -m pytest src/hoverboard_bridge/test src/mpu6050_driver/test src/ina228_driver/test -q
```
Beklenen: hepsi PASS (bu görev onlara dokunmadı; hook zaten zorlar).

- [ ] **Step 6: Commit**

```bash
git add ros2/src/battery_manager
git commit -m "battery_monitor düğümü: yetkili /battery, INA228 yoksa voltaj-yalnız mod"
```

---

### Görev 5: Launch + config kablolaması (`/battery_raw` remap'i dahil)

**Files:**
- Create: `ros2/src/robot_bringup/config/battery_manager.yaml`
- Modify: `ros2/src/robot_bringup/launch/robot.launch.py` (köprü Node'una remap + yeni battery_monitor Node + `fake_battery` arg)

**Interfaces:**
- Consumes: `battery_monitor` çalıştırılabiliri (Görev 4).
- Produces: tam yığında `/battery_raw` (köprü, ham) ve `/battery` (yetkili) konu düzeni. SP5'in davranış düğümü `/battery`'yi buradan okuyacak.

- [ ] **Step 1: `config/battery_manager.yaml`'ı yaz**

```yaml
# Parameters for the battery monitor (SP1 of the battery/docking feature —
# docs/superpowers/specs/2026-07-16-batarya-yonetimi-ve-sarj-istasyonu-design.md).
#
# The INA228 is NOT purchased yet. Until it exists on the I2C bus the node runs
# in voltage-only mode: /battery still flows, percentage is NaN, /diagnostics
# carries a WARN. Nothing else in the stack should care.

battery_monitor:
  ros__parameters:
    i2c_bus: 1          # same bus as the MPU6050; INA228 at 0x40, MPU at 0x68
    address: 0x40
    rate_hz: 10.0

    # CALIBRATE: the physical shunt, once bought. Wrong value scales every
    # current and every SoC step by the same factor.
    shunt_ohms: 0.0015
    # CALIBRATE: flip so discharge reads NEGATIVE (BatteryState convention).
    # Robot idling on battery must show a small negative current.
    invert_current: false
    # CALIBRATE: run the pack full -> 3.3 V/cell once and integrate. 4.4 Ah is
    # the typical 10S2P hoverboard pack, a guess until measured.
    capacity_ah: 4.4
    cells: 10

    # Charge-complete detection (CV tail). full_voltage_per_cell is slightly
    # under 4.20 so a tired charger that tops out low still triggers it.
    full_voltage_per_cell: 4.15
    taper_current_a: 0.3
    full_hold_s: 30.0

    # Rest-voltage window for the initial SoC. The robot boots stationary; if
    # someone boots it mid-push the OCV init is wrong until the next full charge.
    init_window_s: 5.0
```

- [ ] **Step 2: `robot.launch.py`'yi düzenle**

1. Modül dokümantasyonundaki örneklerin altına not eklemek gerekmiyor; sadece kod değişiklikleri:
2. `fake_imu` LaunchConfiguration satırının yanına ekle:

```python
    fake_battery = LaunchConfiguration("fake_battery")
```

3. `fake_imu` DeclareLaunchArgument'ının yanına ekle:

```python
        DeclareLaunchArgument(
            "fake_battery", default_value="false",
            description="Simulate the INA228 — dev machine only, there is no I2C there.",
        ),
```

4. Köprü Node'una `remappings` ekle (mevcut Node çağrısında sadece bu satır yeni):

```python
        Node(
            package="hoverboard_bridge",
            executable="hoverboard_bridge",
            name="hoverboard_bridge",
            output="screen",
            parameters=[bridge_params, {"port": esp32_port}],
            # The bridge's voltage is a RAW board measurement; battery_monitor
            # merges it with the INA228 current and owns the real /battery.
            remappings=[("battery", "battery_raw")],
            respawn=True,
            respawn_delay=2.0,
        ),
```

5. Köprü Node'unun hemen altına battery_monitor'u ekle (koşulsuz — sensör yokken voltaj-yalnız moda düşmek düğümün kendi işi):

```python
        # Battery monitor — the single authoritative /battery. Runs even with
        # no INA228 on the bus (today's reality): voltage-only mode, SoC NaN.
        Node(
            package="battery_manager",
            executable="battery_monitor",
            name="battery_monitor",
            output="screen",
            parameters=[battery_params, {"use_fake_bus": fake_battery}],
            respawn=True,
            respawn_delay=2.0,
        ),
```

6. `generate_launch_description` başına, `bridge_params` satırının yanına:

```python
    battery_params = os.path.join(pkg, "config", "battery_manager.yaml")
```

- [ ] **Step 3: `robot_bringup/setup.py`'de config glob'unu doğrula**

Oku: `ros2/src/robot_bringup/setup.py`. `config/*.yaml` glob ile kuruluyorsa değişiklik gerekmez; dosyalar tek tek sayılıyorsa `battery_manager.yaml`'ı listeye ekle.

- [ ] **Step 4: Build + duman testi**

Çalıştır:
```bash
cd ros2 && colcon build --symlink-install && source install/setup.bash
ros2 run hoverboard_bridge fake_esp32 &
FAKE_PID=$!
timeout 25 ros2 launch robot_bringup robot.launch.py esp32_port:=/tmp/fake_esp32 fake_battery:=true &
sleep 12
ros2 topic echo /battery --once
ros2 topic echo /battery_raw --once
kill $FAKE_PID
```
Beklenen: `/battery` mesajında `percentage` sayısal (NaN değil, fake akım -2 A ile), `current: -2.0`; `/battery_raw`'da köprünün ham yayını. `/battery`'yi artık köprü DEĞİL battery_monitor yayınlıyor (`ros2 topic info /battery --verbose` ile düğüm adı `battery_monitor` görünmeli).

- [ ] **Step 5: Tüm test paketlerini koş**

```bash
python3 -m pytest src/hoverboard_bridge/test src/robot_sim/test src/mpu6050_driver/test src/ina228_driver/test src/battery_manager/test -q
```
Beklenen: hepsi PASS. (hoverboard_bridge entegrasyon testleri köprüyü kendi başına, remap'siz koşturur; remap yalnızca robot.launch.py'de yaşadığı için onları etkilemez — yine de kırılan olursa incele, "remap yüzünden olamaz" deme.)

- [ ] **Step 6: Commit**

```bash
git add ros2/src/robot_bringup
git commit -m "robot.launch.py: köprü /battery_raw'a remap, battery_monitor tam yığına eklendi"
```

---

### Görev 6: Dokümantasyon (wiring-map + handoff)

**Files:**
- Modify: `docs/wiring-map.md` (yeni INA228 bölümü)
- Modify: `docs/handoff.md` (SP1 durumu, alımlar, konu düzeni)

**Interfaces:**
- Consumes: önceki görevlerin kararları.
- Produces: SP6 (istasyon donanımı) ve B4 (kalibrasyon) için başvuru metni.

- [ ] **Step 1: `docs/wiring-map.md`'ye INA228 bölümü ekle**

Mevcut bölüm numaralandırmasını izleyerek (dosyayı oku, uygun yere yerleştir) şu içerikte bir bölüm:

```markdown
## INA228 akım sensörü (SP1 — batarya izleme)

Amaç: coulomb sayan SoC için paket akımı. Pi I2C bus 1, adres 0x40 (A1=A0=GND;
MPU6050 0x68 ile çakışmaz).

**Şönt yerleşimi — MULTİMETREYLE DOĞRULA, TAHMİN YÜRÜTME:**
- Hedef: paketin **ortak ana eksi hattı** (BMS'ten çıkan B-/P- ortaklığı),
  ki tek şönt hem deşarjı hem şarj portundan gelen şarj akımını görsün.
- Hoverboard BMS'lerinde şarj (C-) ve deşarj (P-) yolları ayrı olabilir.
  Multimetre süreklilik testiyle doğrula: şarj portu eksisi ile motor besleme
  eksisi şönt konumunun paket tarafında MI birleşiyor?
- Ortak hat yoksa: şönt deşarj hattına; şarj tespiti voltaj sıçramasından
  (battery_raw, kontak enerjilenince yükselir). battery_manager bunu SP5'te ele alır.
- **Düşük-taraf montaj** (eksi hat) → INA228'in VBUS pini şasi civarında kalır,
  ~0 V okur; paket voltajı köprüden gelir. VBUS pinini boşta bırakma, GND'ye bağla.
- Şönt değeri: 1.5 mΩ / ≥50 A sınıfı (40 A tepe × 1.5 mΩ = 60 mV, INA228
  ±163.84 mV aralığının içinde). `shunt_ohms` config'te CALIBRATE.
- İşaret: `invert_current` — robot bataryadan boşta çalışırken /battery.current
  KÜÇÜK NEGATİF olmalı. Pozitifse parametreyi çevir.
- Tüm GND'ler tek ortak noktada (mevcut kural); INA228 I2C'si Pi'ın 3.3 V
  domain'inde, şönt gerilimi mV seviyesinde — uzun kabloyu I2C tarafına değil
  şönt tarafına koyma.
```

- [ ] **Step 2: `docs/handoff.md`'yi güncelle**

Şu değişiklikler (dosyayı oku, mevcut üsluba uydur):
1. "Repo durumu" tablosuna/listesine `ros2/src/ina228_driver/` ve `ros2/src/battery_manager/` satırları.
2. "Bilinen blokerler / bekleyen alımlar"a: **INA228 modülü + 1.5 mΩ/50 A şönt (~150-300 TL)** — SP1 kodu hazır, sensör takılınca `invert_current`, `shunt_ohms`, `capacity_ah` CALIBRATE.
3. Yol haritası İz A'ya SP1'in bittiği, batarya/docking spec ve planının yolu:
   `docs/superpowers/specs/2026-07-16-batarya-yonetimi-ve-sarj-istasyonu-design.md`.
4. Konu düzeni notu: köprü `/battery_raw` (ham), `battery_monitor` `/battery` (yetkili).
5. İz B kalibrasyon adımına (B4): INA228 işaret + şönt + kapasite kalibrasyonu ve
   şönt yerleşiminin multimetre doğrulaması eklendi.

- [ ] **Step 3: Commit**

```bash
git add docs/wiring-map.md docs/handoff.md
git commit -m "Dokümantasyon: INA228 kablolama bölümü, handoff SP1 güncellemesi"
```

---

## Kapsam dışı (bilinçli — sonraki alt projeler)

- Eşik/davranış (eve dön, güvenli dur) → SP5. SP1 yalnızca ölçer ve yayınlar.
- `robot_sim`'e sahte batarya modeli → SP5'in entegrasyon testleriyle birlikte
  (boşalma/dolma davranışının tüketicisi o).
- `mpu6050.yaml` gibi ayrı bir sensör launch girdisi: battery_monitor
  `sensors.launch.py`'ye değil `robot.launch.py`'ye kondu çünkü köprü gibi
  koşulsuz çalışıyor (sensörsüz de anlamlı).
- INA228'in on-chip CURRENT/SHUNT_CAL/enerji akümülatörleri: kullanılmıyor
  (YAGNI — coulomb sayma Python'da, birim test edilebilir yerde).
