#!/usr/bin/env python3
"""PostToolUse hook: ros2/src/<paket>/ altında bir dosya düzenlenince
o paketin pytest'ini koşar. Test kırmızıysa exit 2 ile Claude'a bildirir.

⚠️ ROS PATH'TEN ÇIKARILARAK koşar — `./scripts/ci.sh unit` ile aynı şekilde.
Yani sadece ROS'suz testler koşar, yığın testleri temizce atlanır. İki sebep:
- Ortama bağlıydı: `/opt/ros` source edilmiş ama overlay edilmemiş kabukta
  (Claude'unki tam olarak bu) `robot_sim` her düzenlemede COLLECTION HATASI
  veriyordu (`hoverboard_bridge` bulunamıyor) — test değil, ortam kırmızıydı.
- Yığın testleri dakikalar sürer, aşağıdaki zaman aşımı 90 sn. Zaman aşımı
  pytest'i öldürür ama yığınlar `start_new_session` ile ayrı grupta açılır,
  yani sim_node / köprü / EKF / gz YETİM kalır ve sonraki koşunun
  topic'lerine yayın yapar. Hook bunun kaynağı olmamalı.
Yığın testleri: `./scripts/ci.sh stack nav2 gazebo` ve CI.
"""
import json
import os
import re
import subprocess
import sys

try:
    data = json.load(sys.stdin)
except (json.JSONDecodeError, ValueError):
    sys.exit(0)

path = (data.get("tool_input") or {}).get("file_path") or ""
path_norm = path.replace("\\", "/")

m = re.search(r"(.*?/ros2/src/([^/]+))/", path_norm)
if not m or not path_norm.endswith(".py"):
    sys.exit(0)

pkg_dir = m.group(1)
pkg_name = m.group(2)
test_dir = os.path.join(pkg_dir, "test")
if not os.path.isdir(test_dir):
    sys.exit(0)

env = {k: v for k, v in os.environ.items()
       if k not in ("PYTHONPATH", "AMENT_PREFIX_PATH")}
result = subprocess.run(
    [sys.executable, "-m", "pytest", "test", "-q", "--no-header",
     "-p", "no:cacheprovider"],
    cwd=pkg_dir, capture_output=True, text=True, timeout=90, env=env,
)
if result.returncode != 0:
    print(
        f"'{pkg_name}' paketinin testleri düzenleme sonrası BAŞARISIZ:\n"
        + (result.stdout or "") + (result.stderr or ""),
        file=sys.stderr,
    )
    sys.exit(2)

sys.exit(0)
