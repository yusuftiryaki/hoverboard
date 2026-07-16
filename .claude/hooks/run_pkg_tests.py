#!/usr/bin/env python3
"""PostToolUse hook: ros2/src/<paket>/ altında bir dosya düzenlenince
o paketin pytest'ini koşar. Test kırmızıysa exit 2 ile Claude'a bildirir.
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

result = subprocess.run(
    [sys.executable, "-m", "pytest", "test", "-q", "--no-header"],
    cwd=pkg_dir, capture_output=True, text=True, timeout=90,
)
if result.returncode != 0:
    print(
        f"'{pkg_name}' paketinin testleri düzenleme sonrası BAŞARISIZ:\n"
        + (result.stdout or "") + (result.stderr or ""),
        file=sys.stderr,
    )
    sys.exit(2)

sys.exit(0)
