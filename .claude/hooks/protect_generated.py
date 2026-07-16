#!/usr/bin/env python3
"""PreToolUse hook: block edits to generated directories.

colcon --symlink-install ve PlatformIO çıktıları elle düzenlenmemeli —
değişiklikler bir sonraki build'de sessizce kaybolur. Kaynak dosyayı
ros2/src/ veya firmware/*/src/ altında düzenleyin.
"""
import json
import sys

BLOCKED_SEGMENTS = ("ros2/build/", "ros2/install/", "ros2/log/", "/.pio/")

try:
    data = json.load(sys.stdin)
except (json.JSONDecodeError, ValueError):
    sys.exit(0)

path = (data.get("tool_input") or {}).get("file_path") or ""
path_norm = path.replace("\\", "/")

for seg in BLOCKED_SEGMENTS:
    if seg in path_norm:
        print(json.dumps({
            "hookSpecificOutput": {
                "hookEventName": "PreToolUse",
                "permissionDecision": "deny",
                "permissionDecisionReason": (
                    f"'{path}' üretilmiş bir dizinde ({seg}) — burası colcon/PlatformIO "
                    "tarafından yeniden yazılır, değişiklik kaybolur. Kaynak dosyayı "
                    "ros2/src/ veya firmware/*/src/ altında düzenle."
                ),
            }
        }))
        sys.exit(0)

sys.exit(0)
