#!/usr/bin/env bash
# Removes Fast DDS shared-memory files left behind by ROS processes that died
# without cleaning up (SIGKILL, a killed launch, a crashed test stack).
#
# ⚠️ Why this exists: with ~1800 such files in /dev/shm, every new ROS node
# hangs inside rcl_node_init (Fast DDS probing the dead ports one timeout at a
# time) — silently, no log line. sim_demo.sh came up with Nav2 active while
# web_ui, mission_runner, site_map_server and the battery nodes never got past
# Node.__init__, so the page at :8080 simply never answered. After a wipe the
# same stack served the page in 15 s.
#
# Safe rule: if NO process maps any Fast DDS file, nothing is using them and
# they all go. Otherwise only `fastdds shm clean` runs (the official tool, it
# keeps whatever a live participant holds) — and it says how much it freed.

set -euo pipefail

shopt -s nullglob
files=(/dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*)
[[ ${#files[@]} -gt 0 ]] || exit 0

# grep over the files, not `cat | grep -q`: under pipefail the early exit
# SIGPIPEs cat, the pipeline "fails", and a LIVE stack's files were deleted.
if grep -qs '/dev/shm/\(sem\.\)\?fastrtps_' /proc/[0-9]*/maps; then
  if command -v fastdds >/dev/null; then
    echo ">> DDS: ${#files[@]} paylaşımlı bellek dosyası, bazıları kullanımda — fastdds shm clean"
    fastdds shm clean 2>&1 | sed 's/^/   /' || true
  fi
else
  echo ">> DDS: kullanılmayan ${#files[@]} paylaşımlı bellek dosyası silindi (/dev/shm)"
  rm -f -- "${files[@]}"
fi
