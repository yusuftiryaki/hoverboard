#!/usr/bin/env bash
#
# WSL disk temizlik scripti — güvenli, regenere olabilen cache'leri temizler.
# Devcontainer kullanımı korunur (en yeni server sürümleri tutulur).
#
# Kullanım:
#   ./wsl-cleanup.sh            # önce ne kadar yer açılacağını gösterir, onay ister
#   ./wsl-cleanup.sh --yes      # onaysız çalıştırır
#   ./wsl-cleanup.sh --dry-run  # hiçbir şey silmeden sadece raporlar
#
set -uo pipefail

DRY_RUN=0
ASSUME_YES=0
for arg in "$@"; do
  case "$arg" in
    --dry-run) DRY_RUN=1 ;;
    --yes|-y)  ASSUME_YES=1 ;;
    *) echo "Bilinmeyen argüman: $arg"; exit 1 ;;
  esac
done

# En yeni kaç devcontainer server sürümü tutulsun
KEEP_CONTAINER_VERSIONS=2

# --- yardımcılar ---------------------------------------------------------
human() { du -sh "$1" 2>/dev/null | cut -f1; }
size_kb() { du -sk "$1" 2>/dev/null | cut -f1; }

FREED_KB=0
add_freed() { FREED_KB=$(( FREED_KB + ${1:-0} )); }

# Bir yolu (glob dahil) güvenle temizler, açılan yeri raporlar
clean_path() {
  local label="$1"; shift
  local total=0 found=0
  for p in "$@"; do
    [ -e "$p" ] || continue
    found=1
    local kb; kb=$(size_kb "$p"); kb=${kb:-0}
    total=$(( total + kb ))
    if [ "$DRY_RUN" -eq 0 ]; then
      rm -rf "$p"
    fi
  done
  if [ "$found" -eq 1 ]; then
    printf "  %-42s %8s\n" "$label" "$(numfmt --to=iec --from-unit=1024 $total 2>/dev/null || echo ${total}K)"
    add_freed "$total"
  fi
}

echo "============================================================"
if [ "$DRY_RUN" -eq 1 ]; then
  echo " WSL TEMİZLİK — DRY RUN (hiçbir şey silinmez)"
else
  echo " WSL TEMİZLİK"
fi
echo "============================================================"
df -h / | awk 'NR==1||/\/$/{print "  "$0}'
echo

# --- 1) Devcontainer eski server sürümleri -------------------------------
CDIR="$HOME/.vscode-remote-containers/bin"
if [ -d "$CDIR" ]; then
  echo "[1] Devcontainer eski server sürümleri (en yeni $KEEP_CONTAINER_VERSIONS tutulur):"
  mapfile -t OLD < <(ls -1t "$CDIR" 2>/dev/null | tail -n +$((KEEP_CONTAINER_VERSIONS + 1)))
  if [ "${#OLD[@]}" -eq 0 ]; then
    echo "  (silinecek eski sürüm yok)"
  else
    for v in "${OLD[@]}"; do
      clean_path "server $v" "$CDIR/$v"
    done
  fi
  echo
fi

# --- 2) VS Code server cache'leri ----------------------------------------
echo "[2] VS Code server cache'leri:"
clean_path "CachedExtensionVSIXs" "$HOME/.vscode-server/data/CachedExtensionVSIXs"/*
clean_path "vscode-server logs"   "$HOME/.vscode-server/data/logs"/*
echo

# --- 3) Paket yöneticisi cache'leri --------------------------------------
echo "[3] Paket yöneticisi cache'leri (regenere olur):"
clean_path "npm _cacache"       "$HOME/.npm/_cacache"
clean_path "pip cache"          "$HOME/.cache/pip"
clean_path "uv cache (~/.cache)" "$HOME/.cache/uv"
clean_path "uv cache (.local)"  "$HOME/.local/share/uv/cache"
clean_path "huggingface cache"  "$HOME/.cache/huggingface"
echo

# --- 4) Sistem cache/log (sudo gerekebilir) ------------------------------
echo "[4] Sistem cache & log:"
if [ "$DRY_RUN" -eq 0 ]; then
  if sudo -n true 2>/dev/null || [ -t 0 ]; then
    before=$(size_kb /var/cache/apt); sudo apt-get clean 2>/dev/null
    after=$(size_kb /var/cache/apt);  freed=$(( ${before:-0} - ${after:-0} ))
    printf "  %-42s %8s\n" "apt clean" "$(numfmt --to=iec --from-unit=1024 $freed 2>/dev/null)"; add_freed "$freed"

    before=$(size_kb /var/log/journal)
    sudo journalctl --vacuum-size=100M >/dev/null 2>&1
    after=$(size_kb /var/log/journal); freed=$(( ${before:-0} - ${after:-0} ))
    printf "  %-42s %8s\n" "journal vacuum -> 100M" "$(numfmt --to=iec --from-unit=1024 $freed 2>/dev/null)"; add_freed "$freed"
  else
    echo "  (sudo yok — apt/journal atlandı)"
  fi
else
  printf "  %-42s %8s\n" "apt clean"              "~$(human /var/cache/apt)"
  printf "  %-42s %8s\n" "journal vacuum -> 100M" "~$(human /var/log/journal)"
fi
echo

# --- 5) İSTEĞE BAĞLI: proje build cache'leri (varsayılan KAPALI) ----------
# Güvenli ama projeye tekrar girince yeniden oluşur; ilk build/açılış yavaşlar.
# Kullanmak istersen aşağıdaki satırların başındaki # işaretini kaldır:
#
# echo "[5] Proje build cache'leri:"
# clean_path "oba browse.vc.db" "$HOME/repos/oba_projesi/.vscode/browse.vc.db"
# clean_path "angular cache"    "$HOME/repos/entes/entwatch/EEnerjiDoktoruWeb/ClientApp/.angular/cache"
# clean_path "agac .next"       "$HOME/repos/agac-goruntuleme/.next"
# echo

echo "============================================================"
printf " TOPLAM %s: %s\n" \
  "$([ "$DRY_RUN" -eq 1 ] && echo 'açılabilir' || echo 'açıldı')" \
  "$(numfmt --to=iec --from-unit=1024 $FREED_KB 2>/dev/null || echo ${FREED_KB}K)"
echo "============================================================"

if [ "$DRY_RUN" -eq 0 ]; then
  echo
  echo "NOT: WSL'de silinen alan Windows'taki .vhdx dosyasını otomatik küçültmez."
  echo "Diski gerçekten geri kazanmak için Windows'ta (WSL kapalıyken):"
  echo "  wsl --shutdown"
  echo "  diskpart -> select vdisk file=\"...\\ext4.vhdx\" -> compact vdisk"
fi
