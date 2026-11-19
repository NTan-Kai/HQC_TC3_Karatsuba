#!/usr/bin/env bash
# kr260_load_fpga.sh — Safeguarded Linux FPGA Manager loader for KR260.
# Derived in approach from SoC_FPGA/KV260/KV260_Software/update_bitstream.sh,
# with KR260/overlay and target-file checks.
# Usage:
#   ./kr260_load_fpga.sh --check
#   sudo ./kr260_load_fpga.sh --load design_1_wrapper.bit.bin --dtbo pl.dtbo
#   sudo ./kr260_load_fpga.sh --load design_1_wrapper.bit.bin --bitstream-only
# Add --allow-unverified-board ONLY if the board has been physically confirmed.
set -Eeuo pipefail

usage() {
  cat <<'HELP'
KR260 TC3+Karatsuba Linux FPGA loader

  ./kr260_load_fpga.sh --check
  sudo ./kr260_load_fpga.sh --load FILE.bit.bin --dtbo FILE.dtbo
  sudo ./kr260_load_fpga.sh --load FILE.bit.bin --bitstream-only

Options:
  --check                     Print board and FPGA-manager information; no changes.
  --load FILE                 Full PL configuration in .bin / .bit.bin format.
  --dtbo FILE                 Load matching Device Tree Overlay (fpgautil required).
  --bitstream-only            Explicitly allow programming PL without a DT overlay.
  --allow-unverified-board    Override name check for a physically verified KR260.
  --yes                       Skip interactive confirmation.
  --help                      Show this help.

NOTE: This is Linux FPGA Manager loading, NOT JTAG loading.
      Do NOT use a ZCU104 .bit/.bin on KR260 (different FPGA part).
      The original .bit must first be converted using bootgen on a PC.
HELP
}

MODE=''
BIN=''
DTBO=''
BIT_ONLY=0
ALLOW_UNKNOWN=0
YES=0
while (($#)); do
  case "$1" in
    --check) MODE=check; shift;;
    --load) [[ $# -ge 2 ]] || { echo 'Missing value after --load' >&2; exit 2; }; MODE=load; BIN=$2; shift 2;;
    --dtbo) [[ $# -ge 2 ]] || { echo 'Missing value after --dtbo' >&2; exit 2; }; DTBO=$2; shift 2;;
    --bitstream-only) BIT_ONLY=1; shift;;
    --allow-unverified-board) ALLOW_UNKNOWN=1; shift;;
    --yes) YES=1; shift;;
    --help|-h) usage; exit 0;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2;;
  esac
done
[[ -n "$MODE" ]] || { usage; exit 2; }

get_dt_property() {
  local file=$1
  if [[ -r "$file" ]]; then tr '\000' '\n' < "$file" | head -c 500; fi
}
MODEL=$(get_dt_property /proc/device-tree/model)
COMPATIBLE=$(get_dt_property /proc/device-tree/compatible)
HOST=$(hostname 2>/dev/null || echo unknown)
MANAGER=/sys/class/fpga_manager/fpga0

echo '=== KR260 FPGA loading check ==='
echo "Host:         $HOST"
echo "DT model:     ${MODEL:-unavailable}"
echo "DT compatible:${COMPATIBLE:- unavailable}"
if [[ -d "$MANAGER" ]]; then
  echo "FPGA manager: present ($MANAGER)"
  for prop in name state flags; do
    if [[ -r "$MANAGER/$prop" ]]; then
      printf '  %-6s %s\n' "$prop:" "$(cat "$MANAGER/$prop")"
    fi
  done
else
  echo 'FPGA manager: NOT FOUND'
fi
if command -v fpgautil >/dev/null 2>&1; then
  echo "fpgautil:     $(command -v fpgautil)"
else
  echo 'fpgautil:     NOT FOUND (sysfs fallback only for bitstream-only)'
fi
if [[ -d /sys/kernel/config/device-tree/overlays ]]; then
  echo 'Active configfs overlays:'
  find /sys/kernel/config/device-tree/overlays -mindepth 1 -maxdepth 1 -type d -printf '  %f\n' 2>/dev/null || true
fi

if [[ "$MODE" == check ]]; then
  echo 'Check complete: no hardware changed.'
  exit 0
fi

[[ -f "$BIN" && -s "$BIN" ]] || { echo "ERROR: .bin does not exist/is empty: $BIN" >&2; exit 3; }
case "$BIN" in
  *.bit.bin|*.bin) ;;
  *.bit) echo 'ERROR: .bit has a header. Convert KR260 .bit with bootgen first.' >&2; exit 3;;
  *) echo 'ERROR: expected .bit.bin or .bin, NOT raw .bit.' >&2; exit 3;;
esac
if [[ -n "$DTBO" ]]; then
  [[ -f "$DTBO" && -s "$DTBO" && "$DTBO" == *.dtbo ]] || { echo "ERROR: valid .dtbo required: $DTBO" >&2; exit 3; }
  ((BIT_ONLY == 0)) || { echo 'ERROR: select --dtbo OR --bitstream-only, not both.' >&2; exit 2; }
else
  ((BIT_ONLY == 1)) || { echo 'ERROR: provide matching --dtbo FILE, OR explicitly use --bitstream-only.' >&2; exit 2; }
fi
[[ -d "$MANAGER" ]] || { echo 'ERROR: Linux FPGA Manager is not available.' >&2; exit 4; }

DT_TEXT="${MODEL} ${COMPATIBLE}"
if ! grep -qiE 'kr260|robotics starter kit' <<<"$DT_TEXT"; then
  echo 'WARNING: Device Tree does NOT uniquely identify this device as KR260.' >&2
  echo 'Verify physically: this must be KR260/XCK26, NOT KV260, ZCU104 or TySOM.' >&2
  ((ALLOW_UNKNOWN == 1)) || { echo 'Re-run with --allow-unverified-board only AFTER physical verification.' >&2; exit 5; }
fi

if [[ ${EUID:-$(id -u)} -ne 0 ]]; then
  echo 'ERROR: run --load with sudo (e.g. sudo ./kr260_load_fpga.sh ...).' >&2
  exit 6
fi

# Replacing a loaded FPGA region with an overlay in use can break running programs.
if [[ -d /sys/kernel/config/device-tree/overlays ]]; then
  count=$(find /sys/kernel/config/device-tree/overlays -mindepth 1 -maxdepth 1 -type d 2>/dev/null | wc -l)
  if ((count > 0)); then
    echo "ERROR: $count active device-tree overlay(s) detected." >&2
    echo 'Coordinate with the board owner; stop current applications and remove relevant overlay safely first.' >&2
    echo 'This script intentionally does not delete existing overlays.' >&2
    exit 7
  fi
fi

BIN_ABS=$(realpath "$BIN")
if [[ -n "$DTBO" ]]; then
  DTBO_ABS=$(realpath "$DTBO")
else
  DTBO_ABS=''
fi

echo
printf 'Target:    %s\nBitstream: %s\nOverlay:   %s\n' "$HOST" "$BIN_ABS" "${DTBO_ABS:-NONE (PL only)}"
if ((YES == 0)); then
  read -r -p "This will RECONFIGURE the FPGA. Type LOAD-KR260 to continue: " confirmation
  [[ "$confirmation" == 'LOAD-KR260' ]] || { echo 'Cancelled: no hardware changed.'; exit 8; }
fi

if command -v fpgautil >/dev/null 2>&1; then
  echo 'Loading with fpgautil (Full bitstream)...'
  if [[ -n "$DTBO_ABS" ]]; then
    fpgautil -b "$BIN_ABS" -o "$DTBO_ABS" -f Full
  else
    fpgautil -b "$BIN_ABS" -f Full
  fi
else
  [[ -z "$DTBO_ABS" ]] || { echo 'ERROR: fpgautil is unavailable; cannot apply .dtbo in this script.' >&2; exit 9; }
  [[ -w "$MANAGER/firmware" && -w "$MANAGER/flags" ]] || { echo 'ERROR: FPGA Manager sysfs load attributes are not writable.' >&2; exit 9; }
  stamp=$(sha256sum "$BIN_ABS" | cut -c1-12)
  fwname="kr260_tc3_${stamp}.bin"
  echo "Installing firmware in /lib/firmware/$fwname"
  install -D -m 0644 "$BIN_ABS" "/lib/firmware/$fwname"
  echo 0 > "$MANAGER/flags"  # 0 = Full configuration
  echo "$fwname" > "$MANAGER/firmware"
fi

echo '=== FPGA Manager state after programming ==='
cat "$MANAGER/state" 2>/dev/null || true
echo 'Important: successful loading does not prove that AXI/CRGA works.'
if [[ -z "$DTBO_ABS" ]]; then
  echo 'No DT overlay was loaded. PS-to-PL clocks/bus devices may not be available to Linux.'
fi
