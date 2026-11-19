#!/bin/sh
# Full PL image only. Does not configure PS clocks, resets or device tree.
set -eu
script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
image=${1:-"$script_dir/design_1_wrapper.bit"}
manager=/sys/class/fpga_manager/fpga0
if [ "$#" -gt 1 ]; then echo "Usage: sudo sh $0 [full-image.bit|full-image.bin]" >&2; exit 2; fi
if [ "$(id -u)" -ne 0 ]; then echo "Run with sudo sh $0 [image]" >&2; exit 2; fi
if [ ! -s "$image" ]; then echo "Missing/empty image: $image" >&2; exit 2; fi
if [ ! -w "$manager/firmware" ] || [ ! -w "$manager/flags" ]; then
    echo "FPGA Manager firmware/flags unavailable: $manager" >&2; exit 2
fi
case "$image" in
    *.bit) suffix=bit ;;
    *.bin) suffix=bin ;;
    *) echo "Expected .bit or .bin full image" >&2; exit 2 ;;
esac
digest=$(sha256sum -- "$image")
digest=${digest%% *}
firmware="hqc_axil_${digest}.${suffix}"
echo "Image: $image"
echo "SHA256: $digest"
echo "Stop other applications using PL before loading."
mkdir -p /lib/firmware
cp -- "$image" "/lib/firmware/$firmware"
printf '0\n' > "$manager/flags"
if ! printf '%s\n' "$firmware" > "$manager/firmware"; then
    echo "FPGA load failed. Inspect: dmesg | tail -n 60" >&2; exit 1
fi
state=$(cat "$manager/state")
echo "FPGA Manager state: $state"
if [ "$state" != operating ]; then
    echo "FPGA not operating. Inspect: dmesg | tail -n 60" >&2; exit 1
fi
echo "PL configuration completed. PS clock/reset and AXI operation still need checking."
echo "Next: sudo ./main --probe"
