#!/usr/bin/env bash
#
# Cargo runner for the nRF52840 Supermini. The board ships with the Adafruit
# nRF52 bootloader, which offers two ways in, and this tries both:
#
#   1. UF2 — the bootloader mounts as a mass-storage drive; copying the file
#      onto it flashes and reboots. No tooling needed on the host.
#   2. Serial DFU — the same bootloader also exposes a CDC port that
#      `adafruit-nrfutil` talks to. Scriptable, and it does not depend on the
#      desktop happening to automount anything.
#
#   cargo run --release          # via .cargo/config.toml
#   tools/flash.sh target/thumbv7em-none-eabihf/release/nrf-daq
#
# **Put the board in bootloader mode first: double-tap RESET.** The application
# does not implement the 1200-baud touch, so there is no way to get there from
# software while it is running.
#
# Overrides: PLANT_TRACE_UF2_MOUNT, PLANT_TRACE_DFU_PORT, PLANT_TRACE_UF2_BASE.
set -euo pipefail

elf="${1:?usage: flash.sh <elf>}"
base="${PLANT_TRACE_UF2_BASE:-0x26000}"
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

bin="${elf}.bin"
uf2="${elf}.uf2"
hex="${elf}.hex"

# `rust-objcopy` comes from cargo-binutils, which is in the dev shell. Fall back
# to the llvm-tools copy inside the toolchain so this also works without it.
if command -v rust-objcopy >/dev/null 2>&1; then
  objcopy=rust-objcopy
else
  objcopy="$(dirname "$(rustc --print target-libdir)")/bin/llvm-objcopy"
fi

"$objcopy" -O binary "$elf" "$bin"
python3 "$here/uf2.py" --base "$base" "$bin" "$uf2"

# ── 1. UF2 mass storage ──────────────────────────────────────────────────────
#
# The drive is found by the presence of INFO_UF2.TXT rather than by its label,
# which differs between vendors of this board.
mount="${PLANT_TRACE_UF2_MOUNT:-}"
if [ -z "$mount" ]; then
  for candidate in /run/media/"$USER"/* /media/"$USER"/* /mnt/*; do
    [ -f "$candidate/INFO_UF2.TXT" ] && mount="$candidate" && break
  done
fi

if [ -n "$mount" ]; then
  echo "bootloader: $mount"
  sed -n 's/^/  /p' "$mount/INFO_UF2.TXT" 2>/dev/null || true
  cp "$uf2" "$mount/"
  sync
  echo "flashed; the board reboots into the application by itself"
  exit 0
fi

# ── 2. Serial DFU ────────────────────────────────────────────────────────────
port="${PLANT_TRACE_DFU_PORT:-}"
if [ -z "$port" ]; then
  # Only consider a port that is *not* the running application: once the app is
  # up it also enumerates as ttyACM, and flashing to it would just time out.
  for candidate in /dev/ttyACM*; do
    [ -e "$candidate" ] || continue
    if ! udevadm info --query=property --name="$candidate" 2>/dev/null |
      grep -q '^ID_MODEL=plant-trace'; then
      port="$candidate"
      break
    fi
  done
fi

if [ -n "$port" ] && command -v adafruit-nrfutil >/dev/null 2>&1; then
  echo "serial DFU on $port"
  "$objcopy" -O ihex "$elf" "$hex"
  # dev-type 0x0052 is the nRF52 family; --singlebank writes straight into the
  # application slot instead of staging a copy, which there is no room for.
  adafruit-nrfutil dfu genpkg --dev-type 0x0052 --application "$hex" "${elf}.zip"
  adafruit-nrfutil dfu serial -pkg "${elf}.zip" -p "$port" -b 115200 --singlebank
  exit 0
fi

cat <<EOF

The board is not in bootloader mode — double-tap RESET, then re-run this.

Once it is, either path works:
  cp $uf2 /run/media/$USER/<DRIVE>/        # drag-and-drop
  PLANT_TRACE_DFU_PORT=/dev/ttyACM0 \\
      tools/flash.sh $elf                  # serial DFU

EOF
exit 0
