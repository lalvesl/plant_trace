#!/usr/bin/env python3
"""Turn a raw binary into a UF2 image for the nRF52840's UF2 bootloader.

There is no packaged bin-to-UF2 converter in nixpkgs for this family, and the
format is small enough not to need one: a UF2 file is a flat sequence of
512-byte blocks, each carrying a 32-byte header, at most 476 payload bytes and
a trailing magic word. The bootloader writes each block at the address in its
header, which is why `--base` has to match the application offset the
bootloader reserves — see `firmware/nrf-daq/memory.x`.

    tools/uf2.py firmware.bin firmware.uf2

Reference: https://github.com/microsoft/uf2 (FORMAT.md)
"""

from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

UF2_MAGIC_START0 = 0x0A324655  # "UF2\n"
UF2_MAGIC_START1 = 0x9E5D5157
UF2_MAGIC_END = 0x0AB16F30

# Bit 13: the file has a familyID in the reserved header word. Without it the
# nRF52840 bootloader accepts the file but will not check that it was built for
# this chip, so an RP2040 image would be written straight into flash.
FLAG_FAMILY_ID = 0x00002000

FAMILY_NRF52840 = 0xADA52840

# 512-byte block, 32-byte header, 4-byte trailing magic.
PAYLOAD_BYTES = 256


def convert(data: bytes, base: int, family: int) -> bytes:
    blocks = (len(data) + PAYLOAD_BYTES - 1) // PAYLOAD_BYTES
    out = bytearray()
    for i in range(blocks):
        # The last chunk is padded to a full 256 bytes, and `payloadSize` says
        # 256 on every block. The Adafruit bootloader rejects any block whose
        # payload is not exactly 256 bytes: a short final block is silently
        # dropped, the block count never completes, and the board sits in the
        # bootloader forever instead of rebooting into the application.
        chunk = data[i * PAYLOAD_BYTES : (i + 1) * PAYLOAD_BYTES].ljust(
            PAYLOAD_BYTES, b"\xff"
        )
        header = struct.pack(
            "<IIIIIIII",
            UF2_MAGIC_START0,
            UF2_MAGIC_START1,
            FLAG_FAMILY_ID,
            base + i * PAYLOAD_BYTES,
            PAYLOAD_BYTES,
            i,
            blocks,
            family,
        )
        # Every block is padded to the full 512 bytes: the bootloader reads the
        # file as fixed-size records.
        out += header + chunk.ljust(476, b"\x00") + struct.pack("<I", UF2_MAGIC_END)
    return bytes(out)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("input", type=Path, help="raw binary (objcopy -O binary)")
    p.add_argument("output", type=Path, help="UF2 file to write")
    p.add_argument(
        "--base",
        type=lambda s: int(s, 0),
        default=0x26000,
        help="flash address of the first byte (default: 0x26000)",
    )
    p.add_argument(
        "--family",
        type=lambda s: int(s, 0),
        default=FAMILY_NRF52840,
        help="UF2 family ID (default: 0xADA52840, nRF52840)",
    )
    args = p.parse_args()

    data = args.input.read_bytes()
    if not data:
        print(f"{args.input}: empty", file=sys.stderr)
        return 1

    uf2 = convert(data, args.base, args.family)
    args.output.write_bytes(uf2)
    print(
        f"{args.output}: {len(uf2) // 512} blocks, "
        f"{len(data)} bytes at {args.base:#x} (family {args.family:#x})"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
