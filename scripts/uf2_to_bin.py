#!/usr/bin/env python3
"""One family's blocks of a UF2 file as a flat binary, and the address it
starts at.  Universal images carry several families (an RP2040 image and an
RP2350 one, say), at the same addresses.

    python3 scripts/uf2_to_bin.py blink.uf2 blink.bin [--family 0xe48bff56]
"""

import argparse

import struct
import sys

MAGIC_START0, MAGIC_START1, MAGIC_END = 0x0A324655, 0x9E5D5157, 0x0AB16F30
FLAG_NOT_MAIN_FLASH = 0x1
FLAG_FAMILY = 0x2000
RP2040 = 0xE48BFF56


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("source")
    parser.add_argument("target")
    parser.add_argument("--family", type=lambda v: int(v, 0), default=RP2040)
    args = parser.parse_args()
    data = open(args.source, "rb").read()
    blocks = {}
    for offset in range(0, len(data), 512):
        block = data[offset:offset + 512]
        start0, start1, flags, address, size, _, _, family = struct.unpack("<8I", block[:32])
        end, = struct.unpack("<I", block[508:512])
        if (start0, start1, end) != (MAGIC_START0, MAGIC_START1, MAGIC_END):
            sys.exit(f"block at {offset}: not UF2")
        if flags & FLAG_NOT_MAIN_FLASH or (flags & FLAG_FAMILY and family != args.family):
            continue
        blocks[address] = block[32:32 + size]
    if not blocks:
        sys.exit(f"no blocks of family 0x{args.family:08x}")
    base = min(blocks)
    image = bytearray(max(a + len(b) for a, b in blocks.items()) - base)
    for address, payload in blocks.items():
        image[address - base:address - base + len(payload)] = payload
    open(args.target, "wb").write(image)
    print(f"0x{base:08x} {len(image)} bytes")


if __name__ == "__main__":
    main()
