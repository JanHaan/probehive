#!/usr/bin/env python3
"""Memory access over the Arty's probe to an RP2040, and how fast it is.

    python3 scripts/dap_bench.py [--kib 16] [--clocks 1e6,4e6,...]

Halts core 0, reads a few known words, then for each SWCLK writes random
data to SRAM, reads it back, checks it and reports both rates and how long
one word takes, read with a request of its own.  Finishes with a system
reset, so the Pico boots its program again.
"""

import argparse
import os
import struct
import time

from dap_udp import DAP, DAPError, Memory, attach_rp2040

DHCSR, AIRCR, CPUID = 0xE000EDF0, 0xE000ED0C, 0xE000ED00
SRAM_TEST = 0x20010000


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="10.42.0.50")
    parser.add_argument("--slot", type=int, default=0, help="the probe's slot (gateware/arty_slots.py)")
    parser.add_argument("--kib", type=int, default=16)
    parser.add_argument("--clocks", default="1e6,4e6,10e6,25e6")
    parser.add_argument("--windows", default="1,4", help="requests in flight")
    args = parser.parse_args()

    dap = DAP(args.host, slot=args.slot)
    memory = Memory(dap)
    try:
        print(f"DPIDR    0x{attach_rp2040(dap, 1_000_000):08x}")
        memory.power_up()
        print(f"AP IDR   0x{memory.idr():08x}")
        memory.write32(DHCSR, 0xA05F0003)               # DBGKEY | C_HALT | C_DEBUGEN
        print(f"DHCSR    0x{memory.read32(DHCSR):08x}  (S_HALT is bit 17)")
        print(f"CPUID    0x{memory.read32(CPUID):08x}")
        magic = memory.read32(0x10)
        print(f"bootrom  0x{magic:08x}  {struct.pack('<I', magic)[:3]!r}")

        words = args.kib * 256
        runs = [(int(float(c)), int(w)) for c in args.clocks.split(",")
                for w in args.windows.split(",")]
        for hz, window in runs:
            dap.swj_clock(hz)
            data = list(struct.unpack(f"<{words}I", os.urandom(4 * words)))
            try:
                start = time.perf_counter()
                memory.write_block(SRAM_TEST, data, window)
                write_s = time.perf_counter() - start
                start = time.perf_counter()
                back = memory.read_block(SRAM_TEST, words, window)
                read_s = time.perf_counter() - start
                start = time.perf_counter()
                for _ in range(200):
                    memory.read32(CPUID)
                single_us = (time.perf_counter() - start) / 200 * 1e6
            except DAPError as error:
                print(f"{hz / 1e6:5.1f} MHz  window {window}  failed: {error}")
                dap.swj_clock(1_000_000)
                attach_rp2040(dap, 1_000_000)
                memory.bank = None
                memory.power_up()
                continue
            errors = sum(a != b for a, b in zip(data, back))
            print(f"{hz / 1e6:5.1f} MHz  window {window}  write {args.kib / write_s:7.1f} KiB/s  "
                  f"read {args.kib / read_s:7.1f} KiB/s  one word {single_us:4.0f} us  "
                  f"{'ok' if errors == 0 else f'{errors} words differ'}")
    finally:
        try:
            dap.swj_clock(1_000_000)
            memory.write32(AIRCR, 0x05FA0004)           # VECTKEY | SYSRESETREQ
        except DAPError:
            pass
        print(f"resent   {dap.retransmissions} requests")
        dap.close()


if __name__ == "__main__":
    main()
