#!/usr/bin/env python3
"""Ask the Arty's CMSIS-DAP probe who it is, then find a target on SWD.

    python3 scripts/dap_probe.py [--target rp2040|plain] [--clock HZ]

`rp2040` wakes the RP2040's multidrop SWD (dormant to SWD, then a TARGETSEL
for core 0) before reading DPIDR; `plain` reads DPIDR after a line reset,
as a single-drop Cortex-M (an STM32, say) wants.
"""

import argparse
import time

from dap_udp import (DAP, DAPError, DORMANT_TO_SWD, LINE_RESET, RP2040_CORE0,
                     select_target)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="10.42.0.50")
    parser.add_argument("--slot", type=int, default=0, help="the probe's slot (gateware/arty_slots.py)")
    parser.add_argument("--target", choices=["rp2040", "plain", "none"], default="rp2040")
    parser.add_argument("--clock", type=int, default=1_000_000)
    args = parser.parse_args()

    dap = DAP(args.host, slot=args.slot)
    try:
        for name, identifier in [("vendor", DAP.INFO_VENDOR), ("product", DAP.INFO_PRODUCT),
                                 ("serial", DAP.INFO_SERIAL), ("protocol", DAP.INFO_PROTOCOL),
                                 ("firmware", DAP.INFO_FIRMWARE)]:
            print(f"{name:12} {dap.info_string(identifier)!r}")
        print(f"{'capabilities':12} 0x{dap.info_int(DAP.INFO_CAPABILITIES):02x}")
        print(f"{'packets':12} {dap.info_int(DAP.INFO_PACKET_COUNT)} x "
              f"{dap.info_int(DAP.INFO_PACKET_SIZE)} bytes")

        start = time.perf_counter()
        rounds = 200
        for _ in range(rounds):
            dap.info(DAP.INFO_PACKET_SIZE)
        print(f"{'round trip':12} {(time.perf_counter() - start) / rounds * 1e6:.0f} us "
              f"(DAP_Info, mean of {rounds})")

        if args.target == "none":
            return
        dap.connect_swd()
        dap.swj_clock(args.clock)
        dap.transfer_configure(idle_cycles=0, wait_retry=64)
        dap.swd_configure(turnaround=1)
        if args.target == "rp2040":
            dap.swj_sequence(DORMANT_TO_SWD, 224)
            dap.swj_sequence(LINE_RESET, 64)
            select_target(dap, RP2040_CORE0)
        else:
            dap.swj_sequence(LINE_RESET, 64)
        try:
            dpidr = dap.read_dp(0x0)
            print(f"{'DPIDR':12} 0x{dpidr:08x}")
        except DAPError as error:
            print(f"{'DPIDR':12} no answer: {error}")
    finally:
        if dap.retransmissions:
            print(f"{'resent':12} {dap.retransmissions} requests")
        dap.close()


if __name__ == "__main__":
    main()
