#!/usr/bin/env python3
"""The rig's GPIO (gateware/arty_slots.py's AUX), by shield pin, over Etherbone.

    python3 scripts/gpio.py build/slots_ci/                 # every pin: driven or not, level
    python3 scripts/gpio.py build/slots_ci/ IO32=0 IO33=1   # drive them
    python3 scripts/gpio.py build/slots_ci/ IO32=z          # release it (input)

Pins come up released.  A pin driven keeps its level until set again or released.
"""

import argparse
import json
import os
import sys

from litex.tools.remote.comm_udp import CommUDP


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("build", help="build/slots_<layout>/ of the loaded bitstream")
    parser.add_argument("pins", nargs="*", help="PIN=0, PIN=1 or PIN=z")
    parser.add_argument("--host", default="10.42.0.50")
    args = parser.parse_args()

    generated = os.path.join(args.build, "generated")
    names = json.load(open(os.path.join(generated, "rig.json")))["gpio"]
    regs = json.load(open(os.path.join(generated, "csr.json")))["csr_registers"]
    if not names or "gpio_oe" not in regs:
        sys.exit("this layout has no GPIO")
    addr = {r: regs[f"gpio_{r}"]["addr"] for r in ("oe", "out", "in")}

    bus = CommUDP(args.host)
    bus.open()
    oe, out = bus.read(addr["oe"]), bus.read(addr["out"])
    for setting in args.pins:
        pin, _, level = setting.partition("=")
        if pin not in names or level not in ("0", "1", "z"):
            sys.exit(f"{setting}: want one of {', '.join(names)} =0, =1 or =z")
        bit = 1 << names.index(pin)
        if level == "z":
            oe &= ~bit
        else:
            out = out | bit if level == "1" else out & ~bit
            oe |= bit
    if args.pins:
        bus.write(addr["out"], out)
        bus.write(addr["oe"], oe)
    levels = bus.read(addr["in"])
    bus.close()
    for i, pin in enumerate(names):
        driven = (oe >> i) & 1
        print(f"{pin:5} {'driven' if driven else 'input '} {(levels >> i) & 1}")


if __name__ == "__main__":
    main()
