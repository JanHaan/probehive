#!/usr/bin/env python3
"""Find BUFGCTRLs that drive a global line without being configured.

nextpnr has routed data nets through an empty BUFGCTRL (I0 -> O onto a GCLK
spine) while its FASM writer left out that buffer's IN_USE, ZINV and
IS_IGNORE_INVERTED bits.  The buffer then stays off and every load behind it
sees a constant: a block RAM byte lane that never wrote, CSR reads with fixed
bits set.  patches/nextpnr keeps the router out of empty BUFGCTRLs; this
checks a FASM for the defect, whatever made it.

    python3 scripts/bufg_check.py build/slots_<layout>/gateware/arty_slots.fasm [...]

Exits 1 when any FASM has an unconfigured BUFGCTRL on a global line.
"""

import re
import sys


def unconfigured(path):
    driven, used = {}, set()
    for line in open(path):
        m = re.match(r"(CLK_BUFG_\w+_X\d+Y\d+)\.CLK_BUFG_CK_GCLK(\d+)\.CLK_BUFG_BUFGCTRL(\d+)_O", line)
        if m:
            driven[(m[1], int(m[3]))] = int(m[2])
        m = re.match(r"(CLK_BUFG_\w+_X\d+Y\d+)\.BUFGCTRL\.BUFGCTRL_X0Y(\d+)\.IN_USE", line)
        if m:
            used.add((m[1], int(m[2])))
    return [(tile, n, driven[(tile, n)]) for tile, n in sorted(driven) if (tile, n) not in used]


def main():
    bad = 0
    for path in sys.argv[1:]:
        found = unconfigured(path)
        bad += len(found)
        print(f"{path}: " + ("every BUFGCTRL on a global line is configured" if not found else
                             ", ".join(f"{tile} BUFGCTRL{n} drives GCLK{g} unconfigured" for tile, n, g in found)))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
