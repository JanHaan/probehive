#!/usr/bin/env python3
"""Whether a Vivado build met its timing: setup, hold and pulse width, from its timing report.

    python3 scripts/timing_check.py build/slots_<layout>/gateware/arty_slots_timing.rpt

Prints the worst slack of each clock and exits 1 when any is negative: a bitstream that misses
timing may work on the bench and fail with the next placement, temperature or board.
"""

import re
import sys

COLUMNS = ("WNS", "WHS", "WPWS")


def summary(text):
    """The design's worst setup, hold and pulse width slack, and each clock's."""
    lines = text.splitlines()
    design, clocks = None, []
    for i, line in enumerate(lines):
        if re.match(r"\s*WNS\(ns\)\s+TNS\(ns\)", line) and design is None:
            values = lines[i + 2].split()
            design = (float(values[0]), float(values[4]), float(values[8]))
        if re.match(r"Clock\s+WNS\(ns\)", line) and not clocks:
            for row in lines[i + 2:]:
                parts = row.split()
                if not parts:
                    break
                try:
                    clocks.append((parts[0], float(parts[1]), float(parts[5]), float(parts[9])))
                except (IndexError, ValueError):
                    pass                                  # a clock without paths of some kind
    return design, clocks


def main():
    text = open(sys.argv[1]).read()
    design, clocks = summary(text)
    if design is None:
        sys.exit(f"{sys.argv[1]}: no design timing summary")
    for name, *slack in clocks:
        print(f"  {name:16} " + "  ".join(f"{c} {s:+7.3f} ns" for c, s in zip(COLUMNS, slack)))
    met = all(s >= 0 for s in design)
    print(("timing met: " if met else "TIMING NOT MET: ") +
          ", ".join(f"{c} {s:+.3f} ns" for c, s in zip(COLUMNS, design)))
    return 0 if met else 1


if __name__ == "__main__":
    sys.exit(main())
