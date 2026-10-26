#!/usr/bin/env python3
"""Does the shell's Yosys keep a shift register's clock enable when it maps
the register to Xilinx SRLs?  Yosys 0.67 does not (YosysHQ/yosys@3006280d,
fixed in 0.68), and LiteEth's header registers stop working.  Simulates
sim/srl_enable/shreg.v as written and as synth_xilinx maps it; they must agree.

    python3 sim/srl_enable_check.py
"""

import os
import subprocess
import sys
import tempfile

HERE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "srl_enable")


def simulate(sources, work):
    binary = os.path.join(work, "sim.vvp")
    subprocess.run(["iverilog", "-o", binary, "-s", "tb", *sources], check=True)
    lines = subprocess.run(["vvp", "-n", binary], check=True, capture_output=True,
                           text=True).stdout.splitlines()
    return next(line for line in lines if line.startswith("head"))


def main():
    shreg, tb = os.path.join(HERE, "shreg.v"), os.path.join(HERE, "tb.v")
    with tempfile.TemporaryDirectory() as work:
        netlist = os.path.join(work, "net.v")
        subprocess.run(["yosys", "-q", "-p", f"read_verilog {shreg}; "
                        f"synth_xilinx -flatten -abc9 -arch xc7 -top shreg; "
                        f"write_verilog -noattr {netlist}"], check=True)
        with open(netlist) as f:
            srls = f.read().count("SRL16E #(")
        datdir = subprocess.run(["yosys-config", "--datdir"], check=True,
                                capture_output=True, text=True).stdout.strip()
        rtl = simulate([tb, shreg], work)
        mapped = simulate([tb, netlist, os.path.join(datdir, "xilinx", "cells_sim.v")], work)
    print(f"RTL:    {rtl}\nmapped: {mapped} ({srls} SRL16E)")
    if rtl != mapped:
        sys.exit("FAIL: this Yosys drops the enable of shift registers it maps to SRLs")
    print("ok")


if __name__ == "__main__":
    main()
