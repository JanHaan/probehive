#!/usr/bin/env python3
"""The Arty A7-35T as several debug probes on Ethernet: slots and a CPU.

Each slot is a probe for one target, on pins of its own; a VexRiscv runs the
CMSIS-DAP commands that do not touch the wire and hands the others to the
slot's engine (gateware/slots.py, firmware/).  All slots share one UDP port,
the datagram naming its slot.

    python3 gateware/arty_slots.py --build           # firmware, then the bitstream
    python3 gateware/arty_slots.py --load
    python3 gateware/arty_slots.py --sim             # Verilog for sim/soc_sim.py

The CPU, its bus and the network side run in sys (50 MHz); the slots' engines
in `slot` (100 MHz), each SWD clock a whole number of its half cycles.
"""

import argparse
import json
import os
import re
import struct
import subprocess
import sys
import time

from migen import Cat, ClockDomain, ClockSignal, If, ResetSignal, Signal

from litex.build.generic_platform import GenericPlatform, IOStandard, Misc, Pins, Subsignal
from litex.gen import LiteXModule
from litex.gen.fhdl.verilog import convert
from litex.soc.cores.clock import S7PLL
from litex.soc.cores.gpio import GPIOTristate
from litex.soc.integration import export
from litex.soc.integration.soc import SoCRegion
from litex.soc.integration.soc_core import SoCCore
from litex.soc.interconnect import stream
from litex.soc.interconnect.csr import CSRStatus

from liteeth.common import eth_udp_user_description
from liteeth.phy.mii import LiteEthPHYMII
from litex_boards.platforms import digilent_arty

from slots import (DAP_UDP_PORT, SLOT_BYTES, UART_RX_DEPTH, UART_TX_DEPTH, JTAGSlot, SWDSlot, SlotBuffers,
                   Transport, add_slot_sources)

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

SYS_CLK_FREQ = 50e6
SLOT_CLK_FREQ = 100e6
CPU_VARIANT = "standard"      # with LiteX's -march for it, RV32IM
MAIN_RAM = 0x40000000
MAIN_RAM_SIZE = 0x8000
SLOTS_BASE = 0x80000000


def swd_pins_io(name, pins):
    """An SWD slot on three pins named outright: SWCLK, SWDIO, nRESET."""
    return [(name, 0,
        Subsignal("swclk", Pins(pins[0])),
        Subsignal("swdio", Pins(pins[1]), Misc("PULLUP=TRUE")),
        Subsignal("nrst", Pins(pins[2]), Misc("PULLUP=TRUE")),
        IOStandard("LVCMOS33"),
    )]


def uart_io(name, pins):
    """A slot's UART: the probe's TX (to the target's RX) and RX (from the target's TX)."""
    return [(name, 0,
        Subsignal("tx", Pins(pins[0])),
        Subsignal("rx", Pins(pins[1]), Misc("PULLUP=TRUE")),
        IOStandard("LVCMOS33"),
    )]


def jtag_slot_io(name, pins):
    """A JTAG slot on five pins: TCK, TMS, TDI, TDO, nRESET."""
    return [(name, 0,
        Subsignal("tck", Pins(pins[0])),
        Subsignal("tms", Pins(pins[1])),
        Subsignal("tdi", Pins(pins[2])),
        Subsignal("tdo", Pins(pins[3]), Misc("PULLUP=TRUE")),
        Subsignal("nrst", Pins(pins[4]), Misc("PULLUP=TRUE")),
        IOStandard("LVCMOS33"),
    )]


def ck_io(*numbers):
    return [f"ck_io:ck_io{n}" for n in numbers]


# Layouts: each slot's kind and pins.  JA and JD carry 200 ohm series resistors, which keep SWD
# clean at 50 MHz; JB and JC have none.  At most eight slots: the transport's header has three bits.
LAYOUTS = {
    # The CI rig: four SWD slots on JA (SWCLK, SWDIO side by side), two JTAG slots on JD (TCK, TMS,
    # TDI, TDO), every nRESET on the shield's IO26-31
    "ci": [("swd", ["pmoda:0", "pmoda:1", *ck_io(26)]), ("swd", ["pmoda:2", "pmoda:3", *ck_io(27)]),
           ("swd", ["pmoda:4", "pmoda:5", *ck_io(28)]), ("swd", ["pmoda:6", "pmoda:7", *ck_io(29)]),
           ("jtag", ["pmodd:0", "pmodd:1", "pmodd:2", "pmodd:3", *ck_io(30)]),
           ("jtag", ["pmodd:4", "pmodd:5", "pmodd:6", "pmodd:7", *ck_io(31)])],
}

# A layout's other pins: each slot's UART (the probe's TX, then RX) and the GPIO, all on the shield
AUX = {
    "ci": {"uart": [ck_io(2 * i, 2 * i + 1) for i in range(6)],
           "gpio": ck_io(32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 12, 13)},
}


def is_jtag(where):
    return isinstance(where, tuple) and where[0] == "jtag"


class CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq, slot_clk_freq=SLOT_CLK_FREQ):
        self.rst = Signal()
        self.cd_sys = ClockDomain()
        self.cd_eth = ClockDomain()
        self.cd_slot = ClockDomain()
        self.pll = pll = S7PLL(speedgrade=-1)
        self.comb += pll.reset.eq(self.rst)
        pll.register_clkin(platform.request("clk100"), 100e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)
        pll.create_clkout(self.cd_eth, 25e6)
        pll.create_clkout(self.cd_slot, slot_clk_freq)
        self.comb += platform.request("eth_ref_clk").eq(self.cd_eth.clk)


class SimCRG(LiteXModule):
    def __init__(self, platform):
        self.cd_sys = ClockDomain()
        self.cd_slot = ClockDomain()
        clocks = platform.request("sim_clocks")
        self.comb += [
            self.cd_sys.clk.eq(clocks.sys), self.cd_sys.rst.eq(clocks.rst),
            self.cd_slot.clk.eq(clocks.slot), self.cd_slot.rst.eq(clocks.rst),
        ]


class SimPlatform(GenericPlatform):
    """Plain ports for sim/soc_sim.py: clocks, the UDP port's streams, the
    slots' DDR pairs, and the console UART."""

    def __init__(self, nslots):
        io = [
            ("sim_clocks", 0, Subsignal("sys", Pins(1)), Subsignal("slot", Pins(1)), Subsignal("rst", Pins(1))),
            ("serial", 0, Subsignal("tx", Pins(1)), Subsignal("rx", Pins(1))),
        ]
        GenericPlatform.__init__(self, "sim", io)
        self.nslots = nslots

    def build(self, *args, **kwargs):
        raise NotImplementedError


class ArtySlots(SoCCore):
    def __init__(self, layout="ci", firmware=None, sim=False, ip_address="10.42.0.50",
                 slot_clk_freq=SLOT_CLK_FREQ, sys_clk_freq=SYS_CLK_FREQ, cpu_variant=CPU_VARIANT,
                 mac_address=0x10e2d5000050, toolchain="vivado"):
        slots = LAYOUTS[layout]
        aux = AUX.get(layout, {})
        if sim:
            platform = SimPlatform(len(slots))
            self.crg = SimCRG(platform)
        elif toolchain == "vivado":
            # The XC7A35T is the XC7A50T's die: build for the 50T in the Arty's grade, then write
            # the 35T's IDCODE into the bitstream (scripts/bitpatch.py), which is all the chip checks.
            platform = digilent_arty.Platform(variant="a7-35", toolchain="vivado")
            platform.device = "xc7a50ticsg324-1L"
            self.crg = CRG(platform, sys_clk_freq, slot_clk_freq)
        else:
            platform = digilent_arty.Platform(variant="a7-35", toolchain="openxc7")
            # LiteX derives the chip database's name from the part, and the
            # industrial grade's "i" yields one no database has.
            platform.toolchain.dbpart = "xc7a35tcsg324"
            # nextpnr finds paths too short for the sink's hold time (into block RAM above all) and,
            # with --timing-allow-fail, which LiteX passes, lets them through without a word: the
            # CPU's caches then read wrong in some placements.  Its hold-fix pass lengthens them.
            platform.toolchain._pnr_opts += "-o hold-fix "
            self.crg = CRG(platform, sys_clk_freq, slot_clk_freq)

        SoCCore.__init__(self, platform, sys_clk_freq, ident="probehive",
            cpu_type="vexriscv", cpu_variant=cpu_variant, cpu_reset_address=MAIN_RAM,
            integrated_rom_size=0, integrated_sram_size=0,
            integrated_main_ram_size=MAIN_RAM_SIZE, integrated_main_ram_init=firmware or [],
            uart_name="serial", uart_baudrate=1_000_000 if not sim else 10_000_000,
            with_timer=False)

        # A free-running count of sys cycles: the firmware's clock
        self.ticks = CSRStatus(32, description="sys cycles since configuration")
        self.sync += self.ticks.status.eq(self.ticks.status + 1)

        # The network: Etherbone for loading firmware and reading registers, and the slots' port
        if sim:
            self.udp_rx = stream.Endpoint(eth_udp_user_description(8))   # into the probe
            self.udp_tx = stream.Endpoint(eth_udp_user_description(8))   # out of it
            port = type("Port", (), {})()
            port.source, port.sink = self.udp_rx, self.udp_tx
        else:
            self.ethphy = LiteEthPHYMII(clock_pads=platform.request("eth_clocks"),
                                        pads=platform.request("eth"))
            self.add_etherbone(phy=self.ethphy, ip_address=ip_address, mac_address=mac_address)
            port = self.ethcore_etherbone.udp.crossbar.get_port(DAP_UDP_PORT, dw=8, cd="etherbone")

        # The slots
        add_slot_sources(platform)
        self.buffers = []
        for i, where in enumerate(slots):
            name = f"slot{i}"
            if sim:
                pads = None
            else:
                if is_jtag(where):
                    platform.add_extension(jtag_slot_io(name, where[1]))
                else:
                    platform.add_extension(swd_pins_io(name, where[1]))
                pads = platform.request(name)
            uart_pads = None
            if not sim and "uart" in aux:
                platform.add_extension(uart_io(f"{name}_uart", aux["uart"][i]))
                uart_pads = platform.request(f"{name}_uart")
            buffers = SlotBuffers()
            setattr(self, f"slot{i}_buffers", buffers)
            self.bus.add_slave(f"slot{i}", buffers.bus,
                               SoCRegion(origin=SLOTS_BASE + i * SLOT_BYTES, size=SLOT_BYTES, cached=False))
            slot = (JTAGSlot if is_jtag(where) else SWDSlot)(pads, buffers, sim=sim, uart_pads=uart_pads,
                                                             sys_clk_freq=sys_clk_freq)
            setattr(self, name, slot)
            self.buffers.append(buffers)
        self.dap = Transport(port, self.buffers, [getattr(self, f"slot{i}") for i in range(len(slots))])
        # A LED a slot, lit while its engine works or its UART moves bytes, and some 40 ms after:
        # slots 0-3 on the green LD4-LD7, the rest on the RGB LEDs' green, dimmed to an eighth
        if not sim:
            leds = [platform.request("user_led", i) for i in range(4)] + \
                   [platform.request("rgb_led", i).g for i in range(4)]
            dim = Signal(3)
            self.sync += dim.eq(dim + 1)
            for i in range(len(slots)):
                held = Signal(22)
                self.sync += If(getattr(self, f"slot{i}").activity, held.eq(2**21)).Elif(held != 0, held.eq(held - 1))
                self.comb += leds[i].eq((held != 0) & ((dim == 0) if i >= 4 else 1))
        # GPIO for the rig's other lines (BOOT, BOOTSEL, RUN, power): read and set over Etherbone
        if not sim and "gpio" in aux:
            platform.add_extension([("rig_gpio", 0, Pins(" ".join(aux["gpio"])), IOStandard("LVCMOS33"))])
            self.gpio = GPIOTristate(platform.request("rig_gpio"))
        # For the host's scripts: which pin is which (generated/rig.json)
        name = lambda pin: pin.split(":")[-1].replace("ck_io", "IO")
        self.rig = {"gpio": [name(p) for p in aux.get("gpio", [])],
                    "uart": [[name(p) for p in pins] for pins in aux.get("uart", [])]}
        self.add_constant("SLOTS", len(slots))
        self.add_constant("SLOTS_JTAG", sum(1 << i for i, where in enumerate(slots) if is_jtag(where)))
        self.add_constant("SLOTS_BASE", SLOTS_BASE)
        self.add_constant("SLOT_BYTES", SLOT_BYTES)
        self.add_constant("SLOT_CLK_FREQ", int(slot_clk_freq))
        self.add_constant("UART_RX_DEPTH", UART_RX_DEPTH)
        self.add_constant("UART_TX_DEPTH", UART_TX_DEPTH)

        if sim:
            self.sim_ios = {self.udp_rx.valid, self.udp_rx.ready, self.udp_rx.data, self.udp_rx.last,
                            self.udp_rx.ip_address, self.udp_rx.src_port, self.udp_rx.dst_port,
                            self.udp_rx.length,
                            self.udp_tx.valid, self.udp_tx.ready, self.udp_tx.data, self.udp_tx.last,
                            self.udp_tx.ip_address, self.udp_tx.src_port, self.udp_tx.dst_port,
                            self.udp_tx.length}
            for i, where in enumerate(slots):
                s = getattr(self, f"slot{i}")
                pins = ([(s.tck_pair, "tck"), (s.tms_pair, "tms"), (s.tdi_pair, "tdi"), (s.tdo_pair, "tdo")]
                        if is_jtag(where) else
                        [(s.swclk_pair, "swclk"), (s.swdo_pair, "swdo"), (s.swwr_pair, "swwr"),
                         (s.swdi_pair, "swdi")])
                for sig, nm in pins + [(s.nrst, "nrst"), (s.uart_pads.tx, "uart_tx"), (s.uart_pads.rx, "uart_rx")]:
                    sig.name_override = f"slot{i}_{nm}"
                    self.sim_ios.add(sig)
            for sig, nm in [(self.udp_rx, "rx"), (self.udp_tx, "tx")]:
                for field in ("valid", "ready", "data", "last", "ip_address", "src_port", "dst_port",
                              "length"):
                    getattr(sig, field).name_override = f"udp_{nm}_{field}"


def write_headers(soc, directory):
    """The firmware's view of the SoC: register addresses and constants."""
    os.makedirs(directory, exist_ok=True)
    csr_base = soc.mem_regions["csr"].origin
    # The firmware reaches slot n's registers at slot 0's plus n strides (firmware/hw.h)
    bases = []
    while f"slot{len(bases)}" in soc.csr_regions:
        bases.append(soc.csr_regions[f"slot{len(bases)}"].origin)
    strides = {b - a for a, b in zip(bases, bases[1:])}
    assert len(strides) <= 1, f"slot registers not evenly spaced: {[hex(b) for b in bases]}"
    with open(os.path.join(directory, "csr.h"), "w") as f:
        f.write(export.get_csr_header(soc.csr_regions, soc.constants, csr_base=csr_base,
                                      with_access_functions=False))
    with open(os.path.join(directory, "soc.h"), "w") as f:
        f.write(export.get_soc_header(soc.constants, with_access_functions=False))
    with open(os.path.join(directory, "mem.h"), "w") as f:
        f.write(export.get_mem_header(soc.mem_regions))
    with open(os.path.join(directory, "rig.json"), "w") as f:
        json.dump(soc.rig, f, indent=1)
    with open(os.path.join(directory, "csr.json"), "w") as f:
        f.write(export.get_csr_json(csr_regions=soc.csr_regions, constants=soc.constants,
                                    mem_regions=soc.mem_regions))


def firmware_words(path):
    with open(path, "rb") as f:
        data = f.read()
    data += bytes(-len(data) % 4)
    return list(struct.unpack(f"<{len(data) // 4}I", data))


def build_firmware(generated, out, march):
    subprocess.run(["make", "-C", os.path.join(ROOT, "firmware"), f"GENERATED={generated}",
                    f"BUILD={out}", f"MARCH={march}"], check=True)
    return os.path.join(out, "firmware.bin")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--build", action="store_true", help="build firmware and bitstream")
    parser.add_argument("--load", action="store_true", help="load the bitstream into SRAM")
    parser.add_argument("--sim", action="store_true", help="Verilog of the simulation's SoC")
    parser.add_argument("--layout", default="ci", choices=sorted(LAYOUTS))
    parser.add_argument("--ip", default="10.42.0.50")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--toolchain", default="vivado", choices=["vivado", "openxc7"],
                        help="vivado (default): built as an XC7A50T, then given the 35T's IDCODE")
    parser.add_argument("--cpu-variant", default=CPU_VARIANT, help="VexRiscv's: minimal, lite, standard")
    parser.add_argument("--targets", default="", help="--load: slots with an RP2040, for the self-check")
    parser.add_argument("--sys-clk-freq", type=float, default=SYS_CLK_FREQ,
                        help="the CPU's, the bus's and the network side's clock")
    parser.add_argument("--slot-clk-freq", type=float, default=SLOT_CLK_FREQ,
                        help="the engines' clock; SWCLK reaches half of it")
    args = parser.parse_args()

    variant = "sim" if args.sim else args.layout if args.seed == 1 else f"{args.layout}_s{args.seed}"
    if args.slot_clk_freq != SLOT_CLK_FREQ:
        variant += f"_{int(args.slot_clk_freq / 1e6)}mhz"
    if args.sys_clk_freq != SYS_CLK_FREQ:
        variant += f"_sys{int(args.sys_clk_freq / 1e6)}"
    if args.cpu_variant != CPU_VARIANT:
        variant += f"_{args.cpu_variant}"
    if args.toolchain != "vivado":
        variant += f"_{args.toolchain}"
    out = os.path.join(ROOT, "build", f"slots_{variant}")
    generated = os.path.join(out, "generated")

    # First the register map, for the firmware; then the SoC again with the firmware in its RAM
    soc = ArtySlots(args.layout, sim=args.sim, ip_address=args.ip, slot_clk_freq=args.slot_clk_freq,
                    sys_clk_freq=args.sys_clk_freq, cpu_variant=args.cpu_variant,
                    toolchain=args.toolchain)
    soc.finalize()
    write_headers(soc, generated)
    fw = build_firmware(generated, os.path.join(out, "firmware"),
                        re.search(r"-march=(\S+)", soc.cpu.gcc_flags)[1])
    soc = ArtySlots(args.layout, firmware=firmware_words(fw), sim=args.sim, ip_address=args.ip,
                    slot_clk_freq=args.slot_clk_freq, sys_clk_freq=args.sys_clk_freq,
                    cpu_variant=args.cpu_variant, toolchain=args.toolchain)

    if args.sim:
        soc.finalize()
        conv = convert(soc, ios=soc.platform.constraint_manager.get_io_signals() | soc.sim_ios,
                       name="sim_top", platform=soc.platform)
        with open(os.path.join(out, "sim_top.v"), "w") as f:
            f.write(conv.main_source)
        for name, content in conv.data_files.items():
            with open(os.path.join(out, name), "w") as f:
                f.write(content)
        with open(os.path.join(out, "sources.txt"), "w") as f:
            f.write("\n".join(src for src, *_ in soc.platform.sources) + "\n")
        print(os.path.join(out, "sim_top.v"))
        return

    if args.build or args.load:
        gateware = os.path.join(out, "gateware")
        if args.build and args.toolchain == "vivado":
            soc.build(build_dir=gateware, build_name="arty_slots")
            os.replace(os.path.join(gateware, "arty_slots.bit"), os.path.join(gateware, "arty_slots_50t.bit"))
            subprocess.run([sys.executable, os.path.join(ROOT, "scripts", "bitpatch.py"),
                            os.path.join(gateware, "arty_slots_50t.bit"), os.path.join(gateware, "arty_slots.bit"),
                            "--idcode", "0x0362D093"], check=True)
        if args.toolchain == "vivado" and (args.build or args.load):
            # Vivado writes the bitstream whether or not timing was met
            timing = subprocess.run([sys.executable, os.path.join(ROOT, "scripts", "timing_check.py"),
                                     os.path.join(gateware, "arty_slots_timing.rpt")])
            if timing.returncode:
                sys.exit("timing not met: do not use this bitstream")
        elif args.build:
            soc.build(build_dir=gateware, build_name="arty_slots", seed=args.seed)
        if args.toolchain == "openxc7":
            # An unpatched nextpnr leaves BUFGCTRLs it routes through unconfigured (patches/nextpnr)
            bufg = subprocess.run([sys.executable, os.path.join(ROOT, "scripts", "bufg_check.py"),
                                   os.path.join(gateware, "arty_slots.fasm")])
            if bufg.returncode:
                sys.exit("unconfigured BUFGCTRL on a global line: this bitstream is broken")
        if args.load:
            subprocess.run(["openFPGALoader", "-b", "arty", os.path.join(gateware, "arty_slots.bit")],
                           check=True)
            # A bitstream is used only once scripts/self_check.py passes on the board
            time.sleep(3)
            check = subprocess.run([sys.executable, os.path.join(ROOT, "scripts", "self_check.py"), out,
                                    "--targets", args.targets])
            if check.returncode:
                sys.exit("self-check failed: do not use this bitstream (build another seed)")


if __name__ == "__main__":
    main()
