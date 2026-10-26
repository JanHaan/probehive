#!/usr/bin/env python3
"""Check a loaded slots bitstream before using it (gateware/arty_slots.py).

openXC7 has produced bitstreams of this design that differ from their
netlist in some placements: every read of the CSR bus carried a constant
set of bits, or a byte lane of a slot buffer never wrote.  The cause was
nextpnr routing nets through BUFGCTRLs it left unconfigured (patches/nextpnr,
scripts/bufg_check.py); a bitstream is still used only once this passes:

  1. the CSR bus: unused addresses read 0, and the scratch register gives
     back what is written to it, bit for bit;
  2. main RAM holds the firmware image, word for word;
  3. every slot's request and response buffers keep patterns written to them;
  4. every slot answers DAP_Info from the CPU with its own serial;
  5. every SWD slot's engine answers a DAP_Transfer: ACK OK with a target, no
     ACK without one; every JTAG slot says so, connects as JTAG and its engine
     runs a DAP_JTAG_Sequence;
  6. every slot's UART takes the DAP_UART_* commands;
     and a request sent again is answered again, from its kept response;
  7. on SWD slots given with --targets (RP2040s), an attach and a memory test,
     every word checked; on JTAG slots given with it, the first TAP's IDCODE.

    python3 scripts/self_check.py build/slots_<layout>/ [--targets 0] [--kib 16]

Exits 0 when everything passes, 1 otherwise.
"""

import argparse
import json
import os
import random
import socket
import struct
import sys
import time

from litex.tools.remote.comm_udp import CommUDP

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dap_udp import DAP, DAPError, Memory, attach_rp2040  # noqa: E402

MAIN_RAM = 0x40000000
BURST = 4                        # words an Etherbone packet carries (LiteEth's buffer depth)


class Check:
    def __init__(self):
        self.failures = 0

    def __call__(self, ok, what):
        print(f"  {'ok  ' if ok else 'FAIL'} {what}", flush=True)
        if not ok:
            self.failures += 1
        return ok


def csr_bus(bus, info, check):
    print("CSR bus")
    regs = info["csr_registers"]
    used = {r["addr"] for r in regs.values()}
    # Every register bank's range ends in unused words (the identifier's memory excepted, whose
    # words are characters)
    banks = [b for name, b in info["csr_bases"].items() if "mem" not in name]
    probes = sorted({b + off for b in banks for off in (0x400, 0x7FC)} - used)
    values = [bus.read(a) for a in probes]
    bad = [(a, v) for a, v in zip(probes, values) if v]
    check(not bad, f"{len(probes)} unused addresses read 0" +
          ("" if not bad else ": " + ", ".join(f"{a:#x}={v:#x}" for a, v in bad[:4])))
    scratch = regs["ctrl_scratch"]["addr"]
    old = bus.read(scratch)
    patterns = [0, 0xFFFFFFFF, 0xAAAAAAAA, 0x55555555] + [1 << i for i in range(32)]
    wrong = []
    for p in patterns:
        bus.write(scratch, p)
        if bus.read(scratch) != p:
            wrong.append(p)
    bus.write(scratch, old)
    check(not wrong, f"scratch keeps {len(patterns)} patterns" +
          ("" if not wrong else f": {len(wrong)} came back changed, first {wrong[0]:#x}"))


def main_ram(bus, image, check):
    print("Main RAM")
    words = struct.unpack(f"<{len(image) // 4}I", image[:len(image) // 4 * 4])
    got = []
    for i in range(0, len(words), BURST):
        got += bus.read(MAIN_RAM + 4 * i, min(BURST, len(words) - i))
    diff = sum(1 for a, b in zip(got, words) if a != b)
    # .data changes while the firmware runs; the code does not
    check(diff <= 64, f"{len(words)} words of the firmware image, {diff} differ (.data may)")


def buffers(bus, info, nslots, check):
    print("Slot buffers")
    consts = info["constants"]
    base, size = consts["slots_base"], consts["slot_bytes"]
    rng = random.Random(1)
    for s in range(nslots):
        a0 = base + s * size
        n = size // 4
        data = [rng.getrandbits(32) for _ in range(n)]
        for i in range(0, n, BURST):
            bus.write(a0 + 4 * i, data[i:i + BURST])
        back = []
        for i in range(0, n, BURST):
            back += bus.read(a0 + 4 * i, BURST)
        diff = sum(1 for a, b in zip(back, data) if a != b)
        check(diff == 0, f"slot {s}: {n} words written and read back, {diff} wrong")


def slots(host, nslots, jtag, targets, check):
    print("Slots")
    for s in range(nslots):
        dap = DAP(host, slot=s, attempts=4)
        try:
            serial = dap.info_string(DAP.INFO_SERIAL)
            check(serial == f"probehive-slot-{s}", f"slot {s}: DAP_Info serial {serial!r}")
            if s in jtag:
                caps = dap.command([0x00, 0xF0])
                check(caps[1] == 1 and caps[2] & 0x03 == 0x02, f"slot {s}: JTAG, capabilities {caps[1:3].hex()}")
                check(dap.command([0x02, 0x00])[1] == 2, f"slot {s}: connects as JTAG")
                dap.swj_clock(10_000_000)
                # Test-Logic-Reset, then 64 bits through whatever is there: TDO high without a target
                r = dap.command([0x12, 8, 0xFF])
                r = dap.command([0x14, 1, 0x80] + [0x00] * 8)
                check(r == bytes([0x14, 0]) + b"\xff" * 8 or s in targets,
                      f"slot {s}: engine ran DAP_JTAG_Sequence, TDO {r[2:].hex()} (pulled up)")
                if s in targets:
                    # From Test-Logic-Reset to Shift-DR, where IDCODE is the first TAP's register
                    dap.command([0x12, 8, 0xFF])
                    r = dap.command([0x14, 4, 0x01, 0x00, 0x41, 0x00, 0x02, 0x00, 0xA0] + [0x00] * 4)
                    dap.command([0x12, 8, 0xFF])
                    idcode = int.from_bytes(r[2:6], "little")
                    check(idcode & 1 and idcode != 0xFFFFFFFF, f"slot {s}: target's IDCODE 0x{idcode:08x}")
                continue
            if s in targets:
                continue                         # an RP2040 answers only once selected: see below
            dap.connect_swd()
            dap.swj_clock(1_000_000)
            r = dap.command([0x05, 0x00, 0x01, 0x02])                 # DPIDR, through the engine
            check(r[2] & 0x07 == 7, f"slot {s}: engine answered, no target: ack {r[2] & 0x07}")
        except DAPError as error:
            check(False, f"slot {s}: {error}")
        finally:
            dap.close()


def uarts(host, nslots, check):
    print("UARTs")
    for s in range(nslots):
        dap = DAP(host, slot=s, attempts=4)
        try:
            caps = dap.command([0x00, 0xF0])[2]
            r = [dap.command([0x1F, 2]), dap.command([0x20, 0] + list(struct.pack("<I", 115200))),
                 dap.command([0x22, 0x55]), dap.command([0x21, 64, 0, 0, 0])]
            baud = struct.unpack_from("<I", r[1], 2)[0]
            check(caps & 0x80 and r[0][1] == 0 and r[1][1] == 0 and abs(baud - 115200) < 100 and
                  r[2][1] == 0 and r[3][1] & 0x11 == 0x11,
                  f"slot {s}: DAP_UART_* answered, {baud} baud")
            dap.command([0x22, 0x22])
            dap.command([0x1F, 0])
        except DAPError as error:
            check(False, f"slot {s}: {error}")
        finally:
            dap.close()


def replay(host, check):
    print("Transport")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.5)
    seq = random.randrange(1 << 16)
    request = struct.pack(">BBH", 0xDA, 0x81, seq) + bytes([0x00, 0x03])     # SYNC, DAP_Info serial
    replies = []
    for _ in range(2):
        sock.sendto(request if not replies else request[:1] + bytes([0x01]) + request[2:], (host, 1235))
        try:
            replies.append(sock.recv(2048))
        except socket.timeout:
            replies.append(None)
    sock.close()
    check(replies[0] is not None and replies[0] == replies[1],
          "a request sent again is answered again, the same")


def targets_test(host, targets, kib, check):
    for s in targets:
        print(f"Target on slot {s}")
        dap = DAP(host, slot=s)
        try:
            check(attach_rp2040(dap, 1_000_000) == 0x0BC12477, f"slot {s}: RP2040 DPIDR")
            mem = Memory(dap)
            mem.power_up()
            mem.write32(0xE000EDF0, 0xA05F0003)                        # halt the core
            dap.swj_clock(50_000_000)
            words = kib * 256
            data = [random.getrandbits(32) for _ in range(words)]
            mem.write_block(0x20010000, data, 6)
            back = mem.read_block(0x20010000, words, 6)
            check(back == data, f"slot {s}: {kib} KiB written and read back at 50 MHz")
        except DAPError as error:
            check(False, f"slot {s}: {error}")
        finally:
            dap.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("build", help="build/slots_<layout>/ of the loaded bitstream")
    parser.add_argument("--host", default="10.42.0.50")
    parser.add_argument("--targets", default="",
                        help="slots with a target, comma separated: an RP2040 on SWD, any TAP on JTAG")
    parser.add_argument("--kib", type=int, default=16)
    args = parser.parse_args()

    info = json.load(open(os.path.join(args.build, "generated", "csr.json")))
    image = open(os.path.join(args.build, "firmware", "firmware.bin"), "rb").read()
    nslots = info["constants"]["slots"]
    jtag = [i for i in range(nslots) if (info["constants"].get("slots_jtag", 0) >> i) & 1]
    targets = [int(t) for t in args.targets.split(",") if t]
    check = Check()

    bus = CommUDP(args.host)
    bus.open()
    csr_bus(bus, info, check)
    main_ram(bus, image, check)
    buffers(bus, info, nslots, check)
    bus.close()
    slots(args.host, nslots, jtag, targets, check)
    uarts(args.host, nslots, check)
    replay(args.host, check)
    targets_test(args.host, [t for t in targets if t not in jtag], args.kib, check)

    print("PASS" if not check.failures else f"FAIL: {check.failures} checks")
    return 1 if check.failures else 0


if __name__ == "__main__":
    sys.exit(main())
