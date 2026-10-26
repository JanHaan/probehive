#!/usr/bin/env python3
"""The slots' SoC, CPU and firmware included, against simulated targets, checked byte for byte.

gateware/arty_slots.py --sim gives the SoC with the UDP port's streams and the slots' DDR pairs
as plain ports; this runs it under Icarus Verilog with a target (sim/slot_target.v) behind each
slot's pins, a round trip away, and sends it datagrams in rounds: every datagram of a round goes
in back to back, then the bench waits for the replies.  The replies are compared with what the
firmware should answer: the commands for the wire through sim/slot_sim.py's port of DAP.c
against a model of each slot's target, the others as the firmware words them.

    python3 sim/soc_sim.py [--slots 4] [--round-trip 20] [--seed 1] [--random 6] [--vcd]

Checked besides the answers: a request sent again is answered again from its kept response, and
not run twice; one out of sequence is not answered; the slots run side by side.
"""

import argparse
import json
import os
import random
import re
import struct
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
sys.path.insert(0, HERE)
from slot_sim import (AP_R, AP_W, DP_R, DP_W, MATCH_MASK, MATCH_VALUE, Reference, Target,  # noqa: E402
                      block_read, block_write, transfer)
import jtag_sim  # noqa: E402

OUT = os.path.join(ROOT, "build", "slots_sim")
MAGIC, VERSION, SYNC = 0xDA, 0x01, 0x80
ENTRIES, MAX_PACKET = 6, 1032


def datagram(slot, seq, packet, sync=False):
    return struct.pack(">BBH", MAGIC, (SYNC if sync else 0) | slot << 4 | VERSION, seq) + bytes(packet)


def info_string(s):
    return bytes([len(s) + 1]) + s.encode() + b"\0"


UART_RX_DEPTH, UART_TX_DEPTH, SYS_HZ = 512, 64, 50_000_000


class Uart:
    """A slot's UART with its TX looped back to its RX, as the testbench wires it.  Bytes sent have
    come in by the next DAP_Delay: the scripts read only after one."""

    def __init__(self):
        self.dap = self.configured = self.rx_on = self.tx_on = False
        self.rx = bytearray()

    def status(self):
        return (0x01 if self.rx_on else 0) | (0x10 if self.tx_on else 0)

    def command(self, cmd):
        c = cmd[0]
        if c == 0x1F:
            if cmd[1] == 0:
                self.__init__()
            elif cmd[1] == 2:
                self.dap = True
            return bytes([0x1F, 0x00 if cmd[1] in (0, 2) else 0xFF]), 2
        if c == 0x20:
            control, baud = cmd[1], struct.unpack_from("<I", cmd, 2)[0]
            status = 0x07 if not self.dap else ((0x01 if control & 0x0F else 0) |
                                                (0x02 if control & 0x30 else 0) | (0x04 if control & 0xC0 else 0))
            if status or not 300 <= baud <= SYS_HZ // 16:
                baud, self.configured = 0, False
            else:
                k = ((1 << 40) + SYS_HZ // 2) // SYS_HZ
                tuning = (baud * k) >> 8
                baud, self.configured = (tuning * SYS_HZ + (1 << 31)) >> 32, True
            return bytes([0x20, status]) + struct.pack("<I", baud), 6
        if c == 0x22:
            control, ok = cmd[1], self.dap
            if self.dap:
                if control & 0x02:
                    self.rx_on = False
                elif control & 0x01:
                    if not self.configured:
                        ok = False
                    elif not self.rx_on:
                        self.rx_on, self.rx = True, bytearray()
                if control & 0x04:
                    self.rx = bytearray()
                if control & 0x20:
                    self.tx_on = False
                elif control & 0x10:
                    if not self.configured:
                        ok = False
                    else:
                        self.tx_on = True
            return bytes([0x22, 0x00 if ok else 0xFF]), 2
        if c == 0x23:
            on = self.dap and self.configured
            return bytes([0x23, self.status() if on else 0]) + struct.pack("<II", len(self.rx) if on else 0, 0), 1
        if c == 0x21:
            rx_n, tx_ask = struct.unpack_from("<HH", cmd, 1)
            data = cmd[5:5 + tx_ask]
            if not self.dap:
                return bytes([0x21, 0, 0, 0, 0, 0]), 5 + tx_ask
            got = bytes(self.rx[:rx_n])
            del self.rx[:len(got)]
            tx_n = min(tx_ask, UART_TX_DEPTH)
            if self.tx_on and self.rx_on:
                self.rx += data[:tx_n]
            return bytes([0x21, self.status()]) + struct.pack("<HH", tx_n, len(got)) + got, 5 + tx_ask
        return None


def uart_script(slot):
    """A slot's UART: refused before its transport is chosen, then a line looped back."""
    text = f"hello slot {slot}".encode()
    transfer = lambda rx, tx=b"": bytes([0x21]) + struct.pack("<HH", rx, len(tx)) + tx
    return [
        transfer(16, b"early"),                                              # no transport yet
        bytes([0x1F, 0x02]),                                                 # DAP_UART_Transport
        bytes([0x20, 0x01]) + struct.pack("<I", 115200),                     # 7 data bits: refused
        bytes([0x20, 0x00]) + struct.pack("<I", 3_000_000),                 # 8N1, 3 Mbaud
        bytes([0x22, 0x11]),                                                 # receive and send
        bytes([0x7F, 3]) + transfer(0, text) + bytes([0x09, 100, 0]) + transfer(64),
        bytes([0x23]),
        bytes([0x22, 0x22]), bytes([0x1F, 0x00]),
    ]


class Firmware:
    """What the firmware answers, the wire's commands through the reference."""

    def __init__(self, slot, retry=100, match_retry=0):
        self.slot = slot
        self.ref = Reference(Target(), retry, match_retry)
        self.uart = Uart()

    def info(self, ident):
        return {0x01: info_string("probehive"), 0x02: info_string("probehive CMSIS-DAP"),
                0x03: info_string(f"probehive-slot-{self.slot}"), 0x04: info_string("2.1.1"),
                0x09: info_string("slots-1"), 0xF0: bytes([1, 0x81]), 0xFE: bytes([1, ENTRIES]),
                0xFB: bytes([4]) + struct.pack("<I", UART_RX_DEPTH),
                0xFC: bytes([4]) + struct.pack("<I", UART_TX_DEPTH),
                0xFF: bytes([2]) + struct.pack("<H", MAX_PACKET)}.get(ident, bytes([0]))

    def one(self, cmd):
        """One command's response and length."""
        c = cmd[0]
        if 0x1F <= c <= 0x23:
            return self.uart.command(cmd)
        if c in (0x05, 0x06, 0x08, 0x12, 0x1D):
            return self.ref.run(cmd)
        if c == 0x00:
            return bytes([0x00]) + self.info(cmd[1]), 2
        if c == 0x01:
            return bytes([0x01, 0x00]), 3
        if c == 0x02:
            return bytes([0x02, 1 if cmd[1] in (0, 1) else 0]), 2
        if c == 0x03:
            return bytes([0x03, 0x00]), 1
        if c == 0x04:
            self.ref.retry = struct.unpack_from("<H", cmd, 2)[0]
            self.ref.match_retry = struct.unpack_from("<H", cmd, 4)[0]
            return bytes([0x04, 0x00]), 6
        if c == 0x09:
            return bytes([0x09, 0x00]), 3
        if c == 0x0A:
            return bytes([0x0A, 0x00, 0x00]), 1
        if c == 0x11:
            return bytes([0x11, 0x00]), 5
        if c == 0x13:
            return bytes([0x13, 0x00]), 2
        return bytes([0xFF]), None

    def packet(self, packet):
        if packet[0] == 0x7F:
            out, p = bytes([0x7F, packet[1]]), 2
            for _ in range(packet[1]):
                r, n = self.one(packet[p:])
                out += r
                if n is None:
                    break
                p += n
            return out
        return self.one(packet)[0]


class JtagFirmware(Firmware):
    """What the firmware answers on a JTAG slot: the wire's commands through jtag_sim's reference."""

    def __init__(self, slot):
        self.slot = slot
        self.ref = jtag_sim.Reference(jtag_sim.Tap())
        self.uart = Uart()

    def info(self, ident):
        return bytes([1, 0x82]) if ident == 0xF0 else super().info(ident)

    def one(self, cmd):
        c = cmd[0]
        if 0x1F <= c <= 0x23:
            return self.uart.command(cmd)
        if c in (0x12, 0x14):
            return self.ref.run(cmd)
        if c == 0x02:
            return bytes([0x02, 2 if cmd[1] in (0, 2) else 0]), 2
        if c == 0x04:
            return bytes([0x04, 0x00]), 6
        if c == 0x15:
            return bytes([0x15, 0x00]), 2 + cmd[1]
        if c == 0x16:
            return bytes([0x16, 0xFF, 0, 0, 0, 0]), 2
        if c in (0x05, 0x06, 0x08, 0x1D):
            return bytes([0xFF]), None
        return super().one(cmd)


def jtag_slot_script(rng, slot, randoms):
    """A JTAG slot's packets: connect, then work its TAP."""
    j = jtag_sim
    s = [
        bytes([0x00, 0xF0]), bytes([0x00, 0x03]),                            # DAP_Info
        bytes([0x02, 0x01]), bytes([0x02, 0x00]), bytes([0x02, 0x02]),     # DAP_Connect SWD, default, JTAG
        bytes([0x11]) + struct.pack("<I", 20_000_000),                      # DAP_SWJ_Clock
        bytes([0x15, 1, 5]),                                                 # DAP_JTAG_Configure
        j.swj([1] * 8), j.jtag((0, [1], 0)),                                 # Test-Logic-Reset, Idle
        j.dr_scan(0),                                                        # IDCODE
        bytes([0x16, 0]),                                                    # DAP_JTAG_IDCODE
        # Through the CPU: the sequence inside DAP_ExecuteCommands
        bytes([0x7F, 3, 0x00, 0x04]) + j.ir_scan(j.I_SCRATCH) + j.dr_scan(0xC0DE0000 | slot),
        j.dr_scan(rng.getrandbits(32)),                                      # the word back
        bytes([0x05, 0x00, 0x01, 0x02]),                                     # DAP_Transfer: not here
        bytes([0x09, 0x08, 0x00]),                                           # DAP_Delay 8 us
    ]
    for _ in range(randoms):
        if rng.random() < 0.2:
            s.append(j.swj([rng.getrandbits(1) for _ in range(rng.randint(1, 255))]))
        else:
            s.append(j.jtag(*[(rng.getrandbits(1), [rng.getrandbits(1) for _ in range(rng.randint(1, 64))],
                               rng.getrandbits(1)) for _ in range(rng.randint(1, 6))]))
    return s


def slot_script(rng, slot, randoms):
    """A slot's packets: attach, then work its target."""
    s = [
        bytes([0x00, 0x04]), bytes([0x00, 0xFF]), bytes([0x00, 0x03]),     # DAP_Info
        bytes([0x02, 0x01]),                                                 # DAP_Connect SWD
        bytes([0x11]) + struct.pack("<I", 50_000_000),                      # DAP_SWJ_Clock
        bytes([0x04, 0x00]) + struct.pack("<HH", 100, 3),                   # DAP_TransferConfigure
        bytes([0x12, 56]) + bytes([0xFF] * 6 + [0x03]),                     # line reset
        transfer((DP_R(0), None)),                                           # DPIDR
        transfer((DP_W(1), 0x50000000 | slot), (DP_W(2), 0), (AP_W(0), 0x23000012),
                 (AP_W(1), 0x100 + 0x40 * slot), (AP_R(3), None), (AP_R(3), None), (DP_R(1), None)),
        # Two commands in one packet
        bytes([0x7F, 2, 0x01, 0x00, 0x01]) + transfer((AP_W(1), 0x40 * slot),),
        block_read(AP_R(3), 64),
        transfer((AP_W(1), 0x400 + 0x100 * slot),),
        block_write(AP_W(3), [0xC0DE0000 + 0x1000 * slot + i for i in range(200)]),
        transfer((AP_W(1), 0x400 + 0x100 * slot),),
        block_read(AP_R(3), 200),
        transfer((DP_W(1) | MATCH_MASK, 0x0000FFFF), (DP_W(1), 0x12340000 | slot),
                 (DP_R(1) | MATCH_VALUE, slot)),
        bytes([0x09, 0x10, 0x00]),                                           # DAP_Delay 16 us
        bytes([0x42]),                                                       # not a command
    ]
    for _ in range(randoms):
        reqs = []
        for _ in range(rng.randrange(1, 10)):
            k = rng.randrange(5)
            if k == 0:
                reqs.append((AP_W(1), rng.randrange(0, 256) * 4))
            elif k == 1:
                reqs.append((AP_W(3), rng.getrandbits(32)))
            elif k in (2, 3):
                reqs.append((AP_R(3), None))
            else:
                reqs.append((DP_R(1), None))
        s.append(transfer(*reqs))
    return s


TESTBENCH = r"""
`timescale 1ns/1ps
module tb;
    reg sys_clk = 0, slot_clk = 0, rst = 1;
    always #%(sys_half)s sys_clk = ~sys_clk;                                   // the SoC's sys clock
    always #5 slot_clk = ~slot_clk;                                         // 100 MHz, aligned

    reg  [7:0] rx_data = 0; reg rx_valid = 0, rx_last = 0; wire rx_ready;
    wire [7:0] tx_data; wire tx_valid, tx_last; wire [15:0] tx_length, tx_dst_port, tx_src_port;
    wire [31:0] tx_ip; wire serial_tx;
%(slot_wires)s
    sim_top dut(.sim_clocks_sys(sys_clk), .sim_clocks_slot(slot_clk), .sim_clocks_rst(rst),
        .serial_rx(1'b1), .serial_tx(serial_tx),
        .udp_rx_data(rx_data), .udp_rx_valid(rx_valid), .udp_rx_last(rx_last), .udp_rx_ready(rx_ready),
        .udp_rx_ip_address(32'h0A2A0001), .udp_rx_src_port(16'd4000), .udp_rx_dst_port(16'd1235),
        .udp_rx_length(16'd0),
        .udp_tx_data(tx_data), .udp_tx_valid(tx_valid), .udp_tx_last(tx_last), .udp_tx_ready(1'b1),
        .udp_tx_ip_address(tx_ip), .udp_tx_src_port(tx_src_port), .udp_tx_dst_port(tx_dst_port),
        .udp_tx_length(tx_length)%(slot_ports)s);
%(slot_targets)s
    // Replies, a line each
    // A reply is kept until its last byte and written then, in one go, so that the targets' lines
    // cannot land in it
    integer replies = 0, tx_n = 0, tx_i;
    reg [7:0] tx_buf [0:2047];
    always @(posedge sys_clk) if (tx_valid) begin
        tx_buf[tx_n] = tx_data;
        tx_n = tx_n + 1;
        if (tx_n == 1) $display("TXSTART %%0t", $time);
        if (tx_last) begin
            $write("TX %%0t", $time);
            for (tx_i = 0; tx_i < tx_n; tx_i = tx_i + 1) $write(" %%02x", tx_buf[tx_i]);
            $write("\n");
            replies = replies + 1;
            tx_n = 0;
        end
    end

    always @(posedge sys_clk) if (dut.dbus_cyc && dut.dbus_stb && dut.dbus_ack) $display("DB %%0t %%h %%0d", $time, dut.dbus_adr, dut.dbus_we);
    always @(posedge sys_clk) if (dut.ibus_cyc && dut.ibus_stb && dut.ibus_ack) $display("IB %%0t %%h", $time, dut.ibus_adr);
    // The console, at 10 Mbaud: five sys cycles a bit
    integer b; reg [7:0] ch;
    initial forever begin
        @(negedge serial_tx);
        #(150);
        for (b = 0; b < 8; b = b + 1) begin ch[b] = serial_tx; #100; end
        $display("U %%02x", ch);
    end

    reg [7:0] bytes [0:%(total)d];
    integer lengths [0:%(count)d];
    integer rounds [0:%(count)d];
    integer expect_after [0:%(count)d];
    integer p, i, n;
    initial begin
        %(vcd)s
        $readmemh("%(bytes)s", bytes);
        $readmemh("%(lengths)s", lengths);
        $readmemh("%(rounds)s", rounds);
        $readmemh("%(expects)s", expect_after);
        repeat (20) @(posedge sys_clk);
        rst = 0;
        repeat (2000) @(posedge sys_clk);
        p = 0;
        for (n = 0; n < %(count)d; n = n + 1) begin
            for (i = 0; i < lengths[n]; i = i + 1) begin
                @(negedge sys_clk);
                rx_valid = 1; rx_data = bytes[p]; rx_last = (i == lengths[n] - 1);
                if (rx_last) $display("RXEND %%0t %%0d", $time, n);
                p = p + 1;
            end
            @(negedge sys_clk); rx_valid = 0; rx_last = 0;
            repeat (4) @(negedge sys_clk);
            // The end of a round: wait for its replies
            if (n == %(count)d - 1 || rounds[n + 1] != rounds[n]) begin
                $display("R %%0t %%0d", $time, rounds[n]);
                while (replies < expect_after[n]) @(posedge sys_clk);
                repeat (3000) @(posedge sys_clk);
            end
        end
        $display("END %%0t", $time);
        $finish;
    end
    initial begin #%(timeout)d $display("TIMEOUT %%0t", $time); $finish; end
endmodule
"""

SLOT_WIRES = """    wire [1:0] s{i}_clk, s{i}_do, s{i}_wr, s{i}_di; wire s{i}_nrst, u{i};
"""
JTAG_WIRES = """    wire [1:0] s{i}_tck, s{i}_tms, s{i}_tdi, s{i}_tdo; wire s{i}_nrst, u{i};
"""
JTAG_PORTS = """,
        .slot{i}_tck(s{i}_tck), .slot{i}_tms(s{i}_tms), .slot{i}_tdi(s{i}_tdi), .slot{i}_tdo(s{i}_tdo),
        .slot{i}_nrst(s{i}_nrst), .slot{i}_uart_tx(u{i}), .slot{i}_uart_rx(u{i})"""
JTAG_TARGET = """
    // Slot {i}'s pins, through DDR registers, and its TAP a round trip away
    wire p{i}_tck, p{i}_tms, p{i}_tdi, t{i}_tdo;
    ODDR o{i}c(.Q(p{i}_tck), .C(slot_clk), .CE(1'b1), .D1(s{i}_tck[0]), .D2(s{i}_tck[1]), .R(1'b0), .S(1'b0));
    ODDR o{i}m(.Q(p{i}_tms), .C(slot_clk), .CE(1'b1), .D1(s{i}_tms[0]), .D2(s{i}_tms[1]), .R(1'b0), .S(1'b0));
    ODDR o{i}i(.Q(p{i}_tdi), .C(slot_clk), .CE(1'b1), .D1(s{i}_tdi[0]), .D2(s{i}_tdi[1]), .R(1'b0), .S(1'b0));
    reg o{i}_late = 1;
    always @(t{i}_tdo) o{i}_late <= #({rt}) t{i}_tdo;
    IDDR i{i}o(.Q1(s{i}_tdo[0]), .Q2(s{i}_tdo[1]), .C(slot_clk), .CE(1'b1), .D(o{i}_late), .R(1'b0), .S(1'b0));
    jtag_target target{i}(.tck(p{i}_tck), .tms(p{i}_tms), .tdi(p{i}_tdi), .tdo(t{i}_tdo));
"""
SLOT_PORTS = """,
        .slot{i}_swclk(s{i}_clk), .slot{i}_swdo(s{i}_do), .slot{i}_swwr(s{i}_wr), .slot{i}_swdi(s{i}_di),
        .slot{i}_nrst(s{i}_nrst), .slot{i}_uart_tx(u{i}), .slot{i}_uart_rx(u{i})"""
SLOT_TARGET = """
    // Slot {i}'s pins, through DDR registers, and its target a round trip away
    wire p{i}_clk, p{i}_do, p{i}_wr, p{i}_di;
    ODDR o{i}c(.Q(p{i}_clk), .C(slot_clk), .CE(1'b1), .D1(s{i}_clk[0]), .D2(s{i}_clk[1]), .R(1'b0), .S(1'b0));
    ODDR o{i}d(.Q(p{i}_do), .C(slot_clk), .CE(1'b1), .D1(s{i}_do[0]), .D2(s{i}_do[1]), .R(1'b0), .S(1'b0));
    ODDR o{i}w(.Q(p{i}_wr), .C(slot_clk), .CE(1'b1), .D1(s{i}_wr[0]), .D2(s{i}_wr[1]), .R(1'b0), .S(1'b0));
    IDDR i{i}d(.Q1(s{i}_di[0]), .Q2(s{i}_di[1]), .C(slot_clk), .CE(1'b1), .D(p{i}_di), .R(1'b0), .S(1'b0));
    wire t{i}_drive, t{i}_bit;
    reg d{i}_late = 0, b{i}_late = 1;
    always @(t{i}_drive) d{i}_late <= #({rt}) t{i}_drive;
    always @(t{i}_bit) b{i}_late <= #({rt}) t{i}_bit;
    assign p{i}_di = p{i}_wr ? p{i}_do : (d{i}_late ? b{i}_late : 1'b1);
    swd_target target{i}(.swclk(p{i}_clk), .line(p{i}_di), .drive(t{i}_drive), .out_bit(t{i}_bit));
"""


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--layout", default="ci", help="gateware/arty_slots.py's")
    parser.add_argument("--slots", type=int, default=8, help="slots given work (of the SoC's)")
    parser.add_argument("--round-trip", type=float, default=20)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--random", type=int, default=6)
    parser.add_argument("--vcd", action="store_true")
    parser.add_argument("--no-build", action="store_true", help="reuse build/slots_sim")
    parser.add_argument("--attach", action="store_true",
                        help="slot 0 attaches as scripts/dap_probe.py does, at --clock")
    parser.add_argument("--clock", type=int, default=1_000_000)
    args = parser.parse_args()

    if not args.no_build:
        subprocess.run([sys.executable, os.path.join(ROOT, "gateware", "arty_slots.py"), "--sim",
                        "--layout", args.layout], check=True, stdout=subprocess.DEVNULL)
    with open(os.path.join(OUT, "sim_top.v")) as f:
        top = f.read()
    jtag = {int(i) for i in re.findall(r"slot(\d+)_tck", top)}
    nslots = len(jtag | {int(i) for i in re.findall(r"slot(\d+)_swclk", top)})

    rng = random.Random(args.seed)
    used = min(args.slots, nslots)
    firmware = [JtagFirmware(i) if i in jtag else Firmware(i) for i in range(nslots)]
    scripts = [(jtag_slot_script if i in jtag else slot_script)(rng, i, args.random) +
               [bytes([0x00, 0xFB]), bytes([0x00, 0xFC])] + uart_script(i) for i in range(used)]
    if args.attach:
        sys.path.insert(0, os.path.join(ROOT, "scripts"))
        from dap_udp import DORMANT_TO_SWD, LINE_RESET, RP2040_CORE0, parity
        sel = struct.pack("<I", RP2040_CORE0) + bytes([parity(RP2040_CORE0)])
        scripts[0] = [
            bytes([0x02, 0x01]), bytes([0x11]) + struct.pack("<I", args.clock),
            bytes([0x04, 0x00]) + struct.pack("<HH", 64, 0), bytes([0x13, 0x00]),
            bytes([0x12, 224 & 0xFF]) + DORMANT_TO_SWD, bytes([0x12, 64]) + LINE_RESET,
            # TARGETSEL as raw sequences; what the input run reads is the target's, not modelled
            bytes([0x1D, 3, 8, 0x99, 0x80 | 5, 33]) + sel,
            transfer((DP_R(0), None)),
            transfer((DP_W(1), 0x50000000), (DP_R(1), None)),
        ]

    # Rounds: each slot's next packet, side by side; then the checks of the transport on slot 0
    plan = []                                  # (round, slot, seq, packet, sync, expected or None)
    seqs = [0] * nslots
    for r in range(max(len(s) for s in scripts)):
        for i in range(used):
            if r < len(scripts[i]):
                pkt = scripts[i][r]
                want = b"*" if pkt[0] == 0x1D and args.attach and i == 0 else firmware[i].packet(pkt)
                plan.append((r, i, seqs[i], pkt, r == 0, want))
                seqs[i] += 1
    r = plan[-1][0] + 1
    # Three in flight at once on slot 0, the window the firmware reports
    for k in range(ENTRIES):
        pkt = transfer((AP_R(3), None), (AP_R(3), None))
        plan.append((r, 0, seqs[0], pkt, False, firmware[0].packet(pkt)))
        seqs[0] += 1
    r += 1
    # The last one again: answered from its kept response, not run twice
    last = [x for x in plan if x[1] == 0][-1]
    plan.append((r, 0, last[2], last[3], False, last[5]))
    r += 1
    # One ahead of the sequence: not answered.  Then the sequence goes on.
    plan.append((r, 0, (seqs[0] + 5) & 0xFFFF, bytes([0x01, 0, 0]), False, None))
    pkt = transfer((DP_R(1), None),)
    plan.append((r, 0, seqs[0], pkt, False, firmware[0].packet(pkt)))
    seqs[0] += 1

    # The datagrams, and after each round how many replies there must be
    memory, lengths, rounds, expects = bytearray(), [], [], []
    count = 0
    for n, (rnd, slot, seq, pkt, sync, want) in enumerate(plan):
        d = datagram(slot, seq, pkt, sync)
        memory += d
        lengths.append(len(d))
        rounds.append(rnd)
        if want is not None:
            count += 1
        expects.append(count)

    os.makedirs(OUT, exist_ok=True)
    files = {}
    for name, items, fmt in [("bytes", memory, "{:02x}"), ("lengths", lengths, "{:08x}"),
                             ("rounds", rounds, "{:08x}"), ("expects", expects, "{:08x}")]:
        files[name] = os.path.join(OUT, f"tb_{name}.hex")
        with open(files[name], "w") as f:
            f.write("\n".join(fmt.format(x) for x in items) + "\n")

    with open(os.path.join(OUT, "generated", "csr.json")) as f:
        sys_hz = json.load(f)["constants"]["config_clock_frequency"]
    tb = os.path.join(OUT, "tb_soc.v")
    with open(tb, "w") as f:
        f.write(TESTBENCH % dict(
            slot_wires="".join((JTAG_WIRES if i in jtag else SLOT_WIRES).format(i=i) for i in range(nslots)),
            slot_ports="".join((JTAG_PORTS if i in jtag else SLOT_PORTS).format(i=i) for i in range(nslots)),
            slot_targets="".join((JTAG_TARGET if i in jtag else SLOT_TARGET).format(i=i, rt=args.round_trip)
                                 for i in range(nslots)),
            total=len(memory), count=len(plan), timeout=int(5e9), sys_half=500e6 / sys_hz,
            vcd=f'$dumpfile("{OUT}/soc.vcd"); $dumpvars(0, tb);' if args.vcd else "",
            **files))
    with open(os.path.join(OUT, "sources.txt")) as f:
        sources = [line.strip() for line in f if line.strip()]
    vvp = os.path.join(OUT, "soc.vvp")
    subprocess.run(["iverilog", "-g2012", "-o", vvp, tb, os.path.join(OUT, "sim_top.v"),
                    os.path.join(HERE, "slot_target.v"), os.path.join(HERE, "jtag_target.v"),
                    os.path.join(HERE, "xilinx_ddr.v")] + sources,
                   check=True)
    result = subprocess.run(["vvp", "-n", vvp], cwd=OUT, capture_output=True)
    output = result.stdout.decode("latin-1")
    with open(os.path.join(OUT, "log.txt"), "w") as f:
        f.write(output)

    # Replies by slot and sequence number, in the order they came
    got = {}
    for line in output.splitlines():
        m = re.match(r"TX (\d+)((?: [0-9a-fx]{2})+)$", line)
        if not m:
            continue
        data = bytes.fromhex(m.group(2).replace(" ", ""))
        slot, seq = (data[1] >> 4) & 7, data[2] << 8 | data[3]
        got.setdefault((slot, seq), []).append((int(m.group(1)), data[4:]))

    bad = 0
    wanted = {}
    for rnd, slot, seq, pkt, sync, want in plan:
        if want is not None:
            wanted.setdefault((slot, seq), []).append(want)
    for key, wants in wanted.items():
        replies = got.get(key, [])
        if len(replies) != len(wants) or any(w != g and w != b"*" for w, (_, g) in zip(wants, replies)):
            bad += 1
            if bad <= 12:
                print(f"slot {key[0]} seq {key[1]}:\n  want {[w.hex() for w in wants]}\n"
                      f"  got  {[g.hex() for _, g in replies]}")
    extra = [k for k in got if k not in wanted]
    for k in extra:
        bad += 1
        print(f"slot {k[0]} seq {k[1]}: answered, and should not have been")
    console = "".join(chr(int(c, 16)) for c in re.findall(r"^U ([0-9a-f]{2})$", output, re.M))
    if "TIMEOUT" in output:
        print("TIMEOUT")
        bad += 1
    end = re.search(r"^END (\d+)", output, re.M)
    print(f"{used} slots ({len(jtag & set(range(used)))} JTAG), round trip {args.round_trip:g} ns: {len(wanted)} requests, {bad} wrong; "
          f"console: {console.strip()!r}; simulated {int(end.group(1)) / 1e9 if end else 0:.2f} ms")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
