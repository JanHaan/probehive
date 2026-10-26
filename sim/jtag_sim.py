#!/usr/bin/env python3
"""A JTAG slot's engine, jtag_engine, against a simulated TAP, checked byte for byte.

DAP_JTAG_Sequence and DAP_SWJ_Sequence commands go through the slot's request buffer under Icarus
Verilog, with the pins through DDR registers (sim/xilinx_ddr.v) and the TAP (sim/jtag_target.v)
a round trip away.  Every response is compared with what ARM's CMSIS-DAP reference (DAP.c,
JTAG_Sequence and SWJ_Sequence) gives against a model of the same TAP: reading IDCODE, an
instruction scan, writing and reading back a data register, and random sequences that walk the
TAP wherever their TMS takes it.

    python3 sim/jtag_sim.py [--clock 20e6] [--round-trip 20] [--sample 0] [--seed 1] [--random 40]
                            [--vcd]

The engine's clock is 100 MHz; TCK is a whole number of half cycles of it, at least four, the
fastest that is not above --clock.  TDO is taken --sample half cycles after the rising edge (0:
half way through the high phase), so a round trip must end before then.  Exits non-zero on any
difference.
"""

import argparse
import os
import random
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
OUT = os.path.join(ROOT, "build", "jtag_sim")
ENGINE_HZ = 100e6
IDCODE = 0x120034E5
I_IDCODE, I_SCRATCH = 0x01, 0x02
JTAG_SEQUENCE, SWJ_SEQUENCE = 0x14, 0x12


# The TAP, as sim/jtag_target.v has it ====================================================================

(TLR, RTI, SEL_DR, CAP_DR, SHIFT_DR, EXIT1_DR, PAUSE_DR, EXIT2_DR, UPD_DR,
 SEL_IR, CAP_IR, SHIFT_IR, EXIT1_IR, PAUSE_IR, EXIT2_IR, UPD_IR) = range(16)
NEXT = {TLR: (RTI, TLR), RTI: (RTI, SEL_DR), SEL_DR: (CAP_DR, SEL_IR), CAP_DR: (SHIFT_DR, EXIT1_DR),
        SHIFT_DR: (SHIFT_DR, EXIT1_DR), EXIT1_DR: (PAUSE_DR, UPD_DR), PAUSE_DR: (PAUSE_DR, EXIT2_DR),
        EXIT2_DR: (SHIFT_DR, UPD_DR), UPD_DR: (RTI, SEL_DR), SEL_IR: (CAP_IR, TLR),
        CAP_IR: (SHIFT_IR, EXIT1_IR), SHIFT_IR: (SHIFT_IR, EXIT1_IR), EXIT1_IR: (PAUSE_IR, UPD_IR),
        PAUSE_IR: (PAUSE_IR, EXIT2_IR), EXIT2_IR: (SHIFT_IR, UPD_IR), UPD_IR: (RTI, SEL_DR)}


class Tap:
    def __init__(self):
        self.state, self.ir, self.ir_sr, self.dr, self.scratch, self.tdo = TLR, I_IDCODE, 0, 0, 0, 1

    def cycle(self, tms, tdi):
        """One TCK cycle: the falling edge, TDO as the probe takes it, the rising edge."""
        s = self.state
        self.tdo = self.dr & 1 if s == SHIFT_DR else self.ir_sr & 1 if s == SHIFT_IR else 1
        tdo = self.tdo
        bypass = self.ir not in (I_IDCODE, I_SCRATCH)
        if s == TLR:
            self.ir = I_IDCODE
        elif s == CAP_DR:
            self.dr = IDCODE if self.ir == I_IDCODE else self.scratch if self.ir == I_SCRATCH else 0
        elif s == SHIFT_DR:
            self.dr = tdi if bypass else (tdi << 31) | (self.dr >> 1)
        elif s == UPD_DR:
            if self.ir == I_SCRATCH:
                self.scratch = self.dr
        elif s == CAP_IR:
            self.ir_sr = 0b00001
        elif s == SHIFT_IR:
            self.ir_sr = (tdi << 4) | (self.ir_sr >> 1)
        elif s == UPD_IR:
            self.ir = self.ir_sr
        self.state = NEXT[s][tms]
        return tdo


class Reference:
    """DAP.c's JTAG_Sequence and SWJ_Sequence against a Tap; TDI keeps its level between them, and
    starts low."""

    def __init__(self, tap):
        self.tap, self.tdi = tap, 0

    def run(self, cmd):
        if cmd[0] == SWJ_SEQUENCE:
            n = cmd[1] or 256
            for i in range(n):
                self.tap.cycle((cmd[2 + i // 8] >> (i % 8)) & 1, self.tdi)
            return bytes([SWJ_SEQUENCE, 0]), 2 + (n + 7) // 8
        out, p = bytearray([JTAG_SEQUENCE, 0]), 2
        for _ in range(cmd[1]):
            info = cmd[p]; p += 1
            n, tms, cap = info & 0x3F or 64, (info >> 6) & 1, info >> 7
            while n:
                k = min(8, n); n -= k
                byte, tdo = cmd[p], 0; p += 1
                for i in range(k):
                    self.tdi = (byte >> i) & 1
                    tdo |= self.tap.cycle(tms, self.tdi) << i
                if cap:
                    out.append(tdo)
        return bytes(out), p


# Commands =================================================================================================

def bits_bytes(bits):
    return bytes(sum(b << i for i, b in enumerate(bits[j:j + 8])) for j in range(0, len(bits), 8))


def jtag(*seqs):
    """seqs: (tms, tdi bits, capture)"""
    out = bytearray([JTAG_SEQUENCE, len(seqs)])
    for tms, bits, cap in seqs:
        out.append((len(bits) & 0x3F) | (tms << 6) | (cap << 7))
        out += bits_bytes(bits)
    return bytes(out)


def swj(bits):
    return bytes([SWJ_SEQUENCE, len(bits) & 0xFF]) + bits_bytes(bits)


def word(v, n=32):
    return [(v >> i) & 1 for i in range(n)]


def dr_scan(value, n=32):
    """From Run-Test/Idle through Shift-DR and back, shifting `value` in and the register out."""
    bits = word(value, n)
    return jtag((1, [0], 0), (0, [0, 0], 0), (0, bits[:-1], 1), (1, bits[-1:], 1), (1, [0], 0), (0, [0], 0))


def ir_scan(value):
    bits = word(value, 5)
    return jtag((1, [0, 0], 0), (0, [0, 0], 0), (0, bits[:-1], 1), (1, bits[-1:], 1), (1, [0], 0), (0, [0], 0))


def script(rng, randoms):
    s = [swj([1] * 8), jtag((0, [1], 0)),                                         # reset, Run-Test/Idle
         dr_scan(0),                                                                # IDCODE
         ir_scan(I_SCRATCH), dr_scan(0xDEADBEEF), dr_scan(rng.getrandbits(32)),    # write, read back
         ir_scan(0x1F), dr_scan(0b101, 3),                                          # BYPASS
         jtag(), swj([1] * 5 + [0]), dr_scan(0),                                   # empty; reset again
         jtag((0, [rng.getrandbits(1) for _ in range(64)], 1)),                    # 64 bits in one
         swj([rng.getrandbits(1) for _ in range(256)])]                             # 256 on TMS
    for _ in range(randoms):
        if rng.random() < 0.2:
            s.append(swj([rng.getrandbits(1) for _ in range(rng.randint(1, 255))]))
        else:
            s.append(jtag(*[(rng.getrandbits(1), [rng.getrandbits(1) for _ in range(rng.randint(1, 64))],
                             rng.getrandbits(1)) for _ in range(rng.randint(1, 6))]))
    return s


TESTBENCH = r"""
`timescale 1ns/1ps
module tb;
    reg clk = 0;
    always #5 clk = ~clk;                                               // 100 MHz
    reg rst = 1;

    // The slot's buffers: request bytes read two cycles after the address, response bytes written
    reg [7:0] rq_mem [0:65535];
    reg [7:0] rs_mem [0:65535];
    wire [15:0] rq_addr;
    reg  [7:0] rq_d1 = 0, rq_data = 0;
    always @(posedge clk) begin rq_d1 <= rq_mem[rq_addr]; rq_data <= rq_d1; end
    wire rs_we; wire [15:0] rs_addr; wire [7:0] rs_data;
    always @(posedge clk) if (rs_we) rs_mem[rs_addr] <= rs_data;

    reg cmd_req = 0;
    reg [15:0] cmd_rq_base = 0, cmd_rs_base = 0;
    wire cmd_done, busy; wire [15:0] cmd_rq_len, cmd_rs_len;
    wire [1:0] tck_p, tms_p, tdi_p, tdo_p;

    jtag_engine #(.AW(16)) engine(.clk(clk), .rst(rst),
        .cmd_req(cmd_req), .cmd_rq_base(cmd_rq_base), .cmd_rs_base(cmd_rs_base), .cmd_done(cmd_done),
        .cmd_rq_len(cmd_rq_len), .cmd_rs_len(cmd_rs_len),
        .clk_hi(10'd%(hi)d), .clk_lo(10'd%(lo)d), .cfg_sample(7'd%(sample)d),
        .pins_set(1'b0), .pins_tck(1'b1), .pins_tms(1'b1), .pins_tdi(1'b1),
        .rq_addr(rq_addr), .rq_data(rq_data), .rs_we(rs_we), .rs_addr(rs_addr), .rs_data(rs_data),
        .tck(tck_p), .tms(tms_p), .tdi(tdi_p), .tdo(tdo_p), .busy(busy));

    // The pins, through DDR registers
    wire tck, tms, tdi, tdo;
    ODDR o_tck(.Q(tck), .C(clk), .CE(1'b1), .D1(tck_p[0]), .D2(tck_p[1]), .R(1'b0), .S(1'b0));
    ODDR o_tms(.Q(tms), .C(clk), .CE(1'b1), .D1(tms_p[0]), .D2(tms_p[1]), .R(1'b0), .S(1'b0));
    ODDR o_tdi(.Q(tdi), .C(clk), .CE(1'b1), .D1(tdi_p[0]), .D2(tdi_p[1]), .R(1'b0), .S(1'b0));
    IDDR i_tdo(.Q1(tdo_p[0]), .Q2(tdo_p[1]), .C(clk), .CE(1'b1), .D(tdo), .R(1'b0), .S(1'b0));

    // What the target drives reaches the probe a round trip after the probe's own clock edge
    wire target_tdo;
    reg tdo_late = 1;
    always @(target_tdo) tdo_late <= #(%(round_trip)s) target_tdo;
    assign tdo = tdo_late;
    jtag_target target(.tck(tck), .tms(tms), .tdi(tdi), .tdo(target_tdo));

    // TCK's rising edges, to count the cycles a command takes
    integer edges = 0;
    always @(posedge tck) edges = edges + 1;

    integer bases [0:%(cmax)d];
    integer n, i, e0;
    reg done_was;
    initial begin
        %(vcd)s
        $readmemh("%(bytes)s", rq_mem);
        $readmemh("%(bases)s", bases);
        repeat (20) @(posedge clk);
        rst = 0;
        repeat (20) @(posedge clk);
        for (n = 0; n < %(count)d; n = n + 1) begin
            @(negedge clk);
            cmd_rq_base = bases[n]; cmd_rs_base = (n %% 31) * 2048;
            done_was = cmd_done;
            e0 = edges;
            $display("C %%0t %%0d", $time, n);
            cmd_req = !cmd_req;
            while (cmd_done == done_was) @(posedge clk);
            #1;
            $write("D %%0t %%0d %%0d %%0d %%0d", $time, n, edges - e0, cmd_rq_len, cmd_rs_len);
            for (i = 0; i < cmd_rs_len; i = i + 1) $write(" %%02x", rs_mem[(n %% 31) * 2048 + i]);
            $write("\n");
        end
        $finish;
    end
    initial begin #%(timeout)d $display("TIMEOUT %%0t", $time); $finish; end
endmodule
"""


def count_bits(cmd):
    """TCK cycles of a DAP_JTAG_Sequence"""
    n, p = 0, 2
    for _ in range(cmd[1]):
        k = cmd[p] & 0x3F or 64
        n += k
        p += 1 + (k + 7) // 8
    return n


def ticks_for(clock):
    """TCK in half cycles of the engine: the fastest not above `clock`, at least four."""
    ticks = max(4, -(-int(2 * ENGINE_HZ) // int(clock)))
    return ticks // 2, ticks - ticks // 2


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--clock", type=float, default=20e6)
    parser.add_argument("--round-trip", type=float, default=20, help="ns")
    parser.add_argument("--sample", type=int, default=0, help="half cycles after the rising edge")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--random", type=int, default=40, help="random commands")
    parser.add_argument("--vcd", action="store_true")
    args = parser.parse_args()

    rng = random.Random(args.seed)
    cmds = script(rng, args.random)
    hi, lo = ticks_for(args.clock)

    reference = Reference(Tap())
    expected = [reference.run(c) for c in cmds]

    os.makedirs(OUT, exist_ok=True)
    tag = f"{int(args.clock)}_{args.round_trip:g}_{args.sample}_{args.seed}"
    memory, bases = bytearray(), []
    for c in cmds:
        bases.append(len(memory))
        memory += c + bytes(-len(c) % 4)
    bytes_hex = os.path.join(OUT, f"rq_{tag}.hex")
    bases_hex = os.path.join(OUT, f"bases_{tag}.hex")
    with open(bytes_hex, "w") as f:
        f.write("\n".join(f"{b:02x}" for b in memory) + "\n")
    with open(bases_hex, "w") as f:
        f.write("\n".join(f"{b:08x}" for b in bases) + "\n")

    tb = os.path.join(OUT, f"tb_{tag}.v")
    with open(tb, "w") as f:
        f.write(TESTBENCH % dict(
            hi=hi, lo=lo, sample=args.sample, round_trip=args.round_trip, cmax=len(cmds),
            count=len(cmds), bytes=bytes_hex, bases=bases_hex, timeout=int(2e9),
            vcd=f'$dumpfile("{OUT}/jtag_{tag}.vcd"); $dumpvars(0, tb);' if args.vcd else ""))
    vvp = os.path.join(OUT, f"jtag_{tag}.vvp")
    sources = [tb, os.path.join(HERE, "jtag_target.v"), os.path.join(HERE, "xilinx_ddr.v"),
               os.path.join(ROOT, "gateware", "verilog", "jtag_engine.v")]
    subprocess.run(["iverilog", "-g2012", "-o", vvp] + sources, check=True)
    output = subprocess.run(["vvp", "-n", vvp], check=True, capture_output=True, text=True).stdout
    with open(os.path.join(OUT, f"log_{tag}.txt"), "w") as f:
        f.write(output)

    got, starts = {}, {}
    for line in output.splitlines():
        m = re.match(r"C (\d+) (\d+)$", line)
        if m:
            starts[int(m.group(2))] = int(m.group(1))
        m = re.match(r"D (\d+) (\d+) (\d+) (\d+) (\d+)((?: [0-9a-fx]{2})*)$", line)
        if m:
            got[int(m.group(2))] = (int(m.group(4)), bytes.fromhex(m.group(6).replace(" ", "")),
                                    int(m.group(3)), int(m.group(1)))
    bad = 0
    for n, (c, (want, length)) in enumerate(zip(cmds, expected)):
        if n not in got:
            print(f"command {n} ({c[:8].hex()}...): no response")
            bad += 1
            break
        rq_len, data, edges, _ = got[n]
        bits = count_bits(c) if c[0] == JTAG_SEQUENCE else c[1] or 256
        if data != want or rq_len != length or edges != bits:
            bad += 1
            print(f"command {n} {c[:12].hex()}...:\n  want {length} {want.hex()} ({bits} TCK)\n"
                  f"  got  {rq_len} {data.hex()} ({edges} TCK)")
    if "TIMEOUT" in output:
        print("TIMEOUT")
        bad += 1

    # Wire time of the 256-bit SWJ_Sequence: TCK periods per bit, command overhead included
    period = (hi + lo) * 5.0
    n256 = next(i for i, c in enumerate(cmds) if c[0] == SWJ_SEQUENCE and c[1] == 0)
    took = (got[n256][3] - starts[n256]) / 1000 / period if n256 in got else 0
    print(f"TCK {1e3 / period:.1f} MHz ({lo}+{hi} half cycles), round trip {args.round_trip:g} ns, "
          f"sample {args.sample or (hi + 1) // 2}: {len(cmds)} commands, {bad} differing; "
          f"256 bits on TMS took {took:.1f} TCK periods")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
