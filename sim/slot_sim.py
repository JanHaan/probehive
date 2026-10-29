#!/usr/bin/env python3
"""An SWD slot, swd_exec and swd_phy, against a simulated target, checked byte for byte.

The commands a host sends, DAP_Transfer, DAP_TransferBlock, DAP_WriteABORT, DAP_SWJ_Sequence and
DAP_SWD_Sequence, go through the slot's request buffer under Icarus Verilog, with the pins through
DDR registers (sim/xilinx_ddr.v) and the target (sim/slot_target.v) a round trip away.  Every
response is compared with what a port of ARM's CMSIS-DAP reference (DAP.c) gives against a model
of the same target, so the order of transfers, posted AP reads, value match, WAIT retries and the
end of a command at a FAULT are all checked, not only the words read back.

With --multidrop the target is an SWD-DP on a multi-drop bus, selected only by a TARGETSEL that
lands on the cycle after a line reset, as OpenOCD (DAP_SWD_Sequence: bits driven, read, driven)
and probe-rs (DAP_SWJ_Sequence) send it; a sequence clocked a cycle more or less selects nothing.

With --dormant the target starts dormant and wakes only on the selection alert, exactly four cycles
low and the activation code, so a probe that clocks a cycle between two DAP_SWJ_Sequence commands,
as probe-rs splits the wake-up, leaves it asleep.

    python3 sim/slot_sim.py [--clock 50e6] [--round-trip 20] [--wait-every 0] [--fault-at 0]
                            [--write-fault-at 0] [--seed 1] [--random 20] [--multidrop]
                            [--dormant] [--vcd]

The engine's clock is 100 MHz; SWCLK is a whole number of half cycles of it, at least four, the
fastest that is not above --clock.  Exits non-zero on any difference.
"""

import argparse
import os
import random
import re
import struct
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
OUT = os.path.join(ROOT, "build", "slot_sim")
ENGINE_HZ = 100e6

OK, WAIT, FAULT = 1, 2, 4
DPIDR = 0x0BC12477
TARGETID = 0x01002927                                   # the RP2040's core 0
MEM_WORDS = 1024
ALERT_BYTES = bytes([0x92, 0xF3, 0x09, 0x62, 0x95, 0x2D, 0x85, 0x86,       # ADIv5.2 selection alert
                     0xE9, 0xAF, 0xDD, 0xE3, 0xA2, 0x0E, 0xBC, 0x19])
SELECTION_ALERT = [(ALERT_BYTES[i // 8] >> (i % 8)) & 1 for i in range(128)]


# The target, as sim/slot_target.v has it =================================================================

class Target:
    def __init__(self, wait_every=0, fault_at=0, write_fault_at=0, multidrop=False, dormant=False):
        self.wait_every, self.fault_at, self.write_fault_at = wait_every, fault_at, write_fault_at
        # Multi-drop: selected only by a TARGETSEL with TARGETID right after a line reset
        self.multidrop, self.selected, self.awaiting = multidrop, not multidrop, False
        self.ones, self.seen = 0, []
        # Dormant: woken by the selection alert, four cycles low and the activation code
        self.dormant, self.hist, self.act, self.after_reset = dormant, [], None, None
        self.requests = self.writes = 0
        self.sticky = False
        self.ctrlstat = self.select = self.tar = self.ap8 = self.ap_buf = 0
        self.csw = 0x23000052
        self.mem = [(0xA5A50000 ^ (i * 0x01010101)) & 0xFFFFFFFF for i in range(MEM_WORDS)]

    def ap_read(self, a):
        if a == 0:
            return self.csw
        if a == 1:
            return self.tar
        if a == 2:
            return self.ap8
        word = self.mem[(self.tar >> 2) % MEM_WORDS]
        self.tar = (self.tar + 4) & 0xFFFFFFFF
        return word

    def wire(self, bits):
        """Bits a sequence puts on the line (a released one reads as the pull-up's 1)."""
        for b in bits:
            self.hist = (self.hist + [b])[-128:]
            if self.dormant:
                if self.act is not None:
                    self.act.append(b)
                    if len(self.act) == 12:
                        self.dormant = self.act != [0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 0, 0]
                        self.act = None
                elif self.hist == SELECTION_ALERT:
                    self.act = []
                self.ones = self.ones + 1 if b else 0
                continue
            if not b and self.ones >= 50:
                self.after_reset = [b]
            elif self.after_reset is not None:
                self.after_reset.append(b)
                if len(self.after_reset) == 16:
                    if sum(x << i for i, x in enumerate(self.after_reset)) == 0xE3BC:
                        self.dormant, self.selected, self.awaiting = True, not self.multidrop, False
                        self.ones = 0
                        self.after_reset = None
                        continue
                    self.after_reset = None
            if self.multidrop and not b and self.ones >= 50:
                self.selected, self.awaiting, self.seen = False, True, []
            self.ones = self.ones + 1 if b else 0
            if self.awaiting:
                self.seen.append(b)
                self.targetsel()

    def targetsel(self):
        """After a line reset: idle, then the request, five cycles, 32 bits and parity."""
        bits = self.seen[self.seen.index(1):] if 1 in self.seen else []
        if len(bits) == 8 and sum(b << i for i, b in enumerate(bits)) != 0x99:
            self.awaiting = False                            # not TARGETSEL: deselected
        elif len(bits) == 8 + 5 + 33:
            word = sum(b << i for i, b in enumerate(bits[13:45]))
            self.selected = word == TARGETID and bits[45] == bin(word).count("1") % 2
            self.awaiting = False

    def xfer(self, apndp, rnw, a, data=0):
        self.ones = 0
        self.after_reset = None
        if self.dormant or not self.selected:
            self.awaiting = False
            return 7, None                                   # nothing answers
        if rnw and not apndp and a == 0:
            return OK, DPIDR
        self.requests += 1
        if not rnw:
            self.writes += 1
        faulting = ((self.fault_at and self.requests == self.fault_at) or
                    (self.write_fault_at and not rnw and self.writes == self.write_fault_at) or
                    (self.sticky and apndp))
        waiting = not faulting and self.wait_every and self.requests % self.wait_every == 0
        if faulting:
            self.sticky = True
            return FAULT, None
        if waiting:
            return WAIT, None
        if rnw:
            if not apndp:
                value = {1: self.ctrlstat | (0x20 if self.sticky else 0),
                         2: self.ap_buf, 3: self.ap_buf}.get(a, 0)
            else:
                value = self.ap_buf
                self.ap_buf = self.ap_read(a)
            return OK, value
        if not apndp:
            if a == 0:
                if data != 0:
                    self.sticky = False
            elif a == 1:
                self.ctrlstat = data
            elif a == 2:
                self.select = data
        elif a == 0:
            self.csw = data
        elif a == 1:
            self.tar = data
        elif a == 2:
            self.ap8 = data
        else:
            self.mem[(self.tar >> 2) % MEM_WORDS] = data
            self.tar = (self.tar + 4) & 0xFFFFFFFF
        return OK, None


# ARM's CMSIS-DAP reference, DAP.c, for these commands ====================================================

APNDP, RNW, MATCH_VALUE, MATCH_MASK = 0x01, 0x02, 0x10, 0x20
RDBUFF = 0x0E


class Reference:
    def __init__(self, target, retry, match_retry):
        self.t, self.retry, self.match_retry = target, retry, match_retry
        self.mask = 0xFFFFFFFF

    def swd(self, req, data=0, retries=None):
        r = self.retry if retries is None else retries
        while True:
            ack, value = self.t.xfer(req & APNDP, (req >> 1) & 1, (req >> 2) & 3, data)
            if ack != WAIT or r == 0:
                return ack, value
            r -= 1

    def transfer(self, cmd):
        p = 3                                         # command, index, count
        count = cmd[2]
        out, done, value = b"", 0, 0
        post = check_write = False
        while count:
            count -= 1
            req = cmd[p]
            p += 1
            if req & RNW:
                if post:
                    if (req & (APNDP | MATCH_VALUE)) == APNDP:
                        value, data = self.swd(req)
                    else:
                        value, data = self.swd(RDBUFF)
                        post = False
                    if value != OK:
                        break
                    out += struct.pack("<I", data)
                if req & MATCH_VALUE:
                    match = struct.unpack_from("<I", cmd, p)[0]
                    p += 4
                    mretry = self.match_retry
                    if req & APNDP:
                        value, _ = self.swd(req)
                        if value != OK:
                            break
                    while True:
                        value, data = self.swd(req)
                        if value != OK:
                            break
                        if (data & self.mask) == match or mretry == 0:
                            break
                        mretry -= 1
                    if value == OK and (data & self.mask) != match:
                        value |= 0x10
                    if value != OK:
                        break
                else:
                    if req & APNDP:
                        if not post:
                            value, _ = self.swd(req)
                            if value != OK:
                                break
                            post = True
                    else:
                        value, data = self.swd(req)
                        if value != OK:
                            break
                        out += struct.pack("<I", data)
                check_write = False
            else:
                if post:
                    value, data = self.swd(RDBUFF)
                    if value != OK:
                        break
                    out += struct.pack("<I", data)
                    post = False
                data = struct.unpack_from("<I", cmd, p)[0]
                p += 4
                if req & MATCH_MASK:
                    self.mask = data
                    value = OK
                else:
                    value, _ = self.swd(req, data)
                    if value != OK:
                        break
                    check_write = True
            done += 1
        while count:                                  # past the requests not done
            count -= 1
            req = cmd[p]
            p += 1
            if not (req & RNW) or (req & MATCH_VALUE):
                p += 4
        if value == OK:
            if post:
                v, data = self.swd(RDBUFF)
                value = v
                if v == OK:
                    out += struct.pack("<I", data)
            elif check_write:
                value, _ = self.swd(RDBUFF)
        return bytes([0x05, done, value]) + out, p

    def transfer_block(self, cmd):
        count = struct.unpack_from("<H", cmd, 2)[0]
        req = cmd[4] if count else 0
        out, done, value = b"", 0, 0
        length = 5 + (4 * count if count and not (req & RNW) else 0)
        if count:
            if req & RNW:
                failed = False
                if req & APNDP:
                    value, _ = self.swd(req)
                    failed = value != OK
                if not failed:
                    for i in range(count):
                        r = RDBUFF if (i == count - 1 and req & APNDP) else req
                        value, data = self.swd(r)
                        if value != OK:
                            break
                        out += struct.pack("<I", data)
                        done += 1
            else:
                for i in range(count):
                    value, _ = self.swd(req, struct.unpack_from("<I", cmd, 5 + 4 * i)[0])
                    if value != OK:
                        break
                    done += 1
                else:
                    value, _ = self.swd(RDBUFF)
        return bytes([0x06]) + struct.pack("<H", done) + bytes([value]) + out, length

    def write_abort(self, cmd):
        self.swd(0x00, struct.unpack_from("<I", cmd, 2)[0], retries=0)
        return bytes([0x08, 0x00]), 6

    def swj_sequence(self, cmd):
        bits = cmd[1] or 256
        self.t.wire([(cmd[2 + i // 8] >> (i % 8)) & 1 for i in range(bits)])
        return bytes([0x12, 0x00]), 2 + (bits + 7) // 8

    def swd_sequence(self, cmd):
        # The bits read next to bits driven are not specified: the first after them is taken while
        # the probe may still drive the line, and the last before them, with the probe reading a
        # round trip late, after it drives again.  In SWD these are turnaround cycles.
        p, out, mask = 2, b"", bytearray()
        infos, q = [], 2
        for _ in range(cmd[1]):
            infos.append(cmd[q])
            bits = cmd[q] & 0x3F or 64
            q += 1 + (0 if cmd[q] & 0x80 else (bits + 7) // 8)
        for k in range(cmd[1]):
            info = cmd[p]
            p += 1
            bits = info & 0x3F or 64
            if info & 0x80:
                after_out = k > 0 and not infos[k - 1] & 0x80
                before_out = k + 1 < len(infos) and not infos[k + 1] & 0x80
                loose = (1 if after_out else 0) | ((0b111 << max(bits - 3, 0)) if before_out else 0)
                loose &= (1 << bits) - 1
                mask += loose.to_bytes((bits + 7) // 8, "little")
                # Nothing drives the line: the pull-up, all ones, in whole bytes per run of 32
                self.t.wire([1] * bits)
                while bits:
                    run = min(bits, 32)
                    out += ((1 << run) - 1).to_bytes((run + 7) // 8, "little")
                    bits -= run
            else:
                self.t.wire([(cmd[p + i // 8] >> (i % 8)) & 1 for i in range(bits)])
                p += (bits + 7) // 8
        self.loose = bytes(2) + bytes(mask)
        return bytes([0x1D, 0x00]) + out, p

    def run(self, cmd):
        self.loose = None
        return {0x05: self.transfer, 0x06: self.transfer_block, 0x08: self.write_abort,
                0x12: self.swj_sequence, 0x1D: self.swd_sequence}[cmd[0]](cmd)


# Commands ==================================================================================================

def transfer(*reqs):
    """reqs: (request byte, word or None)"""
    body = b""
    for req, word in reqs:
        body += bytes([req]) + (struct.pack("<I", word) if word is not None else b"")
    return bytes([0x05, 0x00, len(reqs)]) + body


def block_read(req, count):
    return bytes([0x06, 0x00]) + struct.pack("<H", count) + bytes([req])


def block_write(req, words):
    return bytes([0x06, 0x00]) + struct.pack("<H", len(words)) + bytes([req]) + \
        b"".join(struct.pack("<I", w) for w in words)


# Requests: DP/AP, R/W, address
DP_R = lambda a: 0x02 | (a << 2)        # noqa: E731
DP_W = lambda a: 0x00 | (a << 2)        # noqa: E731
AP_R = lambda a: 0x03 | (a << 2)        # noqa: E731
AP_W = lambda a: 0x01 | (a << 2)        # noqa: E731


def script(rng, randoms):
    cmds = [
        bytes([0x12, 56]) + bytes([0xFF] * 6 + [0x03]),                    # line reset
        transfer((DP_R(0), None)),                                          # DPIDR
        transfer((DP_W(1), 0x50000000), (DP_W(2), 0), (AP_W(0), 0x23000012),
                 (AP_W(1), 0x100), (AP_R(3), None), (AP_R(3), None), (AP_R(3), None),
                 (DP_R(1), None), (AP_R(1), None), (AP_R(3), None), (DP_W(2), 0)),
        transfer(),                                                          # no requests
        block_read(AP_R(3), 0),
        transfer((AP_W(1), 0x40),),
        block_read(AP_R(3), 64),
        transfer((AP_W(1), 0x200),),
        block_write(AP_W(3), [0xC0DE0000 + i for i in range(64)]),
        transfer((AP_W(1), 0x200),),
        block_read(AP_R(3), 64),
        block_read(DP_R(1), 3),                                              # a DP block read
        # Value match: a mask, a match at once, a match on the AP, a mismatch
        transfer((DP_W(1) | MATCH_MASK, 0x0000FFFF), (DP_W(1), 0x12345678),
                 (DP_R(1) | MATCH_VALUE, 0x5678)),
        transfer((AP_W(2), 0xCAFE0001), (AP_R(2) | MATCH_VALUE, 0x0001)),
        transfer((DP_R(1) | MATCH_VALUE, 0x0001), (DP_R(1), None)),          # never: the rest skipped
        transfer((DP_W(1) | MATCH_MASK, 0xFFFFFFFF),),
        # Sequences: bits read back from the released line, and driven
        bytes([0x1D, 3, 0x80 | 5, 0x08, 0xAA, 0x80 | 0]),
        # Every way runs follow each other: driven, read, a single bit, read after read
        bytes([0x1D, 6, 3, 0x05, 0x80 | 7, 1, 0x00, 0x80 | 2, 0x80 | 33, 0x80 | 1]),
        bytes([0x12, 0]) + bytes(32),                                        # 256 bits
        # The released line read as ones: the target took them for a request and locked out
        bytes([0x12, 56]) + bytes([0xFF] * 6 + [0x03]),                    # line reset
        transfer((DP_R(0), None)),
        bytes([0x08, 0x00]) + struct.pack("<I", 0x1E),                      # DAP_WriteABORT
    ]
    for _ in range(randoms):
        reqs = []
        for _ in range(rng.randrange(1, 12)):
            kind = rng.randrange(6)
            if kind == 0:
                reqs.append((AP_W(1), rng.randrange(0, MEM_WORDS) * 4))
            elif kind == 1:
                reqs.append((AP_W(3), rng.getrandbits(32)))
            elif kind in (2, 3):
                reqs.append((AP_R(3), None))
            elif kind == 4:
                reqs.append((DP_R(rng.choice([1, 3])), None))
            else:
                reqs.append((AP_R(rng.choice([0, 1, 2])), None))
        cmds.append(transfer(*reqs))
        if rng.random() < 0.3:
            cmds.append(transfer((AP_W(1), rng.randrange(0, MEM_WORDS // 2) * 4),))
            cmds.append(block_read(AP_R(3), rng.randrange(1, 40)) if rng.random() < 0.5 else
                        block_write(AP_W(3), [rng.getrandbits(32) for _ in range(rng.randrange(1, 40))]))
    # After any FAULT: clear it, and read on
    cmds += [bytes([0x08, 0x00]) + struct.pack("<I", 0x1E),
             transfer((AP_W(1), 0x0),), block_read(AP_R(3), 16)]
    return cmds


LINE_RESET = bytes([0x12, 56]) + bytes([0xFF] * 6 + [0x03])          # 50 ones, then low


def bits_bytes(bits):
    return bytes(sum(b << i for i, b in enumerate(bits[j:j + 8])) for j in range(0, len(bits), 8))


def targetsel_openocd(target_id, lead=0):
    """TARGETSEL as OpenOCD sends it: DAP_SWD_Sequence, the request, five cycles read, the ID and
    its parity; `lead` idle cycles driven low before, as a run of their own."""
    seq = bytes([0x08, 0x99, 0x80 | 5, 33]) + struct.pack("<I", target_id) + \
        bytes([bin(target_id).count("1") % 2])
    return bytes([0x1D, 4, lead, 0x00]) + seq if lead else bytes([0x1D, 3]) + seq


def targetsel_probe_rs(target_id):
    """TARGETSEL as probe-rs sends it: one DAP_SWJ_Sequence, every bit driven."""
    bits = [(0x99 >> i) & 1 for i in range(8)] + [0] * 5 + \
        [(target_id >> i) & 1 for i in range(32)] + [bin(target_id).count("1") % 2]
    return bytes([0x12, len(bits)]) + bits_bytes(bits)


def multidrop_script(rng, randoms):
    """A multi-drop target: selected only by an exact TARGETSEL after a line reset."""
    work = [transfer((DP_R(0), None)),
            transfer((DP_W(1), 0x50000000), (DP_W(2), 0), (AP_W(1), 0x80), (AP_W(3), 0xFEEDF00D),
                     (AP_W(1), 0x80), (AP_R(3), None), (DP_R(1), None))]
    cmds = [LINE_RESET, targetsel_openocd(TARGETID)] + work + [
        LINE_RESET, targetsel_openocd(TARGETID ^ 0x10), transfer((DP_R(0), None)),   # another ID
        LINE_RESET, targetsel_probe_rs(TARGETID)] + work + [
        LINE_RESET, transfer((DP_R(0), None)),                                         # no TARGETSEL
        LINE_RESET, targetsel_openocd(TARGETID, lead=2)] + work
    for _ in range(randoms):
        cmds.append(transfer(*[(AP_W(1), rng.randrange(0, MEM_WORDS) * 4) if rng.random() < 0.3
                               else (AP_R(3), None) for _ in range(rng.randrange(1, 8))]))
    return cmds


def swj(data, bits):
    return bytes([0x12, bits & 0xFF]) + data


def dormant_script(rng, randoms, multidrop=False):
    """A dormant target, woken as probe-rs does it, a DAP_SWJ_Sequence a part, and as OpenOCD does,
    one for all; the bits between two commands are only those the host sends."""
    select = [targetsel_probe_rs(TARGETID)] if multidrop else []
    work = [transfer((DP_R(0), None)),
            transfer((DP_W(1), 0x50000000), (DP_W(2), 0), (AP_W(1), 0x80), (AP_W(3), 0xFEEDF00D),
                     (AP_W(1), 0x80), (AP_R(3), None), (DP_R(1), None))]
    cmds = [transfer((DP_R(0), None)),                                         # dormant: no answer
            # probe-rs: a line reset, JTAG to dormant, 8 cycles high, the alert, then 4 low and
            # the activation code, each a command of its own
            swj(bytes([0xFF] * 6 + [0x07]), 51), swj(bytes([0xBA, 0xBB, 0xBB, 0x33]), 31),
            swj(bytes([0xFF]), 8), swj(ALERT_BYTES, 128), swj(bytes([0xA0, 0x01]), 12),
            LINE_RESET] + select + work + [
            # Back to dormant: 56 cycles high, then SWD to dormant at once
            swj(bytes([0xFF] * 7), 56), swj(bytes([0xBC, 0xE3]), 16), transfer((DP_R(0), None)),
            # OpenOCD: 8 high, the alert, 4 low, the code, and a line reset, in one
            swj(bytes([0xFF]) + ALERT_BYTES + bytes([0xA0, 0xF1, 0xFF] + [0xFF] * 7 + [0x00]), 224)
            ] + select + work
    for _ in range(randoms):
        cmds.append(transfer(*[(AP_W(1), rng.randrange(0, MEM_WORDS) * 4) if rng.random() < 0.3
                               else (AP_R(3), None) for _ in range(rng.randrange(1, 8))]))
    return cmds


# The testbench =============================================================================================

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

    reg cmd_req = 0, clk_set = 0, abort = 0;
    reg [15:0] cmd_rq_base = 0, cmd_rs_base = 0;
    wire cmd_done; wire [15:0] cmd_rq_len, cmd_rs_len;
    wire [9:0] phy_hi, phy_lo;

    wire op_valid, op_ready, op_rnw, op_apndp, resume, res_hold, res_valid, res_perr, halted, busy, own;
    wire [1:0] op_kind, op_addr; wire [31:0] op_data, res_data; wire [4:0] op_bits;
    wire [15:0] op_retries; wire [3:0] op_tag, res_tag; wire [2:0] res_ack;
    wire [5:0] cal_sample, cal_first; wire [6:0] cal_count; wire cal_ok, cal_seen;
    wire [1:0] swclk_p, swdo_p, swwr_p, swdi_p;

    swd_exec #(.AW(16)) exec(.clk(clk), .rst(rst),
        .cmd_req(cmd_req), .cmd_rq_base(cmd_rq_base), .cmd_rs_base(cmd_rs_base), .cmd_done(cmd_done),
        .cmd_rq_len(cmd_rq_len), .cmd_rs_len(cmd_rs_len), .abort(abort),
        .cfg_retry(16'd%(retry)d), .cfg_match_retry(16'd%(match_retry)d), .cfg_sample(7'd0),
        .clk_set(clk_set), .clk_hi(10'd%(hi)d), .clk_lo(10'd%(lo)d), .phy_hi(phy_hi), .phy_lo(phy_lo),
        .rq_addr(rq_addr), .rq_data(rq_data), .rs_we(rs_we), .rs_addr(rs_addr), .rs_data(rs_data),
        .op_valid(op_valid), .op_ready(op_ready), .op_kind(op_kind), .op_rnw(op_rnw),
        .op_apndp(op_apndp), .op_addr(op_addr), .op_data(op_data), .op_bits(op_bits),
        .op_retries(op_retries), .op_tag(op_tag), .resume(resume), .res_hold(res_hold),
        .res_valid(res_valid), .res_ack(res_ack), .res_perr(res_perr), .res_data(res_data),
        .res_tag(res_tag), .cal_ok(cal_ok), .cal_seen(cal_seen));

    swd_phy phy(.clk(clk), .rst(rst), .own(own), .swclk(swclk_p), .swdo(swdo_p), .swwr(swwr_p),
        .swdi(swdi_p), .cfg_hi(phy_hi), .cfg_lo(phy_lo), .cfg_turn(2'd0), .cfg_idle(8'd0),
        .cfg_dataphase(1'b0), .cfg_sample(7'd0),
        .op_valid(op_valid), .op_ready(op_ready), .op_kind(op_kind), .op_rnw(op_rnw),
        .op_apndp(op_apndp), .op_addr(op_addr), .op_data(op_data), .op_bits(op_bits),
        .op_retries(op_retries), .op_tag(op_tag), .resume(resume), .res_hold(res_hold),
        .res_valid(res_valid), .res_ack(res_ack), .res_perr(res_perr), .res_data(res_data),
        .res_tag(res_tag), .halted(halted), .busy(busy),
        .cal_sample(cal_sample), .cal_first(cal_first), .cal_count(cal_count), .cal_ok(cal_ok),
        .cal_seen(cal_seen));

    // The pins, through DDR registers; parked by the slot when the engine does not own them
    wire swclk, swdo, swwr, swdi;
    ODDR o_clk(.Q(swclk), .C(clk), .CE(1'b1), .D1(own ? swclk_p[0] : 1'b1), .D2(own ? swclk_p[1] : 1'b1), .R(1'b0), .S(1'b0));
    ODDR o_do (.Q(swdo),  .C(clk), .CE(1'b1), .D1(own ? swdo_p[0] : 1'b0),  .D2(own ? swdo_p[1] : 1'b0),  .R(1'b0), .S(1'b0));
    ODDR o_wr (.Q(swwr),  .C(clk), .CE(1'b1), .D1(own ? swwr_p[0] : 1'b1),  .D2(own ? swwr_p[1] : 1'b1),  .R(1'b0), .S(1'b0));
    IDDR i_di (.Q1(swdi_p[0]), .Q2(swdi_p[1]), .C(clk), .CE(1'b1), .D(swdi), .R(1'b0), .S(1'b0));

    // What the target drives reaches the probe a round trip after the probe's own clock edge
    wire target_drive, target_bit;
    reg drive_late = 0, bit_late = 1;
    always @(target_drive) drive_late <= #(%(round_trip)s) target_drive;
    always @(target_bit) bit_late <= #(%(round_trip)s) target_bit;
    assign swdi = swwr ? swdo : (drive_late ? bit_late : 1'b1);
    swd_target #(.WAIT_EVERY(%(wait_every)d), .FAULT_AT(%(fault_at)d),
                 .WRITE_FAULT_AT(%(write_fault_at)d),
                 .MULTIDROP(%(multidrop)d), .DORMANT(%(dormant)d)) target(.swclk(swclk), .line(swdi),
                 .drive(target_drive), .out_bit(target_bit));

    integer bases [0:%(cmax)d];
    integer n, i;
    reg done_was;
    initial begin
        %(vcd)s
        $readmemh("%(bytes)s", rq_mem);
        $readmemh("%(bases)s", bases);
        repeat (20) @(posedge clk);
        rst = 0;
        repeat (20) @(posedge clk);
        clk_set = 1;
        repeat (20) @(posedge clk);
        for (n = 0; n < %(count)d; n = n + 1) begin
            @(negedge clk);
            cmd_rq_base = bases[n]; cmd_rs_base = (n %% 31) * 2048;
            done_was = cmd_done;
            $display("C %%0t %%0d", $time, n);
            cmd_req = !cmd_req;
            while (cmd_done == done_was) @(posedge clk);
            #1;
            $write("D %%0t %%0d %%0d %%0d", $time, n, cmd_rq_len, cmd_rs_len);
            for (i = 0; i < cmd_rs_len; i = i + 1) $write(" %%02x", rs_mem[(n %% 31) * 2048 + i]);
            $write("\n");
        end
        $display("CAL sample %%0d first %%0d count %%0d ok %%0d hi %%0d lo %%0d", cal_sample,
                 cal_first, cal_count, cal_ok, phy_hi, phy_lo);
        $finish;
    end
    initial begin #%(timeout)d $display("TIMEOUT %%0t", $time); $finish; end
endmodule
"""


def ticks_for(clock):
    """SWCLK in half cycles of the engine: the fastest not above `clock`, at least four."""
    ticks = max(4, -(-int(2 * ENGINE_HZ) // int(clock)))
    return ticks // 2, ticks - ticks // 2


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--clock", type=float, default=50e6)
    parser.add_argument("--round-trip", type=float, default=20, help="ns")
    parser.add_argument("--wait-every", type=int, default=0)
    parser.add_argument("--fault-at", type=int, default=0)
    parser.add_argument("--write-fault-at", type=int, default=0)
    parser.add_argument("--retry", type=int, default=100)
    parser.add_argument("--match-retry", type=int, default=3)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--random", type=int, default=20, help="random DAP_Transfer commands")
    parser.add_argument("--multidrop", action="store_true",
                        help="a multi-drop target, selected by TARGETSEL as OpenOCD and probe-rs send it")
    parser.add_argument("--dormant", action="store_true",
                        help="a dormant target, woken as probe-rs and OpenOCD wake it")
    parser.add_argument("--vcd", action="store_true")
    args = parser.parse_args()

    rng = random.Random(args.seed)
    if args.dormant:
        cmds = dormant_script(rng, args.random, args.multidrop)
    else:
        cmds = (multidrop_script if args.multidrop else script)(rng, args.random)
    hi, lo = ticks_for(args.clock)

    # The reference's responses
    reference = Reference(Target(args.wait_every, args.fault_at, args.write_fault_at, args.multidrop,
                                 args.dormant),
                          args.retry, args.match_retry)
    expected, masks = [], []
    for c in cmds:
        expected.append(reference.run(c))
        masks.append(reference.loose)

    # The request buffer: each command at its own offset
    os.makedirs(OUT, exist_ok=True)
    tag = (f"{int(args.clock)}_{args.round_trip:g}_{args.wait_every}_{args.fault_at}_{args.write_fault_at}_"
           f"{args.seed}{'_md' if args.multidrop else ''}{'_dm' if args.dormant else ''}")
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
            retry=args.retry, match_retry=args.match_retry, hi=hi, lo=lo,
            round_trip=args.round_trip, wait_every=args.wait_every, fault_at=args.fault_at,
            write_fault_at=args.write_fault_at, multidrop=int(args.multidrop),
            dormant=int(args.dormant), cmax=len(cmds), count=len(cmds),
            bytes=bytes_hex, bases=bases_hex, timeout=int(2e9),
            vcd=f'$dumpfile("{OUT}/slot_{tag}.vcd"); $dumpvars(0, tb);' if args.vcd else ""))
    vvp = os.path.join(OUT, f"slot_{tag}.vvp")
    sources = [tb, os.path.join(HERE, "slot_target.v"), os.path.join(HERE, "xilinx_ddr.v"),
               os.path.join(ROOT, "gateware", "verilog", "swd_exec.v"),
               os.path.join(ROOT, "gateware", "verilog", "swd_phy.v")]
    subprocess.run(["iverilog", "-g2012", "-o", vvp] + sources, check=True)
    output = subprocess.run(["vvp", "-n", vvp], check=True, capture_output=True, text=True).stdout
    with open(os.path.join(OUT, f"log_{tag}.txt"), "w") as f:
        f.write(output)

    # Compare
    got = {}
    for line in output.splitlines():
        m = re.match(r"D (\d+) (\d+) (\d+) (\d+)((?: [0-9a-fx]{2})*)$", line)
        if m:
            got[int(m.group(2))] = (int(m.group(3)), bytes.fromhex(m.group(5).replace(" ", "")),
                                    int(m.group(1)))
    bad = 0
    for n, (c, (want, length), mask) in enumerate(zip(cmds, expected, masks)):
        if n not in got:
            print(f"command {n} ({c[:8].hex()}...): no response")
            bad += 1
            break
        rq_len, data, _ = got[n]
        if mask and len(data) == len(want):                    # bits not specified: as they came
            want = bytes(w & ~m | d & m for w, d, m in zip(want, data, mask))
        if data != want or rq_len != length:
            bad += 1
            print(f"command {n} {c[:12].hex()}...:\n  want {length} {want.hex()}\n  got  {rq_len} {data.hex()}")
    if "TIMEOUT" in output:
        print("TIMEOUT")
        bad += 1
    lockouts = output.count("LOCKOUT")
    cal = [line for line in output.splitlines() if line.startswith("CAL")]

    # Wire time: SWCLK cycles between the starts of the transfers of each block read of 64
    period = (hi + lo) * 5.0                                               # ns
    spacing = []
    starts = [int(t) for t in re.findall(r"^T (\d+) R AP12\s+OK", output, re.M)]
    for t0, t1 in zip(starts, starts[1:]):
        gap = (t1 - t0) / 1000 / period
        if gap < 60:
            spacing.append(gap)
    per_xfer = sorted(spacing)[len(spacing) // 2] if spacing else 0
    print(f"clock {1e3 / period:.1f} MHz, round trip {args.round_trip:g} ns: {len(cmds)} commands, "
          f"{bad} differing, {lockouts} lockouts; {cal[0] if cal else 'no CAL'}; "
          f"AP reads {per_xfer:.1f} SWCLK apart (median)")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
