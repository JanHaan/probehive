"""Debug slots: a CPU's view of several probes, each on pins of its own.

A slot is a probe for one target.  Its wire engine (an SWD slot: swd_exec and
swd_phy, gateware/verilog) runs in the `slot` clock (100 MHz) and works from
two buffers of the slot's own: requests, which the receiver fills straight
from the network, and responses, which the sender streams back out.  The CPU
(firmware/) sees both buffers on its bus and every slot's registers: it
decides what each datagram is (the next request, a repeat to answer again, or
neither), answers the commands that do not touch the wire itself, and hands
the ones that do to the slot's engine.

A datagram (UDP port 1235) is a 4-byte header and one CMSIS-DAP command packet:

    0xDA, SYNC (0x80) | slot << 4 | version (1), sequence number (16 bits, big endian), command...

A host that knows nothing of slots talks to slot 0.  The reply has the same header, without SYNC,
and the response.  Each slot's requests run strictly in sequence order, so a host may keep as
many in flight as the slot has entries and retransmit go-back-N: the next number (or any, with
SYNC) is run; one answered already is answered again from its kept response, never run twice,
so a lost reply costs no repeated write; anything else is dropped, and the host's retransmission
brings it again.  scripts/dap_udp.py is such a host.

Each buffer is two block RAMs, 8 KiB, ENTRIES entries of ENTRY bytes.  A
request entry holds a command packet; a response entry the 4 bytes of the
reply's header, which the CPU writes, then the response.
"""

import os
from functools import reduce
from operator import or_

from migen import Array, Cat, ClockSignal, Constant, If, Instance, Memory, Mux, Record, Replicate, \
    ResetSignal, Signal, TSTriple
from migen.genlib.cdc import MultiReg
from migen.genlib.fsm import FSM, NextState, NextValue

from litex.gen import LiteXModule
from litex.soc.interconnect import stream, wishbone
from litex.soc.interconnect.csr import CSR, CSRField, CSRStatus, CSRStorage
from litex.soc.cores.uart import RS232PHYRX, RS232PHYTX
from migen.fhdl.decorators import ResetInserter

HERE = os.path.dirname(os.path.abspath(__file__))
VERILOG = os.path.join(HERE, "verilog")

MAGIC = 0xDA
VERSION = 0x01
SYNC = 0x80
HEADER = 4
MAX_PACKET = 1032                # the CMSIS-DAP packet size the firmware reports
ENTRIES = 6                      # requests queued, and responses kept for replay, per slot
ENTRY = 1280                     # bytes: a response entry's header and packet fit
BUFFER_WORDS = 2048              # 8 KiB, two RAMB36
SLOT_BYTES = 0x4000              # a slot's two buffers on the CPU's bus: requests, then responses
assert ENTRIES * ENTRY <= BUFFER_WORDS * 4 and HEADER + MAX_PACKET <= ENTRY
WAW = (BUFFER_WORDS - 1).bit_length()           # a buffer's word address
BAW = WAW + 2                                   # and byte address
EB = (ENTRIES - 1).bit_length()                 # an entry's number

DAP_UDP_PORT = 1235


class SlotBuffers(LiteXModule):
    """A slot's request and response buffers.  On the CPU's side (sys) each is
    shared with the receiver or the sender, which come first; on the engine's
    side (`cd`) the engine reads requests and writes responses a byte at a
    time, a request byte two cycles after its address."""

    def __init__(self, cd="slot"):
        self.bus = wishbone.Interface(data_width=32, adr_width=30)

        # The receiver's word writes, the sender's word reads (sys)
        self.rx_we  = Signal(4)
        self.rx_adr = Signal(WAW)
        self.rx_dat = Signal(32)
        self.tx_re  = Signal()
        self.tx_adr = Signal(WAW)
        self.tx_dat = Signal(32)
        # The dispatcher's word writes into the responses (a reply's header): taken when hd_ok
        self.hd_req = Signal()
        self.hd_adr = Signal(WAW)
        self.hd_dat = Signal(32)
        self.hd_ok  = Signal()

        # The engine (cd)
        self.rq_addr = Signal(BAW)
        self.rq_data = Signal(8)
        self.rs_we   = Signal()
        self.rs_addr = Signal(BAW)
        self.rs_data = Signal(8)

        # # #

        # Both buffers are verilog/slot_ram.v: ports of the same shape as a Memory's
        def ram(name):
            a = Record([("adr", WAW), ("dat_w", 32), ("we", 4), ("dat_r", 32)], name=name + "_a")
            b = Record([("adr", WAW), ("dat_w", 32), ("we", 4), ("dat_r", 32)], name=name + "_b")
            self.specials += Instance("slot_ram", p_AW=WAW,
                i_clk_a=ClockSignal("sys"), i_adr_a=a.adr, i_we_a=a.we, i_dat_w_a=a.dat_w,
                o_dat_r_a=a.dat_r,
                i_clk_b=ClockSignal(cd), i_adr_b=b.adr, i_we_b=b.we, i_dat_w_b=b.dat_w,
                o_dat_r_b=b.dat_r)
            return a, b
        req_a, req_b = ram("req")
        resp_a, resp_b = ram("resp")
        self.comb += req_b.we.eq(0)

        # The engine's side
        lane = Signal(2)
        bytes_r = Array(req_b.dat_r[8 * i:8 * i + 8] for i in range(4))
        self.comb += [
            req_b.adr.eq(self.rq_addr[2:]),
            resp_b.adr.eq(self.rs_addr[2:]),
            resp_b.dat_w.eq(Replicate(self.rs_data, 4)),
            resp_b.we.eq(Mux(self.rs_we, 1 << self.rs_addr[:2], 0)),
        ]
        getattr(self.sync, cd).__iadd__([
            lane.eq(self.rq_addr[:2]),
            self.rq_data.eq(bytes_r[lane]),
        ])

        # The CPU's side: a request word every cycle the receiver or sender leaves the port free,
        # answered the next
        bus = self.bus
        is_resp = bus.adr[WAW]
        want = bus.cyc & bus.stb & ~bus.ack
        self.comb += self.hd_ok.eq(self.hd_req & ~self.tx_re)
        issue = want & Mux(is_resp, ~self.tx_re & ~self.hd_req, self.rx_we == 0)
        was_resp = Signal()
        self.comb += [
            req_a.adr.eq(Mux(self.rx_we != 0, self.rx_adr, bus.adr[:WAW])),
            req_a.dat_w.eq(Mux(self.rx_we != 0, self.rx_dat, bus.dat_w)),
            req_a.we.eq(Mux(self.rx_we != 0, self.rx_we,
                            Mux(issue & bus.we & ~is_resp, bus.sel, 0))),
            resp_a.adr.eq(Mux(self.tx_re, self.tx_adr, Mux(self.hd_req, self.hd_adr, bus.adr[:WAW]))),
            resp_a.dat_w.eq(Mux(self.hd_req, self.hd_dat, bus.dat_w)),
            resp_a.we.eq(Mux(self.hd_ok, 0xF, Mux(issue & bus.we & is_resp, bus.sel, 0))),
            self.tx_dat.eq(resp_a.dat_r),
            bus.dat_r.eq(Mux(was_resp, resp_a.dat_r, req_a.dat_r)),
        ]
        self.sync += [
            bus.ack.eq(issue),
            was_resp.eq(is_resp),
        ]


class Receiver(LiteXModule):
    """Datagrams from the UDP port into their slots' request entries.

    A datagram for a slot with a free entry goes into it, and a descriptor
    of it into a queue the dispatcher reads; one for a slot with none, or when
    the queue is full, is dropped whole, and the host's retransmission brings
    it again.  Its header is checked here only as far as finding the slot:
    the dispatcher looks at `ok` and the rest."""

    def __init__(self, rx, buffers):
        n = len(buffers)
        slot_bits = max(1, (n - 1).bit_length())

        self.desc = desc = stream.SyncFIFO([
            ("slot", 3), ("entry", EB), ("length", 11), ("ok", 1), ("sync", 1),
            ("seq", 16), ("port", 16), ("ip", 32), ("cmd", 8)], 16)
        self.free = free = Array(Signal(ENTRIES, reset=(1 << ENTRIES) - 1, name=f"free{i}")
                                 for i in range(n))
        self.release_slot = Signal(3)                 # free these entries (the dispatcher)
        self.release_mask = Signal(ENTRIES)
        self.release      = Signal()
        self.dropped      = Signal(32)

        # # #

        pos      = Signal(11)                          # byte of the datagram
        length   = Signal(11)                          # command bytes stored
        slot     = Signal(3)
        entry    = Signal(EB)
        claimed  = Signal()
        magic_ok = Signal()
        ver_ok   = Signal()
        sync     = Signal()
        seq      = Signal(16)
        over     = Signal()
        wbuf     = Signal(32)
        wbe      = Signal(4)
        cmd      = Signal(8)                           # the first command byte

        # The entry to claim: the lowest free one of the slot named in byte 1
        b1_slot = Signal(3)
        f = Signal(ENTRIES)
        first_free = Signal(EB)
        # The lowest free entry: a priority encoder
        lowest = 0
        for e in reversed(range(ENTRIES)):
            lowest = Mux(f[e], e, lowest)
        self.comb += [
            b1_slot.eq(rx.data[4:7]),
            f.eq(free[b1_slot[:slot_bits]]),
            first_free.eq(lowest),
        ]
        can_claim = (b1_slot < n) & (f != 0) & desc.sink.ready

        # Storing a command byte: into the word being built, written at its last lane or the end
        lane = length[:2]
        storing = rx.valid & claimed & (pos >= HEADER) & ~over & (length < MAX_PACKET)
        word = Signal(32)
        be = Signal(4)
        self.comb += [word.eq(wbuf), be.eq(wbe)]
        for i in range(4):
            self.comb += If(storing & (lane == i), word[8 * i:8 * i + 8].eq(rx.data), be[i].eq(1))
        write = storing & ((lane == 3) | rx.last)
        for i, b in enumerate(buffers):
            self.comb += [
                b.rx_we.eq(Mux(write & (slot == i), be, 0)),
                b.rx_adr.eq(entry * (ENTRY // 4) + length[2:]),
                b.rx_dat.eq(word),
            ]

        claim = Signal(ENTRIES)                        # entries claimed this cycle
        self.comb += rx.ready.eq(1)
        self.sync += [
            If(rx.valid,
                pos.eq(pos + 1),
                If(pos == 0,
                    magic_ok.eq(rx.data == MAGIC),
                    claimed.eq(0), over.eq(0), length.eq(0), wbe.eq(0),
                ),
                If(pos == 1,
                    ver_ok.eq((rx.data & 0x0F) == VERSION),
                    sync.eq(rx.data[7]),
                    slot.eq(b1_slot),
                    entry.eq(first_free),
                    claimed.eq(can_claim),
                ),
                If(pos == 2, seq[8:16].eq(rx.data)),
                If(pos == 3, seq[0:8].eq(rx.data)),
                If(pos == HEADER, cmd.eq(rx.data)),
                If(storing,
                    length.eq(length + 1),
                    wbuf.eq(word),
                    wbe.eq(Mux(write, 0, be)),
                ),
                If(claimed & (pos >= HEADER) & (length == MAX_PACKET), over.eq(1)),
                If(rx.last,
                    pos.eq(0),
                    If(~(claimed | ((pos == 1) & can_claim)),
                        self.dropped.eq(self.dropped + 1)),
                ),
            ),
        ]
        # Free entries: claimed by the receiver, released by the CPU
        self.comb += claim.eq(Mux(rx.valid & (pos == 1) & can_claim, 1 << first_free, 0))
        for i in range(n):
            self.sync += free[i].eq((free[i] & ~Mux(b1_slot == i, claim, 0)) |
                                    Mux(self.release & (self.release_slot == i), self.release_mask, 0))

        # The descriptor, once the datagram is in (or ended early)
        ended_claimed = claimed | ((pos == 1) & can_claim)
        self.comb += [
            desc.sink.valid.eq(rx.valid & rx.last & ended_claimed),
            desc.sink.slot.eq(Mux(pos == 1, b1_slot, slot)),
            desc.sink.entry.eq(Mux(pos == 1, first_free, entry)),
            desc.sink.length.eq(length + storing),
            desc.sink.ok.eq(magic_ok & ver_ok & (pos >= HEADER) & ~over &
                            ((length + storing) != 0)),
            desc.sink.sync.eq(sync),
            desc.sink.seq.eq(seq),
            desc.sink.port.eq(rx.src_port),
            desc.sink.ip.eq(rx.ip_address),
            desc.sink.cmd.eq(Mux(pos == HEADER, rx.data, cmd)),
        ]


class Sender(LiteXModule):
    """Response entries out to the UDP port, each as one datagram: the CPU
    queues the slot, the entry, the length with its header, and where to.
    An entry is pending from then until its last byte is out."""

    def __init__(self, tx, buffers, local_port=DAP_UDP_PORT):
        n = len(buffers)
        self.queue = queue = stream.SyncFIFO([
            ("slot", 3), ("entry", EB), ("length", 11), ("port", 16), ("ip", 32)], 8)
        self.pending = Signal(n * ENTRIES)
        self.sent = Signal(32)

        # # #

        pend_set, pend_clr = Signal(n * ENTRIES), Signal(n * ENTRIES)
        self.sync += self.pending.eq((self.pending | pend_set) & ~pend_clr)
        self.comb += If(queue.sink.valid & queue.sink.ready,
                        pend_set.eq(1 << (queue.sink.slot * ENTRIES + queue.sink.entry)))

        slot, entry = Signal(3), Signal(EB)
        length, port, ip = Signal(11), Signal(16), Signal(32)
        index = Signal(11)
        word = Signal(32)
        re = Signal()
        for i, b in enumerate(buffers):
            self.comb += [
                b.tx_re.eq(re & (slot == i)),
                b.tx_adr.eq(entry * (ENTRY // 4) + index[2:]),
            ]
        dat = Array(b.tx_dat for b in buffers)[slot]

        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            queue.source.ready.eq(1),
            If(queue.source.valid,
                NextValue(slot, queue.source.slot),
                NextValue(entry, queue.source.entry),
                NextValue(length, queue.source.length),
                NextValue(port, queue.source.port),
                NextValue(ip, queue.source.ip),
                NextValue(index, 0),
                NextState("FETCH"),
            ),
        )
        fsm.act("FETCH", re.eq(1), NextState("WORD"))
        fsm.act("WORD", NextValue(word, dat), NextState("SEND"))
        fsm.act("SEND",
            tx.valid.eq(1),
            tx.src_port.eq(local_port),
            tx.dst_port.eq(port),
            tx.ip_address.eq(ip),
            tx.length.eq(length),
            tx.data.eq(Array(word[8 * i:8 * i + 8] for i in range(4))[index[:2]]),
            tx.last.eq(index == length - 1),
            If(tx.ready,
                NextValue(index, index + 1),
                If(tx.last,
                    pend_clr.eq(1 << (slot * ENTRIES + entry)),
                    NextValue(self.sent, self.sent + 1),
                    NextState("IDLE"),
                ).Elif(index[:2] == 3,
                    NextState("FETCH"),
                ),
            ),
        )
        if hasattr(tx, "be"):
            self.comb += tx.be.eq(1)


# The commands a slot's engine runs without the CPU, by the kind of slot
SWD_COMMANDS = (0x05, 0x06, 0x08, 0x12, 0x1D)    # Transfer, TransferBlock, WriteABORT, SWJ_/SWD_Sequence
JTAG_COMMANDS = (0x12, 0x14)                      # SWJ_Sequence, JTAG_Sequence
TRANSFER_ABORT = 0x07


def times_entry(x):
    """x * ENTRY / 4 (a word address), by shifts: a multiply would take a DSP48.  The shifts are
    written as concatenations: LiteX's Verilog writer emits `x <<< 8`, and inside a concatenation
    Verilog gives that x's own width, so a shifted 3-bit entry number came out as 0."""
    assert ENTRY == 1280
    return Cat(Constant(0, 8), x) + Cat(Constant(0, 6), x)


class Transport(LiteXModule):
    """The receiver, the sender, and the dispatcher between them and the slots.

    The dispatcher does for every datagram what the module's description
    says: the next request in sequence (or any, with SYNC) is queued; one
    answered already whose response is still kept is answered again from it,
    never run twice; anything else is dropped, and the host retransmits.
    DAP_TransferAbort acts at once on its slot's engine, takes no place in
    the sequence and is not answered.

    A slot runs its queue in order.  A request that starts with a command for
    the wire goes to the slot's engine, and when the engine is done its reply
    goes out: the CPU never sees it.  Any other request becomes a job for the
    CPU, which answers it (using the engine as it needs) and gives back the
    reply's length.  The dispatcher writes every reply's header and queues it
    for the sender, and keeps the response for answering its request again
    until the slot needs the entry."""

    def __init__(self, udp_port, buffers, slots, local_port=DAP_UDP_PORT):
        self.receiver = rcv = Receiver(udp_port.source, buffers)
        self.sender = snd = Sender(udp_port.sink, buffers, local_port)
        n = len(buffers)
        assert n <= 8 and ENTRIES <= 8
        d = rcv.desc.source
        q = snd.queue.sink

        # The CPU's jobs: `jobs` toggles a slot's bit with each new one; job<i> says which entries
        # and how long; job_done gives back the reply's length (with its header)
        self.jobs = CSRStatus(n)
        for i in range(n):
            setattr(self, f"job{i}", CSRStatus(fields=[CSRField("entry", size=EB), CSRField("out", size=EB, offset=4),
                                                      CSRField("length", size=11, offset=8)], name=f"job{i}"))
        self.job_done = CSRStorage(fields=[CSRField("slot", size=3), CSRField("length", size=11, offset=16)])
        self.stats = CSRStatus(fields=[CSRField("dropped", size=16), CSRField("sent", size=16)])
        self.comb += [self.stats.fields.dropped.eq(rcv.dropped), self.stats.fields.sent.eq(snd.sent)]

        # # #

        def per_slot(width, name, reset=0):
            return Array(Signal(width, reset=reset, name=f"{name}{i}") for i in range(n))

        have_last, last = per_slot(1, "have_last"), per_slot(16, "last")
        qhead, qcount = per_slot(EB, "qhead"), per_slot(EB + 1, "qcount")
        queue = Array(Signal(EB, name=f"queue{i}_{k}") for i in range(n) for k in range(ENTRIES))
        resp_valid, resp_next = per_slot(ENTRIES, "resp_valid"), per_slot(EB, "resp_next")
        busy, cpu = per_slot(1, "busy"), per_slot(1, "cpu")
        cur_e, cur_out, cur_len = per_slot(EB, "cur_e"), per_slot(EB, "cur_out"), per_slot(11, "cur_len")
        seen, abort = per_slot(1, "done_seen"), per_slot(1, "abort")
        job_t = per_slot(1, "job_t")
        done_req, done_ack, done_len = per_slot(1, "jd_req"), per_slot(1, "jd_ack"), per_slot(11, "jd_len")

        done_t = Array(sl.done_t for sl in slots)
        rs_len = Array(sl.rs_len for sl in slots)

        # The CPU's side
        self.comb += self.jobs.status.eq(Cat(*job_t))
        for i in range(n):
            f = getattr(self, f"job{i}").fields
            self.comb += [f.entry.eq(cur_e[i]), f.out.eq(cur_out[i]), f.length.eq(cur_len[i])]
            self.sync += If(self.job_done.re & (self.job_done.fields.slot == i),
                            done_req[i].eq(~done_req[i]), done_len[i].eq(self.job_done.fields.length))

        # What each queued request is, and each kept response: by slot and entry
        meta = Memory(8 + 11 + 16 + 16 + 32, 8 * 8)               # cmd, length, seq, port, ip
        meta_w, meta_r = meta.get_port(write_capable=True), meta.get_port(async_read=True)
        kept = Memory(16 + 11, 8 * 8)                             # seq, length with the header
        kept_w, kept_r = kept.get_port(write_capable=True), kept.get_port(async_read=True)
        self.specials += meta, meta_w, meta_r, kept, kept_w, kept_r
        m_cmd, m_len, m_seq = meta_r.dat_r[0:8], meta_r.dat_r[8:19], meta_r.dat_r[19:35]
        m_port, m_ip = meta_r.dat_r[35:51], meta_r.dat_r[51:83]

        cs = Signal(3)                                            # the slot being served
        sp = Signal(3)                                            # the slot looked at next
        k = Signal(EB)
        fin_len = Signal(11)

        def pending(slot, entry):
            return (snd.pending >> (slot * ENTRIES + entry))[0]

        def release(slot, entry):
            return [rcv.release.eq(1), rcv.release_slot.eq(slot), rcv.release_mask.eq(1 << entry)]

        def engine_command(cmd):
            return Array(reduce(or_, [cmd == c for c in sl.ENGINE_COMMANDS]) for sl in slots)[cs]

        tail = Signal(EB + 1)
        self.comb += tail.eq(qhead[d.slot] + qcount[d.slot])
        tail_mod = Mux(tail >= ENTRIES, tail - ENTRIES, tail)
        in_seq = d.sync | ~have_last[d.slot] | (d.seq == last[d.slot] + 1)
        head_e = queue[cs * ENTRIES + qhead[cs]]
        out = resp_next[cs]
        go = Signal()

        for i, sl in enumerate(slots):
            self.comb += [
                sl.hw_go.eq(go & (cs == i)),
                sl.hw_rq_base.eq(Cat(Constant(0, 2), times_entry(head_e))),
                sl.hw_rs_base.eq(Cat(Constant(0, 2), times_entry(out)) + HEADER),
                sl.hw_abort.eq(abort[i]),
            ]

        self.fsm = fsm = FSM(reset_state="IDLE")
        next_sp = Mux(sp == n - 1, 0, sp + 1)
        fsm.act("IDLE",
            NextValue(sp, next_sp),
            If(d.valid,
                NextValue(cs, d.slot), NextState("RX"),
            ).Elif(done_req[sp] != done_ack[sp],
                NextValue(cs, sp), NextValue(fin_len, done_len[sp]),
                NextValue(done_ack[sp], done_req[sp]), NextState("HEADER"),
            ).Elif(busy[sp] & ~cpu[sp] & (done_t[sp] != seen[sp]),
                NextValue(cs, sp), NextValue(seen[sp], done_t[sp]),
                NextValue(fin_len, HEADER + rs_len[sp]), NextState("HEADER"),
            ).Elif(~busy[sp] & (qcount[sp] != 0),
                NextValue(cs, sp), NextState("START"),
            ),
        )

        # A datagram's descriptor: queued, answered again, an abort, or dropped
        fsm.act("RX",
            meta_w.adr.eq(Cat(d.entry, d.slot)),
            meta_w.dat_w.eq(Cat(d.cmd, d.length, d.seq, d.port, d.ip)),
            If(~d.ok | (d.cmd == TRANSFER_ABORT),
                *release(d.slot, d.entry),
                If(d.ok, NextValue(abort[d.slot], 1)),
                d.ready.eq(1), NextState("IDLE"),
            ).Elif(in_seq,
                meta_w.we.eq(1),
                NextValue(queue[d.slot * ENTRIES + tail_mod], d.entry),
                NextValue(qcount[d.slot], qcount[d.slot] + 1),
                NextValue(last[d.slot], d.seq), NextValue(have_last[d.slot], 1),
                If(d.sync, NextValue(resp_valid[d.slot], 0)),
                d.ready.eq(1), NextState("IDLE"),
            ).Else(
                NextValue(k, 0), NextState("REPLAY"),
            ),
        )
        kept_seq, kept_len = kept_r.dat_r[0:16], kept_r.dat_r[16:27]
        fsm.act("REPLAY",
            kept_r.adr.eq(Cat(k, d.slot)),
            If((resp_valid[d.slot] >> k)[0] & (kept_seq == d.seq),
                If(pending(d.slot, k),
                    # On its way out still: once is enough
                    *release(d.slot, d.entry), d.ready.eq(1), NextState("IDLE"),
                ).Else(
                    q.valid.eq(1), q.slot.eq(d.slot), q.entry.eq(k), q.length.eq(kept_len),
                    q.port.eq(d.port), q.ip.eq(d.ip),
                    If(q.ready, *release(d.slot, d.entry), d.ready.eq(1), NextState("IDLE")),
                ),
            ).Elif(k == ENTRIES - 1,
                *release(d.slot, d.entry), d.ready.eq(1), NextState("IDLE"),
            ).Else(
                NextValue(k, k + 1),
            ),
        )

        # The slot's next request: to its engine, or a job for the CPU, once its response entry is
        # out of the sender's way
        fsm.act("START",
            meta_r.adr.eq(Cat(head_e, cs)),
            If(~pending(cs, out),
                NextValue(cur_e[cs], head_e), NextValue(cur_out[cs], out), NextValue(cur_len[cs], m_len),
                NextValue(resp_valid[cs], resp_valid[cs] & ~(1 << out)),
                NextValue(resp_next[cs], Mux(out == ENTRIES - 1, 0, out + 1)),
                NextValue(busy[cs], 1), NextValue(abort[cs], 0),
                If(engine_command(m_cmd),
                    go.eq(1),
                    NextValue(cpu[cs], 0), NextValue(seen[cs], done_t[cs]),
                ).Else(
                    NextValue(cpu[cs], 1), NextValue(job_t[cs], ~job_t[cs]),
                ),
            ),
            NextState("IDLE"),
        )

        # The reply: its header, then to the sender; the request's entry is free again
        hd_adr = times_entry(cur_out[cs])
        header = Cat(Constant(MAGIC, 8), Constant(VERSION, 4), cs, Constant(0, 1), m_seq[8:16], m_seq[0:8])
        for i, b in enumerate(buffers):
            self.comb += [
                b.hd_req.eq(fsm.ongoing("HEADER") & (cs == i)),
                b.hd_adr.eq(hd_adr),
                b.hd_dat.eq(header),
            ]
        hd_ok = Array(b.hd_ok for b in buffers)[cs]
        fsm.act("HEADER",
            meta_r.adr.eq(Cat(cur_e[cs], cs)),
            If(hd_ok, NextState("SEND")),
        )
        fsm.act("SEND",
            meta_r.adr.eq(Cat(cur_e[cs], cs)),
            kept_w.adr.eq(Cat(cur_out[cs], cs)), kept_w.dat_w.eq(Cat(m_seq, fin_len)), kept_w.we.eq(1),
            q.valid.eq(1), q.slot.eq(cs), q.entry.eq(cur_out[cs]), q.length.eq(fin_len),
            q.port.eq(m_port), q.ip.eq(m_ip),
            If(q.ready,
                NextValue(resp_valid[cs], resp_valid[cs] | (1 << cur_out[cs])),
                *release(cs, cur_e[cs]),
                NextValue(qhead[cs], Mux(qhead[cs] == ENTRIES - 1, 0, qhead[cs] + 1)),
                NextValue(qcount[cs], qcount[cs] - 1),
                NextValue(busy[cs], 0), NextValue(cpu[cs], 0),
                NextState("IDLE"),
            ),
        )


UART_RX_DEPTH = 512                                   # Bytes from the target the slot keeps
UART_TX_DEPTH = 64                                    # Bytes to it


def add_uart(slot, pads, sys_clk_freq, sim, baudrate=115200):
    """A slot's UART for DAP_UART_*: a byte at a time through registers in sys, buffered both ways.

    Its registers come in this order in both kinds of slot, right after the pins', so the firmware
    reaches every slot's at the same offsets.  pads: tx (to the target), rx (from it); None gives
    plain signals, the simulation's ports, or with no pins for the UART a line that stays idle."""
    slot.uart_tuning = CSRStorage(32, reset=int(baudrate / sys_clk_freq * 2**32))
    slot.uart_ctl = CSRStorage(fields=[CSRField("rx_en", size=1), CSRField("tx_en", size=1, offset=1),
                                       CSRField("rx_flush", size=1, offset=2, pulse=True),
                                       CSRField("tx_flush", size=1, offset=3, pulse=True),
                                       CSRField("lost_clear", size=1, offset=4, pulse=True)])
    slot.uart_tx = CSRStorage(8)                                     # A write sends a byte
    slot.uart_rx = CSRStatus(fields=[CSRField("data", size=8), CSRField("valid", size=1, offset=8)])
    slot.uart_pop = CSRStorage(1)                                    # A write drops uart_rx's byte
    slot.uart_stat = CSRStatus(fields=[CSRField("rx_level", size=10), CSRField("tx_level", size=7, offset=16),
                                       CSRField("lost", size=1, offset=30)])

    if pads is None:
        pads = Record([("tx", 1), ("rx", 1)])
        if not sim:
            slot.comb += pads.rx.eq(1)
    slot.uart_pads = pads
    ctl = slot.uart_ctl.fields
    slot.uart_phy_tx = phy_tx = RS232PHYTX(pads, slot.uart_tuning.storage)
    slot.uart_phy_rx = phy_rx = RS232PHYRX(pads, slot.uart_tuning.storage)
    slot.uart_tx_fifo = tx_fifo = ResetInserter()(stream.SyncFIFO([("data", 8)], UART_TX_DEPTH, buffered=True))
    slot.uart_rx_fifo = rx_fifo = ResetInserter()(stream.SyncFIFO([("data", 8)], UART_RX_DEPTH, buffered=True))
    lost = Signal()
    slot.comb += [
        tx_fifo.reset.eq(ctl.tx_flush), rx_fifo.reset.eq(ctl.rx_flush),
        # The CPU's bytes in; out to the line while sending is on
        tx_fifo.sink.valid.eq(slot.uart_tx.re), tx_fifo.sink.data.eq(slot.uart_tx.storage),
        phy_tx.sink.valid.eq(tx_fifo.source.valid & ctl.tx_en), phy_tx.sink.data.eq(tx_fifo.source.data),
        tx_fifo.source.ready.eq(phy_tx.sink.ready & ctl.tx_en),
        # The line's bytes in while receiving is on, the rest dropped; the CPU takes them one by one
        phy_rx.source.ready.eq(1),
        rx_fifo.sink.valid.eq(phy_rx.source.valid & ctl.rx_en), rx_fifo.sink.data.eq(phy_rx.source.data),
        rx_fifo.source.ready.eq(slot.uart_pop.re),
        slot.uart_rx.fields.data.eq(rx_fifo.source.data), slot.uart_rx.fields.valid.eq(rx_fifo.source.valid),
        slot.uart_stat.fields.rx_level.eq(rx_fifo.level), slot.uart_stat.fields.tx_level.eq(tx_fifo.level),
        slot.uart_stat.fields.lost.eq(lost),
    ]
    # Bytes moving, for the slot's LED
    slot.uart_active = Signal()
    slot.comb += slot.uart_active.eq(phy_tx.sink.valid | phy_rx.source.valid)
    # A byte that found the buffer full is lost, and stays told until cleared or flushed
    slot.sync += If(ctl.lost_clear | ctl.rx_flush, lost.eq(0)).Elif(
        rx_fifo.sink.valid & ~rx_fifo.sink.ready, lost.eq(1))


def add_slot_sources(platform):
    for name in ("swd_exec.v", "swd_phy.v", "jtag_engine.v", "slot_ram.v"):
        platform.add_source(os.path.join(VERILOG, name))


class SWDSlot(LiteXModule):
    """An SWD slot: swd_exec and swd_phy in the `cd` clock, their pins through
    DDR registers, and the registers the CPU drives them with.

    pads: swclk, swdio, nrst, and optionally guard (pins held low)."""

    ENGINE_COMMANDS = SWD_COMMANDS

    def __init__(self, pads, buffers, cd="slot", sim=False, uart_pads=None, sys_clk_freq=50e6):
        # The command: request and response offsets in the slot's buffers; writing it starts it
        self.cmd = CSRStorage(fields=[CSRField("rq_base", size=BAW), CSRField("rs_base", size=BAW, offset=BAW)])
        # Or the dispatcher starts it (sys), and sees it done
        self.hw_go      = Signal()
        self.hw_rq_base = Signal(BAW)
        self.hw_rs_base = Signal(BAW)
        self.hw_abort   = Signal()
        self.done_t     = Signal()
        self.rq_len     = Signal(BAW)
        self.rs_len     = Signal(BAW)
        self.done = CSRStatus(fields=[CSRField("rq_len", size=12), CSRField("rs_len", size=12, offset=12),
                                      CSRField("toggle", size=1, offset=31)])
        # The clock in half cycles of `cd`; `set` toggles with a new one, which calibration follows
        self.clock = CSRStorage(fields=[CSRField("hi", size=10, reset=50), CSRField("lo", size=10, offset=10, reset=50),
                                        CSRField("set", size=1, offset=31)])
        self.config = CSRStorage(fields=[CSRField("idle", size=8), CSRField("turn", size=2, offset=8),
                                         CSRField("dataphase", size=1, offset=10),
                                         CSRField("sample", size=7, offset=16), CSRField("abort", size=1, offset=31)])
        # The pins while the engine does not drive them (DAP_SWJ_Pins), and nRESET, which it never
        # does: at DAP_SWJ_Pins' bits, SWCLK 0, SWDIO 1, nRESET 7, and whether SWDIO is driven
        self.pins = CSRStorage(fields=[CSRField("clk", size=1, reset=1), CSRField("io", size=1, offset=1, reset=1),
                                       CSRField("nrst", size=1, offset=7, reset=1),
                                       CSRField("oe", size=1, offset=8, reset=1)])
        self.pins_in = CSRStatus(fields=[CSRField("clk", size=1), CSRField("io", size=1, offset=1),
                                         CSRField("nrst", size=1, offset=7)])
        add_uart(self, uart_pads, sys_clk_freq, sim)
        # The registers above are a JTAG slot's too, in the same order; these are SWD's own
        self.retry = CSRStorage(fields=[CSRField("wait", size=16, reset=100), CSRField("match", size=16, offset=16)])
        self.cal = CSRStatus(fields=[CSRField("sample", size=6), CSRField("first", size=6, offset=6),
                                     CSRField("count", size=7, offset=12), CSRField("ok", size=1, offset=19),
                                     CSRField("seen", size=1, offset=20), CSRField("busy", size=1, offset=21)])
        self.clock_in_use = CSRStatus(fields=[CSRField("hi", size=10), CSRField("lo", size=10, offset=10)])

        # # #

        clk, rst = ClockSignal(cd), ResetSignal(cd)
        slot_sync = getattr(self.sync, cd)

        # Settings the engine only reads while idle, held in its clock
        turn, idle, dataphase, sample = Signal(2), Signal(8), Signal(), Signal(7)
        slot_sync += [
            turn.eq(self.config.fields.turn), idle.eq(self.config.fields.idle),
            dataphase.eq(self.config.fields.dataphase), sample.eq(self.config.fields.sample),
        ]

        e = {name: Signal(w, name=name) for name, w in [
            ("op_valid", 1), ("op_ready", 1), ("op_kind", 2), ("op_rnw", 1), ("op_apndp", 1),
            ("op_addr", 2), ("op_data", 32), ("op_bits", 5), ("op_retries", 16), ("op_tag", 4),
            ("resume", 1), ("res_valid", 1), ("res_ack", 3), ("res_perr", 1), ("res_data", 32),
            ("res_tag", 4), ("cal_ok", 1), ("cal_seen", 1), ("halted", 1), ("busy", 1), ("res_hold", 1),
            ("phy_hi", 10), ("phy_lo", 10), ("cmd_done", 1), ("rq_len", 12), ("rs_len", 12),
            ("cal_sample", 6), ("cal_first", 6), ("cal_count", 7), ("own", 1)]}
        e["rq_len"], e["rs_len"] = Signal(BAW, name="rq_len"), Signal(BAW, name="rs_len")
        swclk_p, swdo_p, swwr_p, swdi_p = Signal(2), Signal(2), Signal(2), Signal(2)

        # Whoever starts it, the command's fields are held and `req` toggles
        req, rq_base, rs_base = Signal(), Signal(BAW), Signal(BAW)
        self.sync += [
            If(self.hw_go,
                rq_base.eq(self.hw_rq_base), rs_base.eq(self.hw_rs_base), req.eq(~req),
            ).Elif(self.cmd.re,
                rq_base.eq(self.cmd.fields.rq_base), rs_base.eq(self.cmd.fields.rs_base), req.eq(~req),
            ),
        ]
        self.specials += Instance("swd_exec", p_AW=BAW,
            i_clk=clk, i_rst=rst,
            i_cmd_req=req, i_cmd_rq_base=rq_base,
            i_cmd_rs_base=rs_base, o_cmd_done=e["cmd_done"],
            o_cmd_rq_len=e["rq_len"], o_cmd_rs_len=e["rs_len"],
            i_abort=self.config.fields.abort | self.hw_abort,
            i_cfg_retry=self.retry.fields.wait, i_cfg_match_retry=self.retry.fields.match,
            i_cfg_sample=sample, i_clk_set=self.clock.fields.set,
            i_clk_hi=self.clock.fields.hi, i_clk_lo=self.clock.fields.lo,
            o_phy_hi=e["phy_hi"], o_phy_lo=e["phy_lo"],
            o_rq_addr=buffers.rq_addr, i_rq_data=buffers.rq_data,
            o_rs_we=buffers.rs_we, o_rs_addr=buffers.rs_addr, o_rs_data=buffers.rs_data,
            **{("o_" if k in ("op_valid", "op_kind", "op_rnw", "op_apndp", "op_addr", "op_data",
                              "op_bits", "op_retries", "op_tag", "resume", "res_hold") else "i_") + k: e[k]
               for k in ("op_valid", "op_ready", "op_kind", "op_rnw", "op_apndp", "op_addr",
                         "op_data", "op_bits", "op_retries", "op_tag", "resume", "res_hold", "res_valid",
                         "res_ack", "res_perr", "res_data", "res_tag", "cal_ok", "cal_seen")})

        self.specials += Instance("swd_phy",
            i_clk=clk, i_rst=rst, o_own=e["own"],
            o_swclk=swclk_p, o_swdo=swdo_p, o_swwr=swwr_p, i_swdi=swdi_p,
            i_cfg_hi=e["phy_hi"], i_cfg_lo=e["phy_lo"], i_cfg_turn=turn, i_cfg_idle=idle,
            i_cfg_dataphase=dataphase, i_cfg_sample=sample,
            o_halted=e["halted"], o_busy=e["busy"],
            o_cal_sample=e["cal_sample"], o_cal_first=e["cal_first"], o_cal_count=e["cal_count"],
            **{("i_" if k in ("op_valid", "op_kind", "op_rnw", "op_apndp", "op_addr", "op_data",
                              "op_bits", "op_retries", "op_tag", "resume", "res_hold") else "o_") + k: e[k]
               for k in ("op_valid", "op_ready", "op_kind", "op_rnw", "op_apndp", "op_addr",
                         "op_data", "op_bits", "op_retries", "op_tag", "resume", "res_hold", "res_valid",
                         "res_ack", "res_perr", "res_data", "res_tag", "cal_ok", "cal_seen")})

        # Status for the CPU, into sys
        done_t = Signal()
        self.specials += MultiReg(e["cmd_done"], done_t)
        self.comb += [
            self.done.fields.toggle.eq(done_t),
            self.done.fields.rq_len.eq(e["rq_len"]),          # stable long before the toggle shows
            self.done.fields.rs_len.eq(e["rs_len"]),
            self.done_t.eq(done_t), self.rq_len.eq(e["rq_len"]), self.rs_len.eq(e["rs_len"]),
        ]
        for field, sig in [("sample", "cal_sample"), ("first", "cal_first"), ("count", "cal_count"),
                           ("ok", "cal_ok"), ("seen", "cal_seen"), ("busy", "busy")]:
            self.specials += MultiReg(e[sig], getattr(self.cal.fields, field))
        # Working, for the slot's LED
        self.activity = Signal()
        self.comb += self.activity.eq(self.cal.fields.busy | self.uart_active)
        self.specials += MultiReg(e["phy_hi"], self.clock_in_use.fields.hi)
        self.specials += MultiReg(e["phy_lo"], self.clock_in_use.fields.lo)

        # The pins: the engine's pairs while it owns them, the CPU's levels otherwise
        own = e["own"]
        p = self.pins.fields
        swclk_o, swdo_o, swwr_o = Signal(2), Signal(2), Signal(2)
        pin_swclk, pin_swdio, pin_oe = Signal(), Signal(), Signal()
        self.specials += [MultiReg(p.clk, pin_swclk, cd), MultiReg(p.io, pin_swdio, cd),
                          MultiReg(p.oe, pin_oe, cd)]
        self.comb += [
            swclk_o.eq(Mux(own, swclk_p, Replicate(pin_swclk, 2))),
            swdo_o.eq(Mux(own, swdo_p, Replicate(pin_swdio, 2))),
            swwr_o.eq(Mux(own, swwr_p, Replicate(pin_oe, 2))),
        ]
        swdio_i = Signal()
        if sim:
            # The simulation's pins are plain signals: the pairs as they are, the line as it is
            self.swclk_pair, self.swdo_pair, self.swwr_pair = swclk_o, swdo_o, swwr_o
            self.swdi_pair = swdi_p
            self.comb += swdio_i.eq(swdi_p[0])
        else:
            def oddr(q, pair):
                self.specials += Instance("ODDR", p_DDR_CLK_EDGE="SAME_EDGE", o_Q=q, i_C=clk, i_CE=1,
                                          i_D1=pair[0], i_D2=pair[1], i_R=0, i_S=0)
            swdo_q, swdt_q = Signal(), Signal()
            oddr(pads.swclk, swclk_o)
            oddr(swdo_q, swdo_o)
            oddr(swdt_q, ~swwr_o)                                   # OBUFT's T: 1 releases
            self.specials += Instance("IOBUF", io_IO=pads.swdio, i_I=swdo_q, i_T=swdt_q, o_O=swdio_i)
            self.specials += Instance("IDDR", p_DDR_CLK_EDGE="SAME_EDGE", o_Q1=swdi_p[0], o_Q2=swdi_p[1],
                                      i_C=clk, i_CE=1, i_D=swdio_i, i_R=0, i_S=0)
            if hasattr(pads, "guard"):
                self.comb += pads.guard.eq(0)

        # nRESET: open drain, low or released to its pull-up
        nrst_i = Signal(reset=1)
        if sim:
            self.nrst = Signal()
            self.comb += [self.nrst.eq(p.nrst), nrst_i.eq(p.nrst)]
        else:
            nrst = TSTriple()
            self.specials += nrst.get_tristate(pads.nrst)
            self.comb += [nrst.o.eq(0), nrst.oe.eq(~p.nrst), nrst_i.eq(nrst.i)]
        self.specials += [MultiReg(swdio_i, self.pins_in.fields.io),
                          MultiReg(nrst_i, self.pins_in.fields.nrst)]
        self.comb += self.pins_in.fields.clk.eq(p.clk)


class JTAGSlot(LiteXModule):
    """A JTAG slot: jtag_engine in the `cd` clock, its pins through DDR
    registers, and the registers the CPU drives it with: the same as an SWD
    slot's as far as both have them, in the same order, so the firmware reaches
    either kind's at the same offsets.

    pads: tck, tms, tdi, tdo, nrst."""

    ENGINE_COMMANDS = JTAG_COMMANDS

    def __init__(self, pads, buffers, cd="slot", sim=False, uart_pads=None, sys_clk_freq=50e6):
        self.cmd = CSRStorage(fields=[CSRField("rq_base", size=BAW), CSRField("rs_base", size=BAW, offset=BAW)])
        self.hw_go      = Signal()
        self.hw_rq_base = Signal(BAW)
        self.hw_rs_base = Signal(BAW)
        self.hw_abort   = Signal()                                   # DAP_TransferAbort: no transfers here
        self.done_t     = Signal()
        self.rq_len     = Signal(BAW)
        self.rs_len     = Signal(BAW)
        self.done = CSRStatus(fields=[CSRField("rq_len", size=12), CSRField("rs_len", size=12, offset=12),
                                      CSRField("toggle", size=1, offset=31)])
        # TCK in half cycles of `cd`, high and low, each at least two
        self.clock = CSRStorage(fields=[CSRField("hi", size=10, reset=50), CSRField("lo", size=10, offset=10, reset=50),
                                        CSRField("set", size=1, offset=31)])
        # TDO taken `sample` half cycles after TCK rises; 0: half way through the high phase
        self.config = CSRStorage(fields=[CSRField("sample", size=7, offset=16)])
        # The pins while the engine is idle (DAP_SWJ_Pins), at its bits: TCK 0, TMS 1, TDI 2, nRESET 7.
        # TDI rests low, as jtag_engine says why.
        self.pins = CSRStorage(fields=[CSRField("clk", size=1, reset=1), CSRField("io", size=1, offset=1, reset=1),
                                       CSRField("tdi", size=1, offset=2, reset=0),
                                       CSRField("nrst", size=1, offset=7, reset=1)])
        self.pins_in = CSRStatus(fields=[CSRField("clk", size=1), CSRField("io", size=1, offset=1),
                                         CSRField("tdi", size=1, offset=2), CSRField("tdo", size=1, offset=3),
                                         CSRField("nrst", size=1, offset=7)])
        add_uart(self, uart_pads, sys_clk_freq, sim)

        # # #

        clk, rst = ClockSignal(cd), ResetSignal(cd)
        tck_p, tms_p, tdi_p, tdo_p = Signal(2), Signal(2), Signal(2), Signal(2)
        cmd_done, busy = Signal(), Signal()
        rq_len, rs_len = Signal(BAW), Signal(BAW)

        # Whoever starts it, the command's fields are held and `req` toggles
        req, rq_base, rs_base = Signal(), Signal(BAW), Signal(BAW)
        self.sync += [
            If(self.hw_go,
                rq_base.eq(self.hw_rq_base), rs_base.eq(self.hw_rs_base), req.eq(~req),
            ).Elif(self.cmd.re,
                rq_base.eq(self.cmd.fields.rq_base), rs_base.eq(self.cmd.fields.rs_base), req.eq(~req),
            ),
        ]
        # A write of the pins toggles `pins_set`; the engine takes their levels when it is idle
        pins_set = Signal()
        self.sync += If(self.pins.re, pins_set.eq(~pins_set))
        p = self.pins.fields
        self.specials += Instance("jtag_engine", p_AW=BAW,
            i_clk=clk, i_rst=rst,
            i_cmd_req=req, i_cmd_rq_base=rq_base, i_cmd_rs_base=rs_base,
            o_cmd_done=cmd_done, o_cmd_rq_len=rq_len, o_cmd_rs_len=rs_len,
            i_clk_hi=self.clock.fields.hi, i_clk_lo=self.clock.fields.lo,
            i_cfg_sample=self.config.fields.sample,
            i_pins_set=pins_set, i_pins_tck=p.clk, i_pins_tms=p.io, i_pins_tdi=p.tdi,
            o_rq_addr=buffers.rq_addr, i_rq_data=buffers.rq_data,
            o_rs_we=buffers.rs_we, o_rs_addr=buffers.rs_addr, o_rs_data=buffers.rs_data,
            o_tck=tck_p, o_tms=tms_p, o_tdi=tdi_p, i_tdo=tdo_p, o_busy=busy)

        # Status for the CPU, into sys
        done_t = Signal()
        self.specials += MultiReg(cmd_done, done_t)
        self.comb += [
            self.done.fields.toggle.eq(done_t),
            self.done.fields.rq_len.eq(rq_len),                   # stable long before the toggle shows
            self.done.fields.rs_len.eq(rs_len),
            self.done_t.eq(done_t), self.rq_len.eq(rq_len), self.rs_len.eq(rs_len),
        ]

        tdo_i = Signal(reset=1)
        if sim:
            self.tck_pair, self.tms_pair, self.tdi_pair, self.tdo_pair = tck_p, tms_p, tdi_p, tdo_p
            self.comb += tdo_i.eq(tdo_p[0])
        else:
            for pad, pair in [(pads.tck, tck_p), (pads.tms, tms_p), (pads.tdi, tdi_p)]:
                self.specials += Instance("ODDR", p_DDR_CLK_EDGE="SAME_EDGE", o_Q=pad, i_C=clk, i_CE=1,
                                          i_D1=pair[0], i_D2=pair[1], i_R=0, i_S=0)
            self.comb += tdo_i.eq(pads.tdo)
            self.specials += Instance("IDDR", p_DDR_CLK_EDGE="SAME_EDGE", o_Q1=tdo_p[0], o_Q2=tdo_p[1],
                                      i_C=clk, i_CE=1, i_D=pads.tdo, i_R=0, i_S=0)

        # nRESET: open drain, low or released to its pull-up
        nrst_i = Signal(reset=1)
        if sim:
            self.nrst = Signal()
            self.comb += [self.nrst.eq(p.nrst), nrst_i.eq(p.nrst)]
        else:
            nrst = TSTriple()
            self.specials += nrst.get_tristate(pads.nrst)
            self.comb += [nrst.o.eq(0), nrst.oe.eq(~p.nrst), nrst_i.eq(nrst.i)]

        # Working, for the slot's LED
        busy_s = Signal()
        self.specials += MultiReg(busy, busy_s)
        self.activity = Signal()
        self.comb += self.activity.eq(busy_s | self.uart_active)

        # The levels: TCK, TMS and TDI as the engine last left them (the CPU's once taken)
        f = self.pins_in.fields
        self.specials += [MultiReg(tck_p[1], f.clk), MultiReg(tms_p[1], f.io), MultiReg(tdi_p[1], f.tdi),
                          MultiReg(tdo_i, f.tdo), MultiReg(nrst_i, f.nrst)]
