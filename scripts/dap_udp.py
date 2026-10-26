"""CMSIS-DAP over UDP to the Arty, from the host.

    from dap_udp import DAP
    dap = DAP("10.42.0.50")
    print(dap.info_string(DAP.INFO_PROTOCOL))

Each request carries a sequence number, and the probe runs requests in
that order.  `pipeline` keeps up to `window` of them in flight, by default
and at most the packet count the probe reports: it has room for no more,
and drops what does not fit.  When a reply is missing it resends from the
first unanswered request on (go-back-N), and the probe answers those it
already ran from their stored responses instead of running them twice.

A probe with slots (gateware/arty_slots.py) is several probes on one port:
`slot` picks one, carried in the header's second byte.
"""

import os
import select
import socket
import struct
import time

MAGIC = 0xDA
VERSION = 0x01
SYNC = 0x80
PORT = 1235


class DAPError(Exception):
    pass


class DAP:
    # Commands
    INFO, HOST_STATUS, CONNECT, DISCONNECT = 0x00, 0x01, 0x02, 0x03
    TRANSFER_CONFIGURE, TRANSFER, TRANSFER_BLOCK = 0x04, 0x05, 0x06
    WRITE_ABORT, DELAY, RESET_TARGET = 0x08, 0x09, 0x0A
    SWJ_PINS, SWJ_CLOCK, SWJ_SEQUENCE, SWD_CONFIGURE = 0x10, 0x11, 0x12, 0x13
    SWD_SEQUENCE = 0x1D

    # DAP_Info identifiers
    INFO_VENDOR, INFO_PRODUCT, INFO_SERIAL, INFO_PROTOCOL = 0x01, 0x02, 0x03, 0x04
    INFO_FIRMWARE = 0x09
    INFO_CAPABILITIES, INFO_PACKET_COUNT, INFO_PACKET_SIZE = 0xF0, 0xFE, 0xFF

    # Transfer ACKs
    ACK_OK, ACK_WAIT, ACK_FAULT = 1, 2, 4

    def __init__(self, host, port=PORT, timeout=0.25, attempts=8, window=None, slot=0):
        self.address = (host, port)
        self.timeout = timeout
        self.attempts = attempts
        self.requested_window = window
        self.slot = slot
        self.sequence = int.from_bytes(os.urandom(2), "big")
        self.synced = False
        self.retransmissions = 0
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)

    def close(self):
        self.socket.close()

    def command(self, payload):
        """Sends one CMSIS-DAP command packet, returns its response."""
        return self.pipeline([payload], window=1)[0]

    @property
    def window(self):
        """Requests in flight: what was asked for, at most the probe's packet count."""
        if not hasattr(self, "_window"):
            count = self.info_int(self.INFO_PACKET_COUNT) or 1
            self._window = min(self.requested_window or count, count)
        return self._window

    def pipeline(self, payloads, window=None):
        """Sends CMSIS-DAP command packets, up to `window` in flight, and
        returns their responses in order.  The first request a DAP object
        ever sends starts a new sequence at the probe (SYNC)."""
        # Never more in flight than the probe has room for (one needs no asking)
        window = self.window if window is None else window if window <= 1 else min(window, self.window)
        count = len(payloads)
        first = (self.sequence + 1) & 0xFFFF
        self.sequence = (self.sequence + count) & 0xFFFF
        requests = []
        for i, payload in enumerate(payloads):
            flags = VERSION | self.slot << 4 | (SYNC if not self.synced and i == 0 else 0)
            requests.append(struct.pack(">BBH", MAGIC, flags, (first + i) & 0xFFFF) + bytes(payload))
        self.synced = True

        responses = [None] * count
        answered = 0          # requests before this one have their responses
        sent = 0              # requests before this one were sent since the last rewind
        failures = 0
        deadline = time.monotonic() + self.timeout
        while answered < count:
            while sent < count and sent - answered < window:
                self.socket.sendto(requests[sent], self.address)
                sent += 1
            wait = deadline - time.monotonic()
            if wait > 0 and select.select([self.socket], [], [], wait)[0]:
                reply = self.socket.recv(2048)
                if len(reply) < 4:
                    continue
                magic, flags, sequence = struct.unpack(">BBH", reply[:4])
                index = (sequence - first) & 0xFFFF
                if magic != MAGIC or flags != (VERSION | self.slot << 4) or index >= count or \
                        responses[index] is not None:
                    continue
                response = reply[4:]
                if not response or response[0] != payloads[index][0]:
                    raise DAPError(f"response {response.hex()} to command 0x{payloads[index][0]:02x}")
                responses[index] = response
                while answered < count and responses[answered] is not None:
                    answered += 1
                    failures = 0
                deadline = time.monotonic() + self.timeout
                continue
            # A reply is missing: go back to the first unanswered request.
            failures += 1
            if failures >= self.attempts:
                raise DAPError(f"no reply to command 0x{payloads[answered][0]:02x} "
                               f"after {self.attempts} attempts")
            self.retransmissions += sent - answered
            sent = answered
            deadline = time.monotonic() + self.timeout
        return responses

    # DAP_Info

    def info(self, identifier):
        response = self.command([self.INFO, identifier])
        return response[2:2 + response[1]]

    def info_string(self, identifier):
        return self.info(identifier).rstrip(b"\0").decode(errors="replace")

    def info_int(self, identifier):
        return int.from_bytes(self.info(identifier), "little")

    @property
    def packet_size(self):
        if not hasattr(self, "_packet_size"):
            self._packet_size = self.info_int(self.INFO_PACKET_SIZE)
        return self._packet_size

    # Setup

    def connect_swd(self):
        response = self.command([self.CONNECT, 1])
        if response[1] != 1:
            raise DAPError(f"DAP_Connect answered port {response[1]}")

    def disconnect(self):
        self.command([self.DISCONNECT])

    def swj_clock(self, hz):
        self._ok(self.command([self.SWJ_CLOCK, *struct.pack("<I", hz)]))

    def transfer_configure(self, idle_cycles=0, wait_retry=64, match_retry=0):
        self._ok(self.command([self.TRANSFER_CONFIGURE, idle_cycles,
                               *struct.pack("<HH", wait_retry, match_retry)]))

    def swd_configure(self, turnaround=1, data_phase=False):
        self._ok(self.command([self.SWD_CONFIGURE, (turnaround - 1) | (data_phase << 2)]))

    # Line sequences

    def swj_sequence(self, bits, count):
        """`count` bits of `bits` (bytes, LSB first) on SWDIO/TMS."""
        self._ok(self.command([self.SWJ_SEQUENCE, count & 0xFF, *bits]))

    def swd_sequence(self, sequences):
        """sequences: (cycles, bytes to send) or (cycles, None) to read.
        Returns the bytes read, one entry per read sequence."""
        packet = [self.SWD_SEQUENCE, len(sequences)]
        for cycles, data in sequences:
            packet.append((cycles & 0x3F) | (0x80 if data is None else 0))
            if data is not None:
                packet.extend(data)
        response = self.command(packet)
        self._ok(response)
        reads, position = [], 2
        for cycles, data in sequences:
            if data is None:
                length = ((cycles or 64) + 7) // 8
                reads.append(response[position:position + length])
                position += length
        return reads

    # Transfers: packet builders and result parsers, so that callers can
    # pipeline them, and the one-request forms on top.

    @staticmethod
    def request(ap, read, register):
        return (1 if ap else 0) | (2 if read else 0) | (register & 0xC)

    def transfer_packet(self, operations, dap_index=0):
        """DAP_Transfer.  operations: (ap, register, value) to write,
        (ap, register, None) to read."""
        packet = [self.TRANSFER, dap_index, len(operations)]
        for ap, register, value in operations:
            packet.append(self.request(ap, value is None, register))
            if value is not None:
                packet.extend(struct.pack("<I", value))
        return bytes(packet)

    def transfer_result(self, response, operations):
        """The values a DAP_Transfer read; raises on a bad ACK."""
        done, ack = response[1], response[2]
        if done != len(operations) or ack & 0x07 != self.ACK_OK:
            raise DAPError(f"transfer stopped after {done} of {len(operations)}: ack {ack:#x}")
        reads = response[3:]
        return [struct.unpack("<I", reads[i:i + 4])[0] for i in range(0, len(reads) - 3, 4)]

    def transfer(self, operations, dap_index=0):
        return self.transfer_result(self.command(self.transfer_packet(operations, dap_index)),
                                    operations)

    def block_packet(self, ap, register, data, dap_index=0):
        """DAP_TransferBlock: `data` a word count to read, or words to write."""
        read = isinstance(data, int)
        count = data if read else len(data)
        packet = [self.TRANSFER_BLOCK, dap_index, *struct.pack("<H", count),
                  self.request(ap, read, register)]
        if not read:
            for value in data:
                packet.extend(struct.pack("<I", value))
        return bytes(packet)

    def block_result(self, response, data):
        read = isinstance(data, int)
        count = data if read else len(data)
        done, ack = struct.unpack("<H", response[1:3])[0], response[3]
        if done != count or ack & 0x07 != self.ACK_OK:
            raise DAPError(f"block transfer stopped after {done} of {count}: ack {ack:#x}")
        if not read:
            return None
        return list(struct.unpack(f"<{count}I", response[4:4 + 4 * count]))

    def transfer_block(self, ap, register, data, dap_index=0):
        return self.block_result(self.command(self.block_packet(ap, register, data, dap_index)),
                                 data)

    def read_dp(self, register):
        return self.transfer([(False, register, None)])[0]

    def write_dp(self, register, value):
        self.transfer([(False, register, value)])

    def read_ap(self, register):
        return self.transfer([(True, register, None)])[0]

    def write_ap(self, register, value):
        self.transfer([(True, register, value)])

    @staticmethod
    def _ok(response):
        if len(response) < 2 or response[1] != 0:
            raise DAPError(f"command 0x{response[0]:02x} failed: {response.hex()}")


# ADIv5.2 dormant-to-SWD: 8 high, the selection alert, 4 low, the SWD
# activation code, 8 high, a line reset and 2 idle cycles; 224 bits, as
# OpenOCD's swd_seq_dormant_to_swd.
DORMANT_TO_SWD = bytes([
    0xff,
    0x92, 0xf3, 0x09, 0x62, 0x95, 0x2d, 0x85, 0x86,
    0xe9, 0xaf, 0xdd, 0xe3, 0xa2, 0x0e, 0xbc, 0x19,
    0xa0, 0xf1, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0x00,
])
LINE_RESET = bytes([0xff] * 7 + [0x00])

# RP2040's SWD multidrop target IDs.
RP2040_CORE0 = 0x01002927
RP2040_CORE1 = 0x11002927
RP2040_RESCUE = 0xf1002927


def parity(value):
    return bin(value).count("1") & 1


def select_target(dap, targetsel):
    """A DP TARGETSEL write.  No target drives its ACK, so it goes out as one
    run of bits, all driven, as probe-rs sends it: the request, five ones
    over the turnaround and the ACK, then the 32 data bits and parity."""
    data = 0x1F99 | targetsel << 13 | parity(targetsel) << 45
    dap.swj_sequence(data.to_bytes(6, "little"), 48)


def attach_rp2040(dap, clock_hz, targetsel=RP2040_CORE0):
    """SWD on, the RP2040 woken from dormant and one core selected;
    returns its DPIDR."""
    dap.connect_swd()
    dap.swj_clock(clock_hz)
    dap.transfer_configure(idle_cycles=0, wait_retry=64)
    dap.swd_configure(turnaround=1)
    dap.swj_sequence(DORMANT_TO_SWD, 224)
    dap.swj_sequence(LINE_RESET, 64)
    select_target(dap, targetsel)
    return dap.read_dp(0x0)


# Memory access through a MEM-AP (ADIv5).

DP_CTRL_STAT, DP_SELECT, DP_RDBUFF = 0x4, 0x8, 0xC
AP_CSW, AP_TAR, AP_DRW, AP_IDR = 0x00, 0x04, 0x0C, 0xFC
CSW_32BIT_INCREMENT = 0xA2000012   # 32-bit, auto-increment, privileged data access
TAR_INCREMENT_WRAP = 1024          # ADIv5 guarantees auto-increment only within 1 KiB


class Memory:
    def __init__(self, dap, ap=0):
        self.dap = dap
        self.ap = ap
        self.bank = None

    def power_up(self):
        self.dap.write_dp(0x0, 0x1E)                      # ABORT: clear sticky errors
        self.dap.write_dp(DP_CTRL_STAT, 0x50000000)       # CSYSPWRUPREQ | CDBGPWRUPREQ
        for _ in range(100):
            if self.dap.read_dp(DP_CTRL_STAT) & 0xA0000000 == 0xA0000000:
                break
        else:
            raise DAPError("debug power-up not acknowledged")
        self.bank = None
        self.select(0)
        self.dap.write_ap(AP_CSW, CSW_32BIT_INCREMENT)

    def select(self, register):
        bank = register & 0xF0
        if bank != self.bank:
            self.dap.write_dp(DP_SELECT, (self.ap << 24) | bank)
            self.bank = bank

    def idr(self):
        self.select(AP_IDR)
        value = self.dap.read_ap(AP_IDR & 0xC)
        self.select(0)
        return value

    def read32(self, address):
        return self.dap.transfer([(True, AP_TAR, address), (True, AP_DRW, None)])[0]

    def write32(self, address, value):
        self.dap.transfer([(True, AP_TAR, address), (True, AP_DRW, value)])

    @staticmethod
    def _segments(address, words):
        """(address, count) pieces within one 1 KiB auto-increment page."""
        while words:
            count = min(words, (TAR_INCREMENT_WRAP - address % TAR_INCREMENT_WRAP) // 4)
            yield address, count
            address += 4 * count
            words -= count

    def read_block(self, address, words, window=None):
        """Per page a DAP_Transfer that sets TAR and reads the first word,
        then DAP_TransferBlocks for the rest; all of them pipelined."""
        dap = self.dap
        most = (dap.packet_size - 4) // 4
        packets, results = [], []
        for segment, count in self._segments(address, words):
            operations = [(True, AP_TAR, segment), (True, AP_DRW, None)]
            packets.append(dap.transfer_packet(operations))
            results.append((dap.transfer_result, operations))
            left = count - 1
            while left:
                take = min(left, most)
                packets.append(dap.block_packet(True, AP_DRW, take))
                results.append((dap.block_result, take))
                left -= take
        out = []
        for response, (parse, argument) in zip(dap.pipeline(packets, window), results):
            out.extend(parse(response, argument))
        return out

    def write_block(self, address, values, window=None):
        """As read_block: TAR and the first word(s) in a DAP_Transfer, the
        rest of the page in DAP_TransferBlocks."""
        dap = self.dap
        most = (dap.packet_size - 5) // 4
        packets, results = [], []
        position = 0
        for segment, count in self._segments(address, len(values)):
            head = max(1, count - most)
            operations = [(True, AP_TAR, segment)] + \
                [(True, AP_DRW, value) for value in values[position:position + head]]
            packets.append(dap.transfer_packet(operations))
            results.append((dap.transfer_result, operations))
            position += head
            left = count - head
            while left:
                take = min(left, most)
                chunk = values[position:position + take]
                packets.append(dap.block_packet(True, AP_DRW, chunk))
                results.append((dap.block_result, chunk))
                position += take
                left -= take
        for response, (parse, argument) in zip(dap.pipeline(packets, window), results):
            parse(response, argument)
