#!/usr/bin/env python3
"""OpenOCD's CMSIS-DAP TCP backend in front of the Arty's UDP probe.

    python3 scripts/dap_bridge.py [--listen 127.0.0.1:4441] [--host 10.42.0.50] [--slots 1]

    openocd -f openocd/arty-probe.cfg -f target/rp2040.cfg
    probe-rs info --probe 0000:0001:127.0.0.1:4441

OpenOCD frames each CMSIS-DAP packet on its TCP stream with an 8-byte header
(src/jtag/drivers/cmsis_dap_tcp.c): the signature "DAP\\0", the payload
length and the packet type (1 request, 2 response), little endian.  The
bridge passes each request to the probe as one datagram and frames the
reply.  OpenOCD may queue as many requests as the probe's packet count
before it reads a reply; the bridge sends whatever has arrived together, in
flight at once (scripts/dap_udp.py's pipeline).

With --slots N the bridge listens on N ports, the given one and those after
it, slot 0 on the first: one client a port, each port a probe of its own.

Each slot's UART is a TCP port too, from --uart-listen (4461) on: a client
there gets the target's bytes and sends it its own, through the CMSIS-DAP
UART commands (8N1 at --uart-baud, one rate for every slot or one a slot,
comma separated).  The debugger's port and the UART's share the slot's one
CMSIS-DAP client, so their requests keep the slot's sequence between them.

    nc 127.0.0.1 4461                       # slot 0's console
"""

import argparse
import select
import socket
import struct
import threading
import time

from dap_udp import DAP, DAPError

SIGNATURE = 0x00504144
REQUEST, RESPONSE = 0x01, 0x02
HEADER = struct.Struct("<IHBB")


def receive_exactly(connection, count):
    data = b""
    while len(data) < count:
        chunk = connection.recv(count - len(data))
        if not chunk:
            raise ConnectionError("closed")
        data += chunk
    return data


def receive_request(connection):
    signature, length, kind, _ = HEADER.unpack(receive_exactly(connection, HEADER.size))
    if signature != SIGNATURE or kind != REQUEST:
        raise ConnectionError(f"not a CMSIS-DAP request: {signature:#x} type {kind}")
    return receive_exactly(connection, length)


class Slot:
    """A slot's CMSIS-DAP client, for the debugger's port and the UART's, one request batch at a time."""

    def __init__(self, host, slot):
        self.dap = DAP(host, slot=slot)
        self.lock = threading.Lock()

    def pipeline(self, requests):
        with self.lock:
            try:
                return self.dap.pipeline(requests)
            except DAPError:
                self.dap.synced = False                   # in step again with the next request
                raise

    def resync(self):
        """Start a new sequence with the next request: another host may have used the slot since."""
        with self.lock:
            self.dap.synced = False

    def command(self, request):
        return self.pipeline([request])[0]

    def packet_size(self):
        with self.lock:
            return self.dap.packet_size


def serve(connection, probe):
    probe.resync()
    dap = probe.dap
    while True:
        # Wait for one request, then take every one that is already here.
        requests = [receive_request(connection)]
        while len(requests) < dap.window and select.select([connection], [], [], 0)[0]:
            requests.append(receive_request(connection))
        try:
            responses = probe.pipeline(requests)
        except DAPError as error:
            print(f"probe: {error}")
            responses = [bytes([request[0], 0xFF]) for request in requests]
        connection.sendall(b"".join(HEADER.pack(SIGNATURE, len(r), RESPONSE, 0) + r
                                    for r in responses))


def listen(address, port, probe, slot):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((address, port))
    server.listen(1)
    print(f"listening on {address}:{port} for slot {slot}", flush=True)
    while True:
        connection, peer = server.accept()
        connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        print(f"{peer[0]}:{peer[1]} connected to slot {slot}", flush=True)
        try:
            serve(connection, probe)
        except ConnectionError as error:
            print(f"{peer[0]}:{peer[1]} gone ({error}); {probe.dap.retransmissions} requests resent",
                  flush=True)
        finally:
            connection.close()


def uart_serve(connection, probe, baud):
    """The target's bytes to the client, the client's to the target, until either side ends."""
    probe.resync()
    size = probe.packet_size()
    for request in (bytes([0x1F, 2]), bytes([0x20, 0]) + struct.pack("<I", baud), bytes([0x22, 0x55])):
        response = probe.command(request)
        if response[1] != 0 or (request[0] == 0x20 and struct.unpack_from("<I", response, 2)[0] == 0):
            raise ConnectionError(f"the slot refused {request.hex()}: {response.hex()}")
    connection.setblocking(False)
    pending = b""
    try:
        while True:
            try:
                data = connection.recv(4096)
                if not data:
                    raise ConnectionError("closed")
                pending += data
            except BlockingIOError:
                pass
            tx, pending = pending[:size - 5], pending[size - 5:]
            response = probe.command(bytes([0x21]) + struct.pack("<HH", size - 6, len(tx)) + tx)
            taken, got = struct.unpack_from("<HH", response, 2)
            pending = tx[taken:] + pending                              # what did not fit: again
            if got:
                connection.setblocking(True)
                connection.sendall(response[6:6 + got])
                connection.setblocking(False)
            if not got and not pending:
                time.sleep(0.002)
    finally:
        try:
            probe.pipeline([bytes([0x22, 0x22]), bytes([0x1F, 0])])
        except DAPError:
            pass


def uart_listen(address, port, probe, slot, baud):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((address, port))
    server.listen(1)
    print(f"listening on {address}:{port} for slot {slot}'s UART at {baud} baud", flush=True)
    while True:
        connection, peer = server.accept()
        connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        print(f"{peer[0]}:{peer[1]} connected to slot {slot}'s UART", flush=True)
        try:
            uart_serve(connection, probe, baud)
        except (ConnectionError, DAPError) as error:
            print(f"{peer[0]}:{peer[1]} gone from slot {slot}'s UART ({error})", flush=True)
        finally:
            connection.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--listen", default="127.0.0.1:4441")
    parser.add_argument("--host", default="10.42.0.50")
    parser.add_argument("--slots", type=int, default=1, help="slots, a port each")
    parser.add_argument("--uart-listen", default="127.0.0.1:4461", help="slot 0's UART; 'none': no UARTs")
    parser.add_argument("--uart-baud", default="115200", help="one rate, or one a slot, comma separated")
    args = parser.parse_args()

    address, port = args.listen.rsplit(":", 1)
    probes = [Slot(args.host, slot) for slot in range(args.slots)]
    threads = [threading.Thread(target=listen, args=(address, int(port) + slot, probes[slot], slot),
                                daemon=True) for slot in range(args.slots)]
    if args.uart_listen != "none":
        uart_address, uart_port = args.uart_listen.rsplit(":", 1)
        bauds = [int(b) for b in args.uart_baud.split(",")]
        bauds += [bauds[-1]] * (args.slots - len(bauds))
        threads += [threading.Thread(target=uart_listen, daemon=True,
                                     args=(uart_address, int(uart_port) + slot, probes[slot], slot, bauds[slot]))
                    for slot in range(args.slots)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()


if __name__ == "__main__":
    main()
