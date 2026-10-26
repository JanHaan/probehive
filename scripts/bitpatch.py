"""Rewrite a 7-series bitstream's IDCODE and recompute every CRC check word.

scripts/bitpatch.py IN [OUT] [--idcode 0x0362D093] [--verify]
The CRC is CRC-32C over (register address, word) for every register write, reset
by RCRC; a write to the CRC register is the check (prjxray's icap_crc, UG470)."""
import argparse, struct, sys
sys.path.insert(0, __file__.rsplit("/", 1)[0])
from bitpackets import words
def icap_crc(addr, data, crc):
    poly = 0x82F63B78 << 1; val = (addr << 32) | data
    for _ in range(37):
        if (val & 1) != (crc & 1): crc ^= poly
        val >>= 1; crc >>= 1
    return crc
def walk(w, idcode=None, fix=False):
    """Yield (index, register, value) of each written word; patch in place."""
    k, reg, crc, checks = 1, None, 0, []
    while k < len(w):
        h = w[k]; t = h >> 29
        if t == 1: op, reg, n = (h >> 27) & 3, (h >> 13) & 0x1F, h & 0x7FF
        elif t == 2: op, n = (h >> 27) & 3, h & 0x7FFFFFF
        else: raise SystemExit(f"word {k}: unknown packet {h:08x}")
        if op == 2:
            for j in range(k + 1, k + 1 + n):
                if reg == 12 and idcode is not None: w[j] = idcode
                if reg == 0:
                    checks.append((j, w[j], crc))
                    if fix: w[j] = crc
                    crc = 0
                    continue
                crc = icap_crc(reg, w[j], crc)
                if reg == 4 and w[j] == 7: crc = 0
        k += 1 + n
    return checks
if __name__ == "__main__":
    a = argparse.ArgumentParser(); a.add_argument("inp"); a.add_argument("out", nargs="?")
    a.add_argument("--idcode", type=lambda s: int(s, 0)); a.add_argument("--verify", action="store_true")
    o = a.parse_args()
    b, off, w = words(o.inp)
    checks = walk(list(w))
    for j, have, want in checks:
        print(f"CRC check at word {j}: stream {have:08x}, computed {want:08x}", "ok" if have == want else "MISMATCH")
    if not checks: print("no CRC checks in the stream")
    if o.out:
        checks = walk(w, o.idcode, fix=True)
        body = b"".join(struct.pack(">I", x) for x in w)
        open(o.out, "wb").write(b[:off] + body + b[off + len(body):])
        print(f"wrote {o.out}: IDCODE {o.idcode:#010x}, {len(checks)} CRC words recomputed")
