"""List a 7-series bitstream's configuration packets (frame data summarised)."""
import struct, sys
REG = {0:"CRC",1:"FAR",2:"FDRI",3:"FDRO",4:"CMD",5:"CTL0",6:"MASK",7:"STAT",8:"LOUT",9:"COR0",
       10:"MFWR",11:"CBC",12:"IDCODE",13:"AXSS",14:"COR1",16:"WBSTAR",17:"TIMER",19:"RBCRC_SW",
       22:"BOOTSTS",24:"CTL1",31:"BSPI"}
CMD = {0:"NULL",1:"WCFG",2:"MFW",3:"LFRM/DGHIGH",4:"RCFG",5:"START",6:"RCAP",7:"RCRC",8:"AGHIGH",
       9:"SWITCH",10:"GRESTORE",11:"SHUTDOWN",12:"GCAPTURE",13:"DESYNC",15:"IPROG",16:"CRCC",
       17:"LTIMER",18:"BSPI_READ",19:"FALL_EDGE"}
def words(path):
    b = open(path, "rb").read()
    i = b.find(b"\xaa\x99\x55\x66")
    assert i >= 0, "no sync word"
    return b, i, [struct.unpack(">I", b[j:j+4])[0] for j in range(i, len(b) - 3, 4)]
if __name__ == "__main__":
    b, off, w = words(sys.argv[1])
    print(f"sync at byte {off}, {len(w)} words")
    k, reg = 1, None
    while k < len(w):
        h = w[k]; t = h >> 29
        if t == 1:
            op = (h >> 27) & 3; reg = (h >> 13) & 0x1F; n = h & 0x7FF
        elif t == 2:
            op = (h >> 27) & 3; n = h & 0x7FFFFFF
        else:
            print(f"{k}: ?? {h:08x}"); k += 1; continue
        data = w[k+1:k+1+n]
        if op == 2:
            name = REG.get(reg, str(reg))
            if n > 4: s = f"{n} words"
            else: s = " ".join(f"{d:08x}" + (f"({CMD.get(d,'?')})" if reg == 4 else "") for d in data)
            print(f"{k}: {'T2 ' if t == 2 else ''}write {name}: {s}")
        elif op == 0 and n == 0 and t == 1:
            pass  # NOOP
        else:
            print(f"{k}: op {op} reg {reg} n {n}")
        k += 1 + n
