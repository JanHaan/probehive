# probehive

A Digilent Arty A7 as a six-target CMSIS-DAP debug probe for CI, over
Ethernet: four SWD slots and two JTAG slots, a UART and a reset line for
every target, and twelve GPIO.  probe-rs and OpenOCD drive it; each slot is
a probe of its own, and all six work at once.

```
 probe-rs, OpenOCD, GDB, a CI harness
          │  TCP: a debug port and a UART port a slot (scripts/dap_bridge.py)
          │  UDP, CMSIS-DAP 2.1 packets in datagrams, several in flight
 ┌────────┴────────────────────────────────────────────────────────┐
 │ Arty A7: LiteEth ── dispatcher ── slot 0..5 engines ── pins      │
 │                         │                                        │
 │                     VexRiscv firmware (everything not on the wire)│
 └──────────────────────────────────────────────────────────────────┘
      4 × SWD (RP2040 and other Cortex-M), 2 × JTAG (ESP32, RISC-V)
```

Tested with Raspberry Pi Picos (RP2040, SWD multidrop: probe-rs and
OpenOCD, both cores) and a classic ESP32 (Xtensa over JTAG: probe-rs and
OpenOCD).

## What it does

- **SWD**, up to 50 MHz.  DAP_Transfer, DAP_TransferBlock and DAP_WriteABORT
  run in gateware as ARM's CMSIS-DAP reference (DAP.c) runs them: posted AP
  reads, value match, WAIT retries, the end of a command at a FAULT.  The
  engine reads SWDIO a calibrated delay after the edge that sent it, so the
  round trip through cables and level shifting may exceed half a period.
  DAP_SWJ_Sequence and DAP_SWD_Sequence land on the cycle, so multidrop
  TARGETSEL works whichever way the host sends it.
- **JTAG**, up to 50 MHz on the wire (an ESP32 runs to 33 MHz here; it is
  rated for 20).  DAP_JTAG_Sequence and DAP_SWJ_Sequence run in gateware;
  that is all probe-rs and OpenOCD need for Xtensa and RISC-V.
- **UART** a slot, 8N1 from 300 baud to 3.1 Mbaud, buffered 512 bytes from
  the target and 64 to it, through the CMSIS-DAP 2.1 UART commands.  The
  bridge makes each a TCP port.
- **nRESET** a slot, open drain, through DAP_SWJ_Pins.
- **GPIO**: twelve shield pins for BOOT, BOOTSEL, RUN or power switches,
  set and read over Etherbone (`scripts/gpio.py`).
- **A LED a slot**, lit while it works.

The host keeps as many requests in flight as a slot has entries (six); a
lost datagram is retransmitted, and a request answered already is answered
again from its kept response, never run twice.

## Hardware

An Arty A7-35T, a 10/100 Ethernet link to the host,
and the USB cable for loading the bitstream.  The targets are powered on
their own; leave the Arty's 3.3 V pins unconnected.

| slot | kind | clock | data | TDI | TDO | nRESET | UART TX → target | target → UART RX |
|------|------|-------|------|-----|-----|--------|------------------|------------------|
| 0    | SWD  | JA1   | JA2  |     |     | IO26   | IO0              | IO1              |
| 1    | SWD  | JA3   | JA4  |     |     | IO27   | IO2              | IO3              |
| 2    | SWD  | JA7   | JA8  |     |     | IO28   | IO4              | IO5              |
| 3    | SWD  | JA9   | JA10 |     |     | IO29   | IO6              | IO7              |
| 4    | JTAG | JD1   | JD2  | JD3 | JD4 | IO30   | IO8              | IO9              |
| 5    | JTAG | JD7   | JD8  | JD9 | JD10| IO31   | IO10             | IO11             |

GPIO: IO32 to IO41, IO12 and IO13 (the shield header's names).  JA and JD
have 200 ohm series resistors, which terminate jumper wires at the source;
that is what keeps SWD clean at 50 MHz.  Run a ground wire with every
target's cable.  The LEDs: slots 0-3 on LD4-LD7, slots 4 and 5 on LD0 and
LD1 (green).

An ESP32 reads MTDI (GPIO12, its TDI) as it leaves reset to choose its
flash supply: high means 1.8 V, and a module with 3.3 V flash does not
boot.  So a JTAG slot rests TDI low, and pulling its nRESET low takes TDI
low too.

The Arty's PHY runs at 10 or 100 Mbit/s; a host port that links only at 1
or 10 Gbit/s needs a switch between them.  The probe answers at
`10.42.0.50` (`--ip` changes it), with no DHCP.  `openFPGALoader -b arty`
loads the bitstream over the board's FT2232; on Linux it needs a udev rule
for `0403:6010`, for example

```
SUBSYSTEM=="usb", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6010", MODE="0660", GROUP="dialout", TAG+="uaccess"
```

## Building and loading

```sh
nix develop                                         # LiteX, the toolchains, openFPGALoader
python3 gateware/arty_slots.py --build              # Vivado: build/slots_ci/gateware/
python3 gateware/arty_slots.py --load --targets 0   # into the FPGA's SRAM, then the self-check
```

`--build` fails when timing is not met.  `--load` runs
`scripts/self_check.py`, which checks the register bus, the CPU's memory,
every slot's buffers, engine and UART, and with `--targets` a target on a
slot: an RP2040's attach and a memory test on an SWD slot, the first TAP's
IDCODE on a JTAG slot.  A bitstream that fails it is not to be used.

The design builds with AMD's Vivado by default (see [Vivado](#vivado)), or
with the open toolchain, `--toolchain openxc7`: Yosys, nextpnr and Project
X-Ray through openXC7, no account or license, about 35 minutes a build
instead of 8, and the 100 MHz slot clock met on some placements (`--seed`)
rather than all.

## Using it

```sh
python3 scripts/dap_bridge.py --slots 6 &           # debug ports 4441-4446, UARTs 4461-4466
```

probe-rs (master, for its CMSIS-DAP TCP transport):

```sh
probe-rs info --probe 0000:0001:127.0.0.1:4441
probe-rs download --probe 0000:0001:127.0.0.1:4441 --chip RP2040 --speed 25000 \
    --binary-format bin --base-address 0x10000000 --verify blink.bin
probe-rs info --probe 0000:0001:127.0.0.1:4445 --protocol jtag      # slot 4
```

`scripts/uf2_to_bin.py` turns an RP2040 UF2 file into such an image.

OpenOCD (master, for its CMSIS-DAP TCP backend), and GDB through it:

```sh
openocd -c "set SLOT 0" -f openocd/probehive-swd.cfg -f target/rp2040.cfg
openocd -c "set SLOT 4" -f openocd/probehive-jtag.cfg -f target/esp32.cfg
gdb -ex "target extended-remote :3333"
```

A target's console, and the GPIO:

```sh
nc 127.0.0.1 4461                                   # slot 0's UART, 115200 baud (--uart-baud)
python3 scripts/gpio.py build/slots_ci/ IO32=0      # drive; IO32=z releases; no pins: read all
```

The bridge's debug and UART ports for a slot share one CMSIS-DAP client,
so a debugger and a console can be connected at once.  Without the bridge,
`scripts/dap_udp.py` talks UDP to the probe directly; `scripts/dap_probe.py`
and `scripts/dap_bench.py` are built on it.

## Performance

With an RP2040 on slot 0, every word checked:

| client                                      | SWCLK  | read         | write        |
|---------------------------------------------|--------|--------------|--------------|
| `dap_bench.py`, 6 requests in flight        | 50 MHz | 3,800 KiB/s  | 3,140 KiB/s  |
| probe-rs `benchmark` through the bridge     | 25 MHz | 1.47 MB/s    | 1.45 MB/s    |

A single word read with a request of its own takes about 56 us, most of it
the network's round trip; commands the firmware answers take about as long.
With the other slots busy, slot 0 keeps 2.7 MB/s each way.  An ESP32 reads
about 110 KB/s at 20 to 33 MHz: probe-rs's Xtensa driver asks for a word at
a time, so each costs a round trip.

## Inside

- `gateware/arty_slots.py`: the SoC (LiteX): the CPU, LiteEth with
  Etherbone, the slots, the layout of pins, the build.
- `gateware/slots.py`: the slots, their buffers, the UDP transport and the
  dispatcher, which runs a request for the wire on the slot's engine
  without the CPU, and hands anything else to the firmware.
- `gateware/verilog/`: `swd_exec.v` and `swd_phy.v` (SWD), `jtag_engine.v`,
  `slot_ram.v`.
- `firmware/main.c`: the CMSIS-DAP commands the gateware leaves to the CPU
  (VexRiscv, RV32IM, at 50 MHz).
- `sim/`: `slot_sim.py` checks an SWD slot byte for byte against a port of
  DAP.c on a modelled target, over clocks and round trips, `--multidrop`
  with TARGETSEL sent as OpenOCD and probe-rs send it; `jtag_sim.py` does
  the same for a JTAG slot on a modelled TAP; `soc_sim.py` runs the whole
  SoC, firmware included, against those targets and UART loopbacks.
- `scripts/`: the host side, the self-check, and the build's helpers.

Not done: ARM DAP_Transfer over JTAG (ARM targets go on the SWD slots),
DAP_JTAG_IDCODE, SWO, parity and stop bits other than 8N1 on the UARTs.

## Vivado

The design outgrew the XC7A35T's nominal 20,800 LUTs, and openXC7 met its
100 MHz slot clock only on some placements, checked no hold times and ignored
false paths.  Vivado builds the same design in about 8 minutes where openXC7
took 35, meets that clock on its first placement and signs off setup and hold.

The XC7A35T is the XC7A50T's die.  Vivado's 35T part has every site of the
50T and refuses a design only on counts, before placement (`DRC UTLZ-1`);
the chip itself checks only the IDCODE a bitstream writes, and reports
`ID Error` for the 50T's.  So `arty_slots.py` has Vivado build for
`xc7a50ticsg324-1L`, keeps that bitstream as `arty_slots_50t.bit`, and
`scripts/bitpatch.py` writes the 35T's IDCODE (`0x0362D093`) into
`arty_slots.bit`, recomputing the CRC checks.  AMD supports none of this,
and specifies the 35T's power and heat only for the 35T's capacity.

Setting it up, once per host:

1. AMD's Linux web installer for 2026.1 needs an AMD account.  Unpack it and
   log in from a terminal of your own:

   ```sh
   bash FPGAs_AdaptiveSoCs_Unified_SDI_2026.1_*_Lin64.bin --noexec --target installer
   nix develop -c bash -c 'vivado-fhs -c "cd installer && ./xsetup -b AuthTokenGen"'
   ```

   then install Vivado with only the Artix-7 devices to
   `~/opt/Xilinx` (`xsetup -b ConfigGen` writes a configuration to edit;
   `xsetup --agree XilinxEULA,3rdPartyEULA --batch Install --config ...`
   runs it).  That is about 21 GB downloaded and 30 GB installed.
2. Vivado 2026.1 starts only with a license, even for Artix-7.  Generate a
   node-locked one on AMD's licensing portal for the host ID that
   `lmutil lmhostid` prints (an onboard NIC's, not a USB adapter's), and keep
   only its Basic tier, because with the whole file Vivado picks the Alveo
   tier, which offers no Artix-7 part:

   ```sh
   python3 scripts/basic_license.py ~/Downloads/Xilinx.lic ~/.Xilinx/Xilinx.lic
   ```

   The Basic tier is free and expires a year after it is issued.

`nix develop` puts `vivado` on the PATH: it runs `~/opt/Xilinx/2026.1/Vivado`
(or `$VIVADO_ROOT`) in an FHS environment the flake builds, with
`~/.Xilinx/Xilinx.lic` (or `$XILINXD_LICENSE_FILE`).

## Faults met, and what fixed them

- **openXC7's nextpnr routed nets through BUFGCTRLs it left unconfigured**
  (fixed in openXC7/nextpnr a17bd29c, after the revision the openXC7
  toolchain pins).  A data net took a global clock line through an empty
  BUFGCTRL, whose enable bits the FASM writer dropped, so its loads saw a
  constant: a block RAM byte lane that never wrote, register reads with
  fixed bits set, depending on placement.  The flake carries the fix, and
  `scripts/bufg_check.py` fails an openXC7 build that still has the fault.
- **Yosys 0.67 drops a shift register's enable when it maps it to SRLs**
  (YosysHQ/yosys@3006280d, fixed in 0.68).  LiteEth's ARP and IP header
  registers shifted on every clock, so the core parsed payload as header and
  answered nothing.  `flake.nix` applies the upstream commit for the
  openXC7 build; `sim/srl_enable_check.py` says whether the shell's Yosys
  still needs it.
- **Opening the board's UART restarted the design.**  With JP2 fitted, the
  FT2232's DTR drives the board's reset net (`CK_RST`, C2), and Linux raises
  DTR on every open.  The gateware does not use that reset.
- **LiteX picks no chip database for the industrial-grade part**
  (`xc7a35ticsg324-1L`); `arty_slots.py` names `xc7a35tcsg324` itself for
  openXC7.

## License

MIT or Apache-2.0, at your option; see LICENSE.
