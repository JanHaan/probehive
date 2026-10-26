# The slots' firmware: bare metal, in the SoC's main RAM, for the CPU variant's ISA.
#   make GENERATED=<dir with csr.h, soc.h, mem.h> BUILD=<output dir> MARCH=<LiteX's -march>

CROSS   ?= riscv32-none-elf-
CC      := $(CROSS)gcc
OBJCOPY := $(CROSS)objcopy
SIZE    := $(CROSS)size

GENERATED ?= ../build/slots_rows/generated
BUILD     ?= ../build/slots_rows/firmware
MARCH     ?= rv32i

CFLAGS  := -march=$(MARCH) -mabi=ilp32 -O2 -std=c11 -ffreestanding -fno-builtin -nostdlib \
           -ffunction-sections -fdata-sections -Wall -Wextra -Werror -I$(GENERATED)
LDFLAGS := -T linker.ld -nostdlib -Wl,--gc-sections -Wl,--no-warn-rwx-segments -Wl,-Map,$(BUILD)/firmware.map

SOURCES := crt0.S main.c

$(BUILD)/firmware.bin: $(BUILD)/firmware.elf
	$(OBJCOPY) -O binary $< $@
	$(SIZE) $<

$(BUILD)/firmware.elf: $(SOURCES) linker.ld hw.h $(GENERATED)/csr.h | $(BUILD)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $(SOURCES)

$(BUILD):
	mkdir -p $@

.PHONY: clean
clean:
	rm -rf $(BUILD)
