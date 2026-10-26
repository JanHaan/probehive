/* Registers and buffers of the slots' SoC (gateware/arty_slots.py). */
#ifndef HW_H
#define HW_H

#include <stdint.h>

#include "csr.h"
#include "soc.h"

static inline uint32_t rd(uint32_t addr) { return *(volatile uint32_t *)addr; }
static inline void wr(uint32_t addr, uint32_t value) { *(volatile uint32_t *)addr = value; }

/* A field of a register's value, and a value placed in a field */
#define GET(value, reg, field) \
    (((value) >> CSR_##reg##_##field##_OFFSET) & ((1u << CSR_##reg##_##field##_SIZE) - 1))
#define PUT(reg, field, value) \
    (((uint32_t)(value) & ((1u << CSR_##reg##_##field##_SIZE) - 1)) << CSR_##reg##_##field##_OFFSET)

/* Every slot's registers are slot 0's, a stride apart */
#if SLOTS > 1
#define SLOT_STRIDE (CSR_SLOT1_BASE - CSR_SLOT0_BASE)
#else
#define SLOT_STRIDE 0x800
#endif
#define SLOT_REG(slot, reg) (CSR_SLOT0_##reg##_ADDR + (uint32_t)(slot) * SLOT_STRIDE)

/* A slot's buffers: requests, then responses */
#define SLOT_REQ(slot) ((volatile uint8_t *)(SLOTS_BASE + (uint32_t)(slot) * SLOT_BYTES))
#define SLOT_RESP(slot) ((volatile uint8_t *)(SLOTS_BASE + (uint32_t)(slot) * SLOT_BYTES + SLOT_BYTES / 2))

static inline uint32_t ticks(void) { return rd(CSR_MAIN_TICKS_ADDR); }
#define TICKS_PER_US (CONFIG_CLOCK_FREQUENCY / 1000000)

#endif
