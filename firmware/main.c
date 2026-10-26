/*
 * The slots' firmware: the CMSIS-DAP commands the hardware leaves to it.
 *
 * The dispatcher (gateware/slots.py) takes every datagram: it keeps each
 * slot's sequence, answers repeats, acts on DAP_TransferAbort, and runs a
 * request that starts with a command for the wire on the slot's engine
 * without this CPU.  Any other request becomes a job here: its request and
 * response entries and its length.  The firmware answers its commands, one at
 * a time, handing those for the wire (DAP_Transfer, DAP_TransferBlock,
 * DAP_WriteABORT, DAP_SWJ_Sequence, DAP_SWD_Sequence) to the slot's engine,
 * and gives back the reply's length; the dispatcher sends it.  The loop goes
 * on with the other slots while one waits for its engine, DAP_Delay or
 * DAP_SWJ_Pins.
 */

#include <stdint.h>

#include "hw.h"

#define ENTRIES    6
#define ENTRY      1280
#define HEADER     4
#define MAX_PACKET 1032
#define MAGIC      0xDA
#define VERSION    0x01

/* CMSIS-DAP commands and values */
enum {
    ID_INFO = 0x00, ID_HOST_STATUS = 0x01, ID_CONNECT = 0x02, ID_DISCONNECT = 0x03,
    ID_TRANSFER_CONFIGURE = 0x04, ID_TRANSFER = 0x05, ID_TRANSFER_BLOCK = 0x06,
    ID_TRANSFER_ABORT = 0x07, ID_WRITE_ABORT = 0x08, ID_DELAY = 0x09, ID_RESET_TARGET = 0x0A,
    ID_SWJ_PINS = 0x10, ID_SWJ_CLOCK = 0x11, ID_SWJ_SEQUENCE = 0x12, ID_SWD_CONFIGURE = 0x13,
    ID_JTAG_SEQUENCE = 0x14, ID_JTAG_CONFIGURE = 0x15, ID_JTAG_IDCODE = 0x16,
    ID_SWD_SEQUENCE = 0x1D, ID_UART_TRANSPORT = 0x1F, ID_UART_CONFIGURE = 0x20, ID_UART_TRANSFER = 0x21,
    ID_UART_CONTROL = 0x22, ID_UART_STATUS = 0x23, ID_QUEUE_COMMANDS = 0x7E, ID_EXECUTE_COMMANDS = 0x7F,
    ID_INVALID = 0xFF,
};
#define DAP_OK    0x00
#define DAP_ERROR 0xFF

/* A slot's pins register keeps DAP_SWJ_Pins' bits (gateware/slots.py), and an SWD slot whether
 * SWDIO is driven */
#define PIN_CLK   0x01                      /* SWCLK, TCK */
#define PIN_IO    0x02                      /* SWDIO, TMS */
#define PIN_TDI   0x04                      /* resting low: an ESP32's MTDI is a strapping pin */
#define PIN_TDO   0x08
#define PIN_NRST  0x80
#define PIN_OE    0x100

enum wait { W_NONE, W_ENGINE, W_DELAY, W_PINS };

static inline int is_jtag(int s) { return (SLOTS_JTAG >> s) & 1; }


struct slot {
    /* The job running: its request and response entries */
    int8_t   running;                       /* its request entry, -1: none */
    uint8_t  out;                           /* its response entry */
    uint16_t rp, end;                       /* next command byte, end of the packet */
    uint16_t wp;                            /* next response byte */
    uint8_t  left;                          /* commands still to run */
    uint8_t  wait;
    uint32_t until;                         /* W_DELAY, W_PINS: in ticks */
    uint8_t  pins_select, pins_value;       /* W_PINS */
    uint32_t done_toggle;                   /* the engine's, when it was started */

    uint8_t  connected;

    /* The UART, as ARM's reference (UART.c) keeps it */
    uint8_t  uart_dap;                      /* DAP_UART_Transport: the DAP commands */
    uint8_t  uart_configured;
    uint8_t  uart_rx, uart_tx;              /* receiving, sending */
};

static struct slot slots[SLOTS];

/* The console, on the FT2232's second channel */
static void putc_(char c)
{
    while (rd(CSR_UART_TXFULL_ADDR)) ;
    wr(CSR_UART_RXTX_ADDR, (uint8_t)c);
}

static void puts_(const char *s)
{
    while (*s) putc_(*s++);
}

static void puthex(uint32_t v, int digits)
{
    for (int i = digits - 1; i >= 0; i--) putc_("0123456789abcdef"[(v >> (4 * i)) & 15]);
}

/* Buffers ============================================================================================ */

static inline uint8_t rq(int s, int entry, unsigned at)
{
    return SLOT_REQ(s)[entry * ENTRY + at];
}

static inline void rs(int s, int entry, unsigned at, uint8_t value)
{
    SLOT_RESP(s)[entry * ENTRY + at] = value;
}

static uint32_t rq32(int s, int entry, unsigned at)
{
    return rq(s, entry, at) | (uint32_t)rq(s, entry, at + 1) << 8 |
           (uint32_t)rq(s, entry, at + 2) << 16 | (uint32_t)rq(s, entry, at + 3) << 24;
}

/* Commands answered here ============================================================================= */

static unsigned put_string(int s, int out, unsigned wp, const char *str)
{
    unsigned n = 0;
    while (str[n]) {
        rs(s, out, wp + 1 + n, str[n]);
        n++;
    }
    rs(s, out, wp + 1 + n, 0);
    rs(s, out, wp, n + 1);
    return n + 2;
}

static unsigned put_u32(int s, int out, unsigned wp, uint32_t v)
{
    rs(s, out, wp, 4);
    for (int i = 0; i < 4; i++) rs(s, out, wp + 1 + i, (uint8_t)(v >> (8 * i)));
    return 5;
}

static unsigned dap_info(int s, int out, unsigned wp, uint8_t id)
{
    static char serial[] = "probehive-slot-0";
    switch (id) {
    case 0x01: return put_string(s, out, wp, "probehive");
    case 0x02: return put_string(s, out, wp, "probehive CMSIS-DAP");
    case 0x03: serial[sizeof serial - 2] = '0' + s; return put_string(s, out, wp, serial);
    case 0x04: return put_string(s, out, wp, "2.1.1");
    case 0x09: return put_string(s, out, wp, "slots-1");
    case 0xF0: rs(s, out, wp, 1); rs(s, out, wp + 1, (is_jtag(s) ? 0x02 : 0x01) | 0x80); return 2;  /* UART */
    case 0xFB: return put_u32(s, out, wp, UART_RX_DEPTH);
    case 0xFC: return put_u32(s, out, wp, UART_TX_DEPTH);
    case 0xFE: rs(s, out, wp, 1); rs(s, out, wp + 1, ENTRIES); return 2;
    case 0xFF: rs(s, out, wp, 2); rs(s, out, wp + 1, MAX_PACKET & 0xFF);
               rs(s, out, wp + 2, MAX_PACKET >> 8); return 3;
    default:   rs(s, out, wp, 0); return 1;
    }
}

/* Without the M extension (VexRiscv minimal) division is done here: nixpkgs' libgcc for this
 * compiler is built for hard float, so the firmware carries what it needs itself */
static uint32_t udiv(uint32_t n, uint32_t d)
{
#ifdef __riscv_div
    return n / d;
#else
    uint32_t q = 0, r = 0;
    for (int i = 31; i >= 0; i--) {
        r = (r << 1) | ((n >> i) & 1);
        if (r >= d) { r -= d; q |= 1u << i; }
    }
    return q;
#endif
}

void *memcpy(void *dst, const void *src, unsigned n)
{
    uint8_t *d = dst;
    const uint8_t *s = src;
    while (n--) *d++ = *s++;
    return dst;
}

void *memset(void *dst, int c, unsigned n)
{
    uint8_t *d = dst;
    while (n--) *d++ = (uint8_t)c;
    return dst;
}

/* SWCLK in half cycles of the slots' clock: the fastest not above `hz`, at least four, at most
 * 2046; `set` toggles so that the engine calibrates before its next transfer */
static void set_clock(int s, uint32_t hz)
{
    uint32_t ticks = udiv(2u * SLOT_CLK_FREQ + hz - 1, hz);
    if (ticks < 4) ticks = 4;
    if (ticks > 2046) ticks = 2046;
    uint32_t hi = ticks / 2, lo = ticks - hi;
    uint32_t set = GET(rd(SLOT_REG(s, CLOCK)), SLOT0_CLOCK, SET) ^ 1;
    wr(SLOT_REG(s, CLOCK), PUT(SLOT0_CLOCK, HI, hi) | PUT(SLOT0_CLOCK, LO, lo) | PUT(SLOT0_CLOCK, SET, set));
}

/* The pins DAP_SWJ_Pins reads, at its bits */
static uint8_t pins_in(int s)
{
    return rd(SLOT_REG(s, PINS_IN)) & 0xFF;
}

static void set_pins(int s, uint32_t pins)
{
    wr(SLOT_REG(s, PINS), pins);
}

/* The UART ========================================================================================== */

/* The receiver's and sender's phase steps a sys cycle: baud * 2^32 / clock, here baud * K >> 8 */
#define UART_K    ((uint32_t)((((uint64_t)1 << 40) + CONFIG_CLOCK_FREQUENCY / 2) / CONFIG_CLOCK_FREQUENCY))
#define UART_MIN  300u
#define UART_MAX  (CONFIG_CLOCK_FREQUENCY / 16)

static void uart_ctl(int s, struct slot *sl, uint32_t pulses)
{
    wr(SLOT_REG(s, UART_CTL), PUT(SLOT0_UART_CTL, RX_EN, sl->uart_rx) | PUT(SLOT0_UART_CTL, TX_EN, sl->uart_tx) |
                              pulses);
}

/* DAP_UART_Status' bits; data lost is told once */
static uint8_t uart_status(int s, struct slot *sl)
{
    uint8_t status = (sl->uart_rx ? 0x01 : 0) | (sl->uart_tx ? 0x10 : 0);
    if (GET(rd(SLOT_REG(s, UART_STAT)), SLOT0_UART_STAT, LOST)) {
        status |= 0x02;
        uart_ctl(s, sl, PUT(SLOT0_UART_CTL, LOST_CLEAR, 1));
    }
    return status;
}

static void put32(int s, int out, unsigned at, uint32_t v)
{
    for (int i = 0; i < 4; i++) rs(s, out, at + i, (uint8_t)(v >> (8 * i)));
}

/* The commands a slot's engine runs (gateware/slots.py's SWD_COMMANDS and JTAG_COMMANDS) */
static int is_engine_command(int s, uint8_t id)
{
    if (is_jtag(s))
        return id == ID_SWJ_SEQUENCE || id == ID_JTAG_SEQUENCE;
    return id == ID_TRANSFER || id == ID_TRANSFER_BLOCK || id == ID_WRITE_ABORT ||
           id == ID_SWJ_SEQUENCE || id == ID_SWD_SEQUENCE;
}

/* One command not for the engine, at sl->rp: its response at sl->wp.  Returns 0 if the packet
 * cannot go on (a command not known, whose length is not known either). */
static int command(int s, struct slot *sl)
{
    int e = sl->running, out = sl->out;
    unsigned p = sl->rp, w = sl->wp;
    uint8_t id = rq(s, e, p);

    rs(s, out, w, id);
    switch (id) {
    case ID_INFO:
        w += 1 + dap_info(s, out, w + 1, rq(s, e, p + 1));
        p += 2;
        break;
    case ID_HOST_STATUS:
        rs(s, out, w + 1, DAP_OK); w += 2; p += 3;
        break;
    case ID_CONNECT: {
        /* The slot's own port: SWD (1) or JTAG (2), also for the default (0) */
        uint8_t port = rq(s, e, p + 1), own = is_jtag(s) ? 2 : 1;
        uint8_t mode = (port == 0 || port == own) ? own : 0;
        if (mode) {
            set_pins(s, PIN_CLK | PIN_IO | PIN_NRST | PIN_OE);
            sl->connected = 1;
        }
        rs(s, out, w + 1, mode); w += 2; p += 2;
        break;
    }
    case ID_DISCONNECT:
        set_pins(s, PIN_CLK | PIN_IO | PIN_NRST);                        /* SWDIO released */
        sl->connected = 0;
        rs(s, out, w + 1, DAP_OK); w += 2; p += 1;
        break;
    case ID_TRANSFER_CONFIGURE: {
        uint32_t idle = rq(s, e, p + 1);
        uint32_t wait = rq(s, e, p + 2) | rq(s, e, p + 3) << 8;
        uint32_t match = rq(s, e, p + 4) | rq(s, e, p + 5) << 8;
        if (!is_jtag(s)) {                                      /* no transfers on a JTAG slot */
            wr(SLOT_REG(s, RETRY), PUT(SLOT0_RETRY, WAIT, wait) | PUT(SLOT0_RETRY, MATCH, match));
            uint32_t c = rd(SLOT_REG(s, CONFIG)) & ~PUT(SLOT0_CONFIG, IDLE, 0xFF);
            wr(SLOT_REG(s, CONFIG), c | PUT(SLOT0_CONFIG, IDLE, idle));
        }
        rs(s, out, w + 1, DAP_OK); w += 2; p += 6;
        break;
    }
    case ID_TRANSFER_ABORT:                                              /* nothing runs now */
        p += 1;
        break;
    case ID_DELAY: {
        uint32_t us = rq(s, e, p + 1) | rq(s, e, p + 2) << 8;
        sl->until = ticks() + us * TICKS_PER_US;
        sl->wait = W_DELAY;
        rs(s, out, w + 1, DAP_OK); w += 2; p += 3;
        break;
    }
    case ID_RESET_TARGET:
        rs(s, out, w + 1, DAP_OK); rs(s, out, w + 2, 0); w += 3; p += 1;
        break;
    case ID_SWJ_PINS: {
        uint8_t value = rq(s, e, p + 1), select = rq(s, e, p + 2);
        uint32_t us = rq32(s, e, p + 3);
        /* The outputs it may set, and the pins it may wait for */
        uint32_t outs = PIN_CLK | PIN_IO | PIN_NRST | (is_jtag(s) ? PIN_TDI : 0);
        uint32_t pins = rd(SLOT_REG(s, PINS));
        pins = (pins & ~(select & outs)) | (value & select & outs);
        if (!is_jtag(s) && (select & PIN_IO)) pins |= PIN_OE;
        /* An ESP32 reads MTDI (TDI) for its flash supply as it leaves reset: in reset, TDI is low */
        if (is_jtag(s) && !(pins & PIN_NRST)) pins &= ~PIN_TDI;
        set_pins(s, pins);
        if (us > 3000000) us = 3000000;
        sl->until = ticks() + us * TICKS_PER_US;
        sl->pins_select = select & (outs | (is_jtag(s) ? PIN_TDO : 0));
        sl->pins_value = value;
        sl->wait = W_PINS;                    /* the response's byte when they match or time is up */
        w += 2; p += 7;
        break;
    }
    case ID_SWJ_CLOCK: {
        uint32_t hz = rq32(s, e, p + 1);
        if (hz) set_clock(s, hz);
        rs(s, out, w + 1, hz ? DAP_OK : DAP_ERROR); w += 2; p += 5;
        break;
    }
    case ID_SWD_CONFIGURE: {
        uint8_t cfg = rq(s, e, p + 1);
        if (!is_jtag(s)) {
            uint32_t c = rd(SLOT_REG(s, CONFIG)) & ~(PUT(SLOT0_CONFIG, TURN, 3) | PUT(SLOT0_CONFIG, DATAPHASE, 1));
            wr(SLOT_REG(s, CONFIG), c | PUT(SLOT0_CONFIG, TURN, cfg & 3) |
                                    PUT(SLOT0_CONFIG, DATAPHASE, (cfg >> 2) & 1));
        }
        rs(s, out, w + 1, DAP_OK); w += 2; p += 2;
        break;
    }
    case ID_JTAG_CONFIGURE:                    /* the chain's IR lengths: the host scans by itself */
        if (!is_jtag(s)) goto unknown;
        rs(s, out, w + 1, DAP_OK); w += 2; p += 2 + rq(s, e, p + 1);
        break;
    case ID_UART_TRANSPORT: {                    /* none, or the DAP commands; no USB COM port */
        uint8_t t = rq(s, e, p + 1), ret = DAP_OK;
        if (t == 0) {
            sl->uart_dap = 0; sl->uart_configured = 0; sl->uart_rx = 0; sl->uart_tx = 0;
            uart_ctl(s, sl, PUT(SLOT0_UART_CTL, RX_FLUSH, 1) | PUT(SLOT0_UART_CTL, TX_FLUSH, 1));
        } else if (t == 2)
            sl->uart_dap = 1;
        else
            ret = DAP_ERROR;
        rs(s, out, w + 1, ret); w += 2; p += 2;
        break;
    }
    case ID_UART_CONFIGURE: {                   /* 8 data bits, no parity, 1 stop bit */
        uint8_t control = rq(s, e, p + 1), status = 0;
        uint32_t baud = rq32(s, e, p + 2);
        if (!sl->uart_dap || control != 0) {
            status = (control & 0x0F) ? 0x01 : 0;
            status |= (control & 0x30) ? 0x02 : 0;
            status |= (control & 0xC0) ? 0x04 : 0;
            if (!sl->uart_dap) status = 0x07;
            baud = 0;
        } else if (baud < UART_MIN || baud > UART_MAX)
            baud = 0;                                       /* the baud rate's error, as UART.c */
        if (status == 0 && baud != 0) {
            uint32_t tuning = (uint32_t)(((uint64_t)baud * UART_K) >> 8);
            wr(SLOT_REG(s, UART_TUNING), tuning);
            baud = (uint32_t)(((uint64_t)tuning * CONFIG_CLOCK_FREQUENCY + ((uint64_t)1 << 31)) >> 32);
            sl->uart_configured = 1;
        } else
            sl->uart_configured = 0;
        rs(s, out, w + 1, status); put32(s, out, w + 2, baud); w += 6; p += 6;
        break;
    }
    case ID_UART_CONTROL: {
        uint8_t control = rq(s, e, p + 1), ret = DAP_OK;
        uint32_t pulses = 0;
        if (!sl->uart_dap)
            ret = DAP_ERROR;
        else {
            if (control & 0x02) sl->uart_rx = 0;
            else if (control & 0x01) {
                if (!sl->uart_configured) ret = DAP_ERROR;
                else if (!sl->uart_rx) { sl->uart_rx = 1; pulses |= PUT(SLOT0_UART_CTL, RX_FLUSH, 1); }
            }
            if (control & 0x04) pulses |= PUT(SLOT0_UART_CTL, RX_FLUSH, 1);
            if (control & 0x20) sl->uart_tx = 0;
            else if (control & 0x10) {
                if (!sl->uart_configured) ret = DAP_ERROR;
                else if (!sl->uart_tx) { sl->uart_tx = 1; pulses |= PUT(SLOT0_UART_CTL, TX_FLUSH, 1); }
            }
            if (control & 0x40) pulses |= PUT(SLOT0_UART_CTL, TX_FLUSH, 1);
            uart_ctl(s, sl, pulses);
        }
        rs(s, out, w + 1, ret); w += 2; p += 2;
        break;
    }
    case ID_UART_STATUS: {
        uint32_t st = rd(SLOT_REG(s, UART_STAT));
        int on = sl->uart_dap && sl->uart_configured;
        rs(s, out, w + 1, on ? uart_status(s, sl) : 0);
        put32(s, out, w + 2, on ? GET(st, SLOT0_UART_STAT, RX_LEVEL) : 0);
        put32(s, out, w + 6, on ? GET(st, SLOT0_UART_STAT, TX_LEVEL) : 0);
        w += 10; p += 1;
        break;
    }
    case ID_UART_TRANSFER: {
        /* Bytes received, as many as asked for, are in and fit; then bytes to send, as many as fit */
        uint32_t rx_n = rq(s, e, p + 1) | rq(s, e, p + 2) << 8;
        uint32_t tx_ask = rq(s, e, p + 3) | rq(s, e, p + 4) << 8, tx_n = 0;
        uint8_t status = 0;
        if (sl->uart_dap) {
            uint32_t st = rd(SLOT_REG(s, UART_STAT));
            uint32_t room = MAX_PACKET + HEADER - (w + 6);
            if (rx_n > GET(st, SLOT0_UART_STAT, RX_LEVEL)) rx_n = GET(st, SLOT0_UART_STAT, RX_LEVEL);
            if (rx_n > room) rx_n = room;
            for (uint32_t i = 0; i < rx_n; i++) {
                rs(s, out, w + 6 + i, GET(rd(SLOT_REG(s, UART_RX)), SLOT0_UART_RX, DATA));
                wr(SLOT_REG(s, UART_POP), 1);
            }
            tx_n = UART_TX_DEPTH - GET(st, SLOT0_UART_STAT, TX_LEVEL);
            if (tx_n > tx_ask) tx_n = tx_ask;
            for (uint32_t i = 0; i < tx_n; i++) wr(SLOT_REG(s, UART_TX), rq(s, e, p + 5 + i));
            status = uart_status(s, sl);
        } else
            rx_n = 0;
        rs(s, out, w + 1, status);
        rs(s, out, w + 2, (uint8_t)tx_n); rs(s, out, w + 3, (uint8_t)(tx_n >> 8));
        rs(s, out, w + 4, (uint8_t)rx_n); rs(s, out, w + 5, (uint8_t)(rx_n >> 8));
        w += 6 + rx_n; p += 5 + tx_ask;                    /* past every byte offered */
        break;
    }
    case ID_JTAG_IDCODE:                       /* not done here yet: DAP_JTAG_Sequence reads it */
        if (!is_jtag(s)) goto unknown;
        rs(s, out, w + 1, DAP_ERROR);
        for (int i = 0; i < 4; i++) rs(s, out, w + 2 + i, 0);
        w += 6; p += 2;
        break;
    default:
    unknown:
        rs(s, out, w, ID_INVALID); w += 1;
        sl->rp = p; sl->wp = w;
        return 0;
    }
    sl->rp = p;
    sl->wp = w;
    return 1;
}

/* Running a slot's requests ========================================================================== */

static void start_engine(int s, struct slot *sl)
{
    sl->done_toggle = GET(rd(SLOT_REG(s, DONE)), SLOT0_DONE, TOGGLE);
    wr(SLOT_REG(s, CMD), PUT(SLOT0_CMD, RQ_BASE, sl->running * ENTRY + sl->rp) |
                         PUT(SLOT0_CMD, RS_BASE, sl->out * ENTRY + sl->wp));
    sl->wait = W_ENGINE;
}

/* A job: its entries and length from the dispatcher */
static void begin(int s, struct slot *sl)
{
#if SLOTS > 1
    uint32_t job = rd(CSR_DAP_JOB0_ADDR + (uint32_t)s * (CSR_DAP_JOB1_ADDR - CSR_DAP_JOB0_ADDR));
#else
    uint32_t job = rd(CSR_DAP_JOB0_ADDR);
#endif
    sl->running = GET(job, DAP_JOB0, ENTRY);
    sl->out = GET(job, DAP_JOB0, OUT);
    sl->end = GET(job, DAP_JOB0, LENGTH);
    sl->rp = 0;
    sl->wp = HEADER;
    sl->wait = W_NONE;
    if (rq(s, sl->running, 0) == ID_EXECUTE_COMMANDS && sl->end >= 2) {
        sl->left = rq(s, sl->running, 1);
        rs(s, sl->out, HEADER, ID_EXECUTE_COMMANDS);
        rs(s, sl->out, HEADER + 1, sl->left);
        sl->rp = 2;
        sl->wp = HEADER + 2;
    } else
        sl->left = 1;
}

/* The job is done: the dispatcher writes the header and sends the reply */
static void finish(int s, struct slot *sl)
{
    wr(CSR_DAP_JOB_DONE_ADDR, PUT(DAP_JOB_DONE, SLOT, s) | PUT(DAP_JOB_DONE, LENGTH, sl->wp));
    sl->running = -1;
}

static void step(int s)
{
    struct slot *sl = &slots[s];

    switch (sl->wait) {
    case W_ENGINE: {
        uint32_t done = rd(SLOT_REG(s, DONE));
        if (GET(done, SLOT0_DONE, TOGGLE) == sl->done_toggle) return;
        sl->rp += GET(done, SLOT0_DONE, RQ_LEN);
        sl->wp += GET(done, SLOT0_DONE, RS_LEN);
        sl->wait = W_NONE;
        sl->left--;
        break;
    }
    case W_DELAY:
        if ((int32_t)(ticks() - sl->until) < 0) return;
        sl->wait = W_NONE;
        sl->left--;
        break;
    case W_PINS: {
        uint8_t now = pins_in(s);
        if (((now ^ sl->pins_value) & sl->pins_select) && (int32_t)(ticks() - sl->until) < 0) return;
        rs(s, sl->out, sl->wp - 1, now);
        sl->wait = W_NONE;
        sl->left--;
        break;
    }
    default:
        break;
    }

    while (sl->wait == W_NONE) {
        if (sl->left == 0 || sl->rp >= sl->end) {
            finish(s, sl);
            return;
        }
        uint8_t id = rq(s, sl->running, sl->rp);
        if (is_engine_command(s, id)) {
            start_engine(s, sl);
            return;
        }
        if (!command(s, sl)) {
            sl->left = 0;
            continue;
        }
        if (sl->wait == W_NONE) sl->left--;
    }
}

int main(void)
{
    puts_("probehive slots: ");
    puthex(SLOTS, 1);
    puts_("\n");
    for (int s = 0; s < SLOTS; s++) {
        slots[s].running = -1;
        set_clock(s, 1000000);
        set_pins(s, PIN_CLK | PIN_IO | PIN_NRST | PIN_OE);
    }
    uint32_t seen = 0;                          /* the dispatcher's job toggles, as last taken */
    for (;;) {
        uint32_t jobs = rd(CSR_DAP_JOBS_ADDR);
        for (int s = 0; s < SLOTS; s++) {
            struct slot *sl = &slots[s];
            if (sl->running < 0) {
                if (!(((jobs ^ seen) >> s) & 1)) continue;
                seen ^= 1u << s;
                begin(s, sl);
            }
            step(s);
        }
    }
}
