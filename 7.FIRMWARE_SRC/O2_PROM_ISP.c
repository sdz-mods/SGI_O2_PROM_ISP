/*
 * O2 PROM ISP - Firmware for RP2354B
 *
 * Programs/reads AT29C040A parallel flash via USB CDC.
 * Protocol: text commands with binary data payloads.
 *
 * Commands (host -> MCU, newline terminated):
 *   VER               -> "OK A00\n"
 *   BOOT              -> reboot to BOOTSEL (UF2 mode)
 *   FLASH <size>\n    -> "OK\n", then receive <size> bytes in CHUNK_SIZE
 *                        chunks, each ACK'd with "ACK\n"; finish: "DONE\n"
 *   VERIFY <size>\n   -> same as FLASH but reads back and compares instead
 *                        of writing; mismatch reports "ERR <offset>\n"
 *   DUMP\n            -> "SIZE 524288\n", then sends binary in CHUNK_SIZE
 *                        chunks waiting for "ACK\n" after each; finish: "DONE\n"
 *
 * Flash write note:
 *   Each page write is preceded by a 3-byte fast SDP unlock sequence
 *   (AA→5555, 55→2AAA, A0→5555) required by this chip variant.
 *   All flash bus GPIOs are high-Z when idle; they are driven only during
 *   an active flash_bus_acquire / flash_bus_release window.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/gpio.h"

/* -------------------------------------------------------------------------
 * Constants
 * ---------------------------------------------------------------------- */

#define FIRMWARE_VERSION    "A00"
#define FLASH_SIZE          (512u * 1024u)  /* AT29C040A: 512 KB */
#define PAGE_SIZE           256u            /* AT29C040A page size */
#define CHUNK_SIZE          4096u           /* 16 pages per USB chunk */
#define CMD_BUF_SIZE        64u

/* -------------------------------------------------------------------------
 * Pin definitions
 * ---------------------------------------------------------------------- */

#define DC_BUS_PIN  21u   /* High while MCU owns the flash bus */
#define WE_N_PIN    10u
#define OE_N_PIN    25u
#define CE_N_PIN    26u
#define LED0_PIN    46u   /* On when firmware is running */
#define LED1_PIN    47u   /* 2 Hz heartbeat */

/*
 * Address bus: A0..A18 -> GPIO
 *   A0 =36, A1=35, A2=34, A3=33, A4=20, A5=19, A6=18, A7=17,
 *   A8 = 6, A9= 5, A10=27, A11=4, A12=16, A13=7, A14=8, A15=15,
 *   A16=14, A17=9, A18=11
 */
static const uint8_t ADDR_PINS[19] = {
    36, 35, 34, 33, 20, 19, 18, 17,   /* A0-A7  */
     6,  5, 27,  4, 16,  7,  8, 15,   /* A8-A15 */
    14,  9, 11                         /* A16-A18 */
};

/*
 * Data bus: D0..D7 -> GPIO
 *   D0=37, D1=38, D2=39, D3=28, D4=30, D5=29, D6=32, D7=31
 */
static const uint8_t DATA_PINS[8] = {
    37, 38, 39, 28, 30, 29, 32, 31
};

/* -------------------------------------------------------------------------
 * Heartbeat state
 * ---------------------------------------------------------------------- */

static absolute_time_t led1_next;
static bool led1_state = false;

static void heartbeat_update(void) {
    if (time_reached(led1_next)) {
        led1_state = !led1_state;
        gpio_put(LED1_PIN, led1_state);
        led1_next = make_timeout_time_ms(500); /* 2 toggles/s = 1 Hz */
    }
}

/* -------------------------------------------------------------------------
 * Flash bus helpers
 * ---------------------------------------------------------------------- */

static void flash_set_address(uint32_t addr) {
    for (int i = 0; i < 19; i++) {
        gpio_put(ADDR_PINS[i], (addr >> i) & 1u);
    }
}

static void flash_data_dir_out(void) {
    for (int i = 0; i < 8; i++) {
        gpio_set_dir(DATA_PINS[i], GPIO_OUT);
    }
}

static void flash_data_dir_in(void) {
    for (int i = 0; i < 8; i++) {
        gpio_set_dir(DATA_PINS[i], GPIO_IN);
        gpio_disable_pulls(DATA_PINS[i]);
    }
}

static void flash_set_data_out(uint8_t data) {
    for (int i = 0; i < 8; i++) {
        gpio_put(DATA_PINS[i], (data >> i) & 1u);
    }
}

static uint8_t flash_get_data_in(void) {
    uint8_t data = 0;
    for (int i = 0; i < 8; i++) {
        if (gpio_get(DATA_PINS[i])) {
            data |= (uint8_t)(1u << i);
        }
    }
    return data;
}

/* Acquire flash bus: drive all bus pins as outputs, assert DC_BUS. */
static void flash_bus_acquire(void) {
    /* Address pins: output, start at 0 */
    for (int i = 0; i < 19; i++) {
        gpio_set_dir(ADDR_PINS[i], GPIO_OUT);
        gpio_put(ADDR_PINS[i], 0);
    }
    /* Control lines: output, deasserted (active-low → idle = 1) */
    gpio_set_dir(WE_N_PIN, GPIO_OUT); gpio_put(WE_N_PIN, 1);
    gpio_set_dir(OE_N_PIN, GPIO_OUT); gpio_put(OE_N_PIN, 1);
    gpio_set_dir(CE_N_PIN, GPIO_OUT); gpio_put(CE_N_PIN, 1);
    /* Data bus stays input until a write needs it (flash_data_dir_out) */
    gpio_put(DC_BUS_PIN, 1);
    sleep_us(10);
}

/* Release flash bus: high-Z all flash bus GPIOs, then drop DC_BUS. */
static void flash_bus_release(void) {
    /* Deassert controls before going high-Z */
    gpio_put(WE_N_PIN, 1);
    gpio_put(OE_N_PIN, 1);
    gpio_put(CE_N_PIN, 1);
    /* High-Z address bus */
    for (int i = 0; i < 19; i++) {
        gpio_set_dir(ADDR_PINS[i], GPIO_IN);
        gpio_disable_pulls(ADDR_PINS[i]);
    }
    /* High-Z data bus */
    for (int i = 0; i < 8; i++) {
        gpio_set_dir(DATA_PINS[i], GPIO_IN);
        gpio_disable_pulls(DATA_PINS[i]);
    }
    /* High-Z control lines (external pull-ups keep them inactive) */
    gpio_set_dir(WE_N_PIN, GPIO_IN); gpio_disable_pulls(WE_N_PIN);
    gpio_set_dir(OE_N_PIN, GPIO_IN); gpio_disable_pulls(OE_N_PIN);
    gpio_set_dir(CE_N_PIN, GPIO_IN); gpio_disable_pulls(CE_N_PIN);
    /* Drop DC_BUS — system bus may now reconnect to the flash */
    gpio_put(DC_BUS_PIN, 0);
}

/* -------------------------------------------------------------------------
 * Flash operations
 * ---------------------------------------------------------------------- */

/*
 * Read one byte at a given address.
 * Data pins must already be set to input (flash_data_dir_in).
 */
static uint8_t flash_read_byte(uint32_t addr) {
    flash_set_address(addr);
    gpio_put(CE_N_PIN, 0);
    gpio_put(OE_N_PIN, 0);
    sleep_us(1);            /* tRC >= 70 ns; 1 µs gives ample margin */
    uint8_t data = flash_get_data_in();
    gpio_put(OE_N_PIN, 1);
    gpio_put(CE_N_PIN, 1);
    sleep_us(1);
    return data;
}

/*
 * Write one byte as a single-byte page write.
 * Used for SDP command sequences.
 * Data pins must already be set to output (flash_data_dir_out).
 */
static void flash_write_single_byte(uint32_t addr, uint8_t data) {
    flash_set_address(addr);
    flash_set_data_out(data);
    gpio_put(CE_N_PIN, 0);
    sleep_us(1);
    gpio_put(WE_N_PIN, 0);
    sleep_us(1);    /* tWP >= 100 ns */
    gpio_put(WE_N_PIN, 1);
    sleep_us(1);
    gpio_put(CE_N_PIN, 1);
    sleep_ms(12);   /* tWC max = 10 ms; 12 ms for margin */
}

/*
 * Per-page SDP unlock sequence — must be called immediately before
 * each flash_write_page() call.
 *
 * Three fast CE# cycles (only µs gaps between bytes — no tWC).
 * This is the "Byte/Page Program" software command the chip recognises
 * as a per-write SDP bypass.
 *
 * Data pins must already be set to output.
 */
static void flash_unlock_for_write(void) {
    /* AA → 5555h */
    flash_set_address(0x5555u); flash_set_data_out(0xAAu);
    gpio_put(CE_N_PIN, 0); sleep_us(1);
    gpio_put(WE_N_PIN, 0); sleep_us(1); gpio_put(WE_N_PIN, 1); sleep_us(1);
    gpio_put(CE_N_PIN, 1); sleep_us(2);
    /* 55h → 2AAAh */
    flash_set_address(0x2AAAu); flash_set_data_out(0x55u);
    gpio_put(CE_N_PIN, 0); sleep_us(1);
    gpio_put(WE_N_PIN, 0); sleep_us(1); gpio_put(WE_N_PIN, 1); sleep_us(1);
    gpio_put(CE_N_PIN, 1); sleep_us(2);
    /* A0h → 5555h (byte/page program command) */
    flash_set_address(0x5555u); flash_set_data_out(0xA0u);
    gpio_put(CE_N_PIN, 0); sleep_us(1);
    gpio_put(WE_N_PIN, 0); sleep_us(1); gpio_put(WE_N_PIN, 1); sleep_us(1);
    gpio_put(CE_N_PIN, 1); sleep_us(2);
}

/*
 * Write one 256-byte page to the AT29C040A.
 *
 * page_num selects which 256-byte page (0 .. 2047).
 *
 * Sends the 3-byte fast SDP unlock (flash_unlock_for_write) first, then
 * holds CE_N low while loading all 256 bytes with WE_N pulses.
 * CE_N rising edge starts the internal erase+program cycle.
 *
 * Data pins must already be set to output (flash_data_dir_out).
 */
static void flash_write_page(uint32_t page_num, const uint8_t *data) {
    uint32_t base = page_num * PAGE_SIZE;

    /* Per-page SDP unlock: AA→5555, 55→2AAA, A0→5555 (fast, µs gaps) */
    flash_unlock_for_write();

    /* Page load: CE_N held low through all 256 WE_N pulses */
    gpio_put(CE_N_PIN, 0);

    for (uint32_t i = 0; i < PAGE_SIZE; i++) {
        flash_set_address(base | i);
        flash_set_data_out(data[i]);
        sleep_us(1);            /* tAS + tDS setup */
        gpio_put(WE_N_PIN, 0);
        sleep_us(1);            /* tWP >= 100 ns */
        gpio_put(WE_N_PIN, 1);
        sleep_us(1);            /* tWPH */
    }

    gpio_put(CE_N_PIN, 1);      /* rising edge triggers internal write */
    sleep_ms(15);               /* tWC max = 10 ms + 5 ms margin */
}

/* -------------------------------------------------------------------------
 * GPIO initialisation
 * ---------------------------------------------------------------------- */

static void flash_gpio_init(void) {
    /* All flash bus pins start high-Z (input, no pulls).
     * They are driven only while the bus is acquired (flash_bus_acquire).
     * External pull-ups on WE_N/OE_N/CE_N keep those lines safely inactive. */
    for (int i = 0; i < 19; i++) {
        gpio_init(ADDR_PINS[i]);
        gpio_set_dir(ADDR_PINS[i], GPIO_IN);
        gpio_disable_pulls(ADDR_PINS[i]);
    }
    for (int i = 0; i < 8; i++) {
        gpio_init(DATA_PINS[i]);
        gpio_set_dir(DATA_PINS[i], GPIO_IN);
        gpio_disable_pulls(DATA_PINS[i]);
    }
    gpio_init(WE_N_PIN); gpio_set_dir(WE_N_PIN, GPIO_IN); gpio_disable_pulls(WE_N_PIN);
    gpio_init(OE_N_PIN); gpio_set_dir(OE_N_PIN, GPIO_IN); gpio_disable_pulls(OE_N_PIN);
    gpio_init(CE_N_PIN); gpio_set_dir(CE_N_PIN, GPIO_IN); gpio_disable_pulls(CE_N_PIN);

    /* DC_BUS: output, low when idle */
    gpio_init(DC_BUS_PIN); gpio_set_dir(DC_BUS_PIN, GPIO_OUT); gpio_put(DC_BUS_PIN, 0);
}

/* -------------------------------------------------------------------------
 * Protocol helpers
 * ---------------------------------------------------------------------- */

/* Read one byte from USB CDC; timeout in microseconds. Returns -1 on timeout. */
static int recv_byte_timeout(uint32_t timeout_us) {
    return getchar_timeout_us(timeout_us);
}

/*
 * Receive exactly 'len' bytes into 'buf'.
 * Allows up to 5 s per byte (USB CDC latency is well under 1 ms in practice).
 * Returns false on timeout.
 */
static bool recv_bytes(uint8_t *buf, size_t len) {
    for (size_t i = 0; i < len; i++) {
        heartbeat_update();
        int c = recv_byte_timeout(5000000u);
        if (c < 0) {
            return false;
        }
        buf[i] = (uint8_t)c;
    }
    return true;
}

/* Send 'len' raw bytes (binary-safe). */
static void send_bytes(const uint8_t *buf, size_t len) {
    for (size_t i = 0; i < len; i++) {
        putchar_raw((int)buf[i]);
    }
    stdio_flush();
}

/* Send a text response line. */
static void send_line(const char *msg) {
    printf("%s\n", msg);
    stdio_flush();
}

/* Read a text line from stdin (strips \r\n).  Non-blocking: returns false
 * if no complete line is available yet (call again from main loop). */
static bool readline_nonblocking(char *buf, size_t bufsize, size_t *len) {
    int c = recv_byte_timeout(0); /* non-blocking */
    if (c < 0) {
        return false; /* no data */
    }
    if (c == '\r') {
        return false; /* ignore CR */
    }
    if (c == '\n') {
        buf[*len] = '\0';
        return *len > 0; /* complete line */
    }
    if (*len < bufsize - 1) {
        buf[(*len)++] = (char)c;
    }
    return false;
}

/* -------------------------------------------------------------------------
 * Command handlers
 * ---------------------------------------------------------------------- */

static uint8_t chunk_buf[CHUNK_SIZE];

/* FLASH <size> : program flash with binary data from host */
static void cmd_flash(uint32_t total_size) {
    if (total_size == 0 || total_size > FLASH_SIZE) {
        send_line("ERR Invalid size");
        return;
    }

    send_line("OK");

    flash_bus_acquire();
    flash_data_dir_out();
    /* No global SDP disable — flash_write_page sends per-page 3-byte unlock */

    uint32_t offset = 0;
    bool ok = true;

    while (offset < total_size) {
        uint32_t chunk = total_size - offset;
        if (chunk > CHUNK_SIZE) chunk = CHUNK_SIZE;

        /* Pad last chunk to full CHUNK_SIZE so we always receive complete chunks */
        uint32_t recv_size = chunk;
        if (recv_size < CHUNK_SIZE && (total_size % CHUNK_SIZE) != 0) {
            /* host pads with 0xFF */
            recv_size = CHUNK_SIZE;
        }

        if (!recv_bytes(chunk_buf, recv_size)) {
            send_line("ERR Timeout");
            ok = false;
            break;
        }

        /* Write pages within this chunk */
        uint32_t pages = recv_size / PAGE_SIZE;
        for (uint32_t p = 0; p < pages; p++) {
            uint32_t page_num = (offset / PAGE_SIZE) + p;
            flash_write_page(page_num, &chunk_buf[p * PAGE_SIZE]);
        }

        offset += chunk;
        send_line("ACK");
    }

    flash_bus_release();

    if (ok) {
        send_line("DONE");
    }
}

/* VERIFY <size> : compare host data with flash contents */
static void cmd_verify(uint32_t total_size) {
    if (total_size == 0 || total_size > FLASH_SIZE) {
        send_line("ERR Invalid size");
        return;
    }

    send_line("OK");

    flash_bus_acquire();
    flash_data_dir_in();

    uint32_t offset = 0;
    bool ok = true;

    while (offset < total_size && ok) {
        uint32_t chunk = total_size - offset;
        if (chunk > CHUNK_SIZE) chunk = CHUNK_SIZE;

        uint32_t recv_size = chunk;
        if (recv_size < CHUNK_SIZE && (total_size % CHUNK_SIZE) != 0) {
            recv_size = CHUNK_SIZE;
        }

        if (!recv_bytes(chunk_buf, recv_size)) {
            send_line("ERR Timeout");
            ok = false;
            break;
        }

        /* Compare byte by byte */
        for (uint32_t i = 0; i < chunk; i++) {
            uint8_t got = flash_read_byte(offset + i);
            if (got != chunk_buf[i]) {
                char msg[48];
                snprintf(msg, sizeof(msg), "ERR %lu", (unsigned long)(offset + i));
                send_line(msg);
                ok = false;
                break;
            }
        }

        if (ok) {
            offset += chunk;
            send_line("ACK");
        }
    }

    flash_bus_release();

    if (ok) {
        send_line("DONE");
    }
}

/* DUMP : send entire flash contents to host */
static void cmd_dump(void) {
    char size_msg[32];
    snprintf(size_msg, sizeof(size_msg), "SIZE %u", (unsigned)FLASH_SIZE);
    send_line(size_msg);

    flash_bus_acquire();
    flash_data_dir_in();

    uint32_t offset = 0;

    while (offset < FLASH_SIZE) {
        uint32_t chunk = FLASH_SIZE - offset;
        if (chunk > CHUNK_SIZE) chunk = CHUNK_SIZE;

        /* Read chunk from flash */
        for (uint32_t i = 0; i < chunk; i++) {
            chunk_buf[i] = flash_read_byte(offset + i);
            heartbeat_update();
        }

        send_bytes(chunk_buf, chunk);

        /* Wait for host ACK before sending next chunk */
        /* Read "ACK\n" (4 bytes) */
        uint8_t ack[8];
        size_t ack_len = 0;
        bool got_ack = false;
        for (int t = 0; t < 8; t++) {
            int c = recv_byte_timeout(5000000u);
            if (c < 0) break;
            if (c == '\n') { got_ack = true; break; }
            if (ack_len < sizeof(ack) - 1) ack[ack_len++] = (uint8_t)c;
        }

        if (!got_ack) {
            flash_bus_release();
            send_line("ERR Timeout waiting for ACK");
            return;
        }

        offset += chunk;
    }

    flash_bus_release();
    send_line("DONE");
}

/* -------------------------------------------------------------------------
 * Command dispatcher
 * ---------------------------------------------------------------------- */

static void handle_command(const char *cmd) {
    if (strcmp(cmd, "VER") == 0) {
        printf("OK %s\n", FIRMWARE_VERSION);
        stdio_flush();

    } else if (strcmp(cmd, "BOOT") == 0) {
        send_line("OK");
        sleep_ms(100);
        reset_usb_boot(0, 0);

    } else if (strncmp(cmd, "FLASH ", 6) == 0) {
        uint32_t size = (uint32_t)atol(cmd + 6);
        cmd_flash(size);

    } else if (strncmp(cmd, "VERIFY ", 7) == 0) {
        uint32_t size = (uint32_t)atol(cmd + 7);
        cmd_verify(size);

    } else if (strcmp(cmd, "DUMP") == 0) {
        cmd_dump();

    } else {
        send_line("ERR Unknown command");
    }
}

/* -------------------------------------------------------------------------
 * Main
 * ---------------------------------------------------------------------- */

int main(void) {
    stdio_init_all();

    /* LED0: on while firmware is running */
    gpio_init(LED0_PIN);
    gpio_set_dir(LED0_PIN, GPIO_OUT);
    gpio_put(LED0_PIN, 1);

    /* LED1: 2 Hz heartbeat */
    gpio_init(LED1_PIN);
    gpio_set_dir(LED1_PIN, GPIO_OUT);
    gpio_put(LED1_PIN, 0);
    led1_next = make_timeout_time_ms(500);

    /* Initialise flash bus GPIO (all inactive/input) */
    flash_gpio_init();

    /* Wait for USB host to connect */
    while (!stdio_usb_connected()) {
        heartbeat_update();
        sleep_ms(10);
    }

    /* Main command loop */
    char cmd_buf[CMD_BUF_SIZE];
    size_t cmd_len = 0;

    while (true) {
        heartbeat_update();

        if (readline_nonblocking(cmd_buf, CMD_BUF_SIZE, &cmd_len)) {
            handle_command(cmd_buf);
            cmd_len = 0;
        }
    }

    return 0;
}
