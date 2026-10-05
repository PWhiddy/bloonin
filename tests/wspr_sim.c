/* Runs the unmodified production beacon/GPS loops in two host threads. Each
 * core has a virtual clock; sleep rendezvous keep them within one poll of each
 * other. Only Pico SDK/peripheral interfaces are replaced. Run one scenario per
 * process so production static state has the same lifetime as a firmware boot. */
#define WSPR_SIMULATION 1
#include "sim_pico.h"
#include <assert.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <sched.h>
#include <stdarg.h>
#include <stdatomic.h>
#include <stdlib.h>
#include <string.h>

static int sim_printf(const char *format, ...);
#define printf sim_printf
#include "wspr_beacon.h"
#undef printf

#define SECOND 1000000ll
#define MAX_BYTES 400000
#define MAX_LOGS 4096
#define MAX_FRAMES 32

typedef struct { int64_t at; char byte; } input_byte_t;
typedef struct { int64_t at; char text[512]; bool delivered; } log_t;
typedef struct { int64_t at; double hz; } tone_t;
typedef struct {
    int64_t start, end;
    unsigned count;
    tone_t tones[162];
} frame_t;

static struct {
    const char *name;
    int64_t duration;
    int utc_origin;
    bool usb, usb_cycle, usb_blocked, gps;
    bool time_only, location_only, malformed, fractional, midnight;
    bool noisy, moving, silence, reacquire, no_altitude;
    bool gps_reset_initialized, gps_reset_released;
    bool gps_power_initialized, gps_power_low_preloaded, gps_power_enabled;
    int64_t gps_power_enabled_at;
    int64_t fix_at, loss_at, recover_at, move_at;
    int64_t first_sentence;
    const char *fault;
    input_byte_t uart[MAX_BYTES], serial[128];
    size_t uart_size, uart_read, uart_overruns, serial_size, serial_read;
    uint8_t registers[256];
    bool rf;
    uint8_t outputs_ever_enabled;
    uint8_t unused_outputs_ever_powered;
    unsigned fault_count;
    frame_t frames[MAX_FRAMES];
    unsigned frame_count;
    log_t logs[MAX_LOGS];
    unsigned log_count, dropped;
    pthread_mutex_t log_lock;
    pthread_t threads[2];
    atomic_int_fast64_t ready[2];
    void (*core1_entry)(void);
} sim;
static _Thread_local unsigned core;
static _Thread_local int64_t now;

absolute_time_t get_absolute_time(void) { return now; }

void sleep_until(absolute_time_t target) {
    if (target <= now) return;
    if (target >= sim.duration) {
        atomic_store(&sim.ready[core], INT64_MAX);
        pthread_exit(NULL);
    }
    atomic_store(&sim.ready[core], target);
    unsigned spins = 0;
    while (atomic_load(&sim.ready[1u - core]) < target) {
        if (++spins % 1024u == 0) sched_yield();
    }
    now = target;
}
void sleep_us(uint64_t us) { sleep_until(now + (int64_t)us); }
void sleep_ms(uint32_t ms) { sleep_us(ms * 1000ull); }

static bool usb_at(int64_t at) {
    return sim.usb && (!sim.usb_cycle || at < 20 * SECOND || at >= 150 * SECOND);
}
bool stdio_usb_connected(void) { return usb_at(now); }

static int sim_printf(const char *format, ...) {
    char text[512];
    va_list args;
    va_start(args, format);
    int n = vsnprintf(text, sizeof(text), format, args);
    va_end(args);
    assert(pthread_mutex_lock(&sim.log_lock) == 0);
    assert(sim.log_count < MAX_LOGS);
    log_t *log = &sim.logs[sim.log_count++];
    log->at = now;
    memcpy(log->text, text, sizeof(text));
    /* Model a full output buffer when the connected host stops reading. With
     * timeout zero, neither this nor a disconnected host advances the clock. */
    log->delivered = usb_at(now) && !sim.usb_blocked;
    if (!log->delivered) ++sim.dropped;
    assert(pthread_mutex_unlock(&sim.log_lock) == 0);
    return n;
}

void mutex_init(mutex_t *m) { assert(pthread_mutex_init(m, NULL) == 0); }
void mutex_enter_blocking(mutex_t *m) { assert(pthread_mutex_lock(m) == 0); }
bool mutex_try_enter(mutex_t *m, uint32_t *owner) {
    (void)owner;
    int result = pthread_mutex_trylock(m);
    assert(result == 0 || result == EBUSY);
    return result == 0;
}
void mutex_exit(mutex_t *m) { assert(pthread_mutex_unlock(m) == 0); }

static void *run_core1(void *unused) {
    (void)unused;
    core = 1;
    sim.core1_entry();
    abort();
}
void multicore_launch_core1(void (*entry)(void)) {
    sim.core1_entry = entry;
    assert(pthread_create(&sim.threads[1], NULL, run_core1, NULL) == 0);
}
static void *run_core0(void *unused) {
    (void)unused;
    core = 0;
    wspr_run_beacon();
    abort();
}

bool gpio_get(uint pin) { return pin == C90770_UART_RX_GPIO; }
uint gpio_get_function(uint pin) {
    return pin == C90770_UART_RX_GPIO ? GPIO_FUNC_UART :
        ((pin == C90770_RESET_GPIO && sim.gps_reset_initialized) ||
         (pin == C90770_POWER_ENABLE_GPIO && sim.gps_power_initialized)) ? 5u : 31u;
}
uint gpio_get_dir(uint pin) {
    return pin == C90770_POWER_ENABLE_GPIO && sim.gps_power_enabled ? GPIO_OUT : 0u;
}
bool gpio_is_pulled_up(uint pin) { return pin == C90770_UART_RX_GPIO; }
bool gpio_is_pulled_down(uint pin) { (void)pin; return false; }
void gpio_init(uint pin) {
    if (pin == C90770_RESET_GPIO) sim.gps_reset_initialized = true;
    if (pin == C90770_POWER_ENABLE_GPIO) sim.gps_power_initialized = true;
}
void gpio_disable_pulls(uint pin) {
    if (pin == C90770_RESET_GPIO) {
        assert(sim.gps_reset_initialized);
        sim.gps_reset_released = true;
    } else {
        assert(pin == C90770_POWER_ENABLE_GPIO && sim.gps_power_initialized);
    }
}
void gpio_put(uint pin, bool value) {
    if (pin == C90770_POWER_ENABLE_GPIO) {
        assert(sim.gps_power_initialized && !value);
        sim.gps_power_low_preloaded = true;
    }
}
void gpio_set_dir(uint pin, bool output) {
    if (pin == C90770_POWER_ENABLE_GPIO) {
        assert(output && sim.gps_reset_released && sim.gps_power_low_preloaded);
        sim.gps_power_enabled = true;
        sim.gps_power_enabled_at = now;
    }
}
void gpio_set_function(uint pin, uint function) { (void)pin; (void)function; }
void gpio_pull_up(uint pin) { (void)pin; }
uint i2c_init(i2c_inst_t *i, uint baud) { assert(i == i2c0 && baud == 400000); return baud; }
uint uart_init(uart_inst_t *u, uint baud) {
    assert(u == uart1 && baud == 9600 && sim.gps_reset_released);
    assert(sim.gps_power_enabled && now - sim.gps_power_enabled_at >= 10000);
    return baud;
}
void uart_set_format(uart_inst_t *u, uint bits, uint stops, uint parity) {
    assert(u == uart1 && bits == 8 && stops == 1 && parity == UART_PARITY_NONE);
}
void uart_set_fifo_enabled(uart_inst_t *u, bool enabled) { assert(u == uart1 && enabled); }

bool uart_is_readable(uart_inst_t *u) {
    assert(u == uart1 && core == 1);
    /* A 32-byte UART FIFO loses older data if the consumer falls behind. */
    size_t end = sim.uart_read;
    while (end < sim.uart_size && sim.uart[end].at <= now) ++end;
    if (end - sim.uart_read > 32) {
        sim.uart_overruns += end - sim.uart_read - 32;
        sim.uart_read = end - 32;
    }
    return sim.uart_read < end;
}
bool uart_is_readable_within_us(uart_inst_t *u, uint32_t us) {
    if (uart_is_readable(u)) return true;
    sleep_us(us);
    return uart_is_readable(u);
}
char uart_getc(uart_inst_t *u) {
    assert(uart_is_readable(u));
    return sim.uart[sim.uart_read++].byte;
}
uart_hw_t *uart_get_hw(uart_inst_t *u) {
    static uart_hw_t hw;
    hw.dr = (uint8_t)uart_getc(u);
    if (!strcmp(sim.name, "diag_uart_errors")) hw.dr |= 0xf00u;
    return &hw;
}
int getchar_timeout_us(uint32_t us) {
    assert(core == 0 && us == 0);
    while (sim.serial_read < sim.serial_size && sim.serial[sim.serial_read].at <= now) {
        input_byte_t byte = sim.serial[sim.serial_read++];
        if (usb_at(byte.at) && usb_at(now)) return byte.byte;
    }
    return PICO_ERROR_TIMEOUT;
}

static double output_hz(void) {
    const uint8_t *r = sim.registers;
    uint32_t p3 = ((r[47] & 0xf0u) << 12) | (r[42] << 8) | r[43];
    uint32_t p1 = ((r[44] & 3u) << 16) | (r[45] << 8) | r[46];
    uint32_t p2 = ((r[47] & 15u) << 16) | (r[48] << 8) | r[49];
    assert(p3 != 0);
    return SI5351A_PLL_HZ / ((p1 + 512.0 + (double)p2 / p3) / 128.0);
}
static void record_tone(void) {
    frame_t *frame = &sim.frames[sim.frame_count - 1u];
    assert(frame->count < 162);
    frame->tones[frame->count++] = (tone_t){now, output_hz()};
}
int i2c_write_blocking(i2c_inst_t *i, uint8_t addr, const uint8_t *data, size_t n, bool nostop) {
    (void)nostop;
    assert(core == 0 && i == i2c0 && addr == 0x60 && n > 0);
    /* Account for address/data/ACK wire time at 400 kHz. */
    sleep_us(((n + 1u) * 9u * SECOND + 399999u) / 400000u);
    bool fail = false;
    if (sim.fault) {
        if (!strcmp(sim.fault, "init")) fail = data[0] == 42 && sim.fault_count == 0;
        if (!strcmp(sim.fault, "start")) fail = n == 2 && data[0] == 3 && !(data[1] & 1u);
        if (!strcmp(sim.fault, "symbol")) fail = sim.rf && data[0] >= 42 && now >= 20 * SECOND && sim.fault_count == 0;
        if (!strcmp(sim.fault, "disable")) fail = sim.rf && n == 2 && data[0] == 3 && (data[1] & 1u);
    }
    if (fail) { ++sim.fault_count; return -1; }
    for (size_t j = 1; j < n; ++j) sim.registers[data[0] + j - 1] = data[j];
    /* Observe all eight outputs after every write, including initialization.
     * A silent CLK0 alone does not imply the clock generator is silent. */
    sim.outputs_ever_enabled |= (uint8_t)~sim.registers[3];
    for (unsigned output = 1; output < 8; ++output) {
        if (!(sim.registers[16 + output] & 0x80u)) {
            sim.unused_outputs_ever_powered |= (uint8_t)(1u << output);
        }
    }
    if (n > 1 && data[0] == 3) {
        bool rf = !(sim.registers[3] & 1u);
        if (rf && !sim.rf) {
            assert(sim.frame_count < MAX_FRAMES);
            frame_t *frame = &sim.frames[sim.frame_count++];
            frame->start = now;
            record_tone();
        } else if (!rf && sim.rf) {
            sim.frames[sim.frame_count - 1].end = now;
        }
        sim.rf = rf;
    } else if (sim.rf && n > 1 && (data[0] == 42 || data[0] == 47)) {
        record_tone();
    }
    return (int)n;
}
int i2c_read_blocking(i2c_inst_t *i, uint8_t addr, uint8_t *data, size_t n, bool nostop) {
    (void)nostop;
    assert(i == i2c0 && addr == 0x60);
    sleep_us(((n + 1u) * 9u * SECOND + 399999u) / 400000u);
    memset(data, 0, n);
    return (int)n;
}

static void serial_byte(int64_t at, char byte) {
    assert(sim.serial_size < 128);
    sim.serial[sim.serial_size++] = (input_byte_t){at, byte};
}
static void uart_text(int64_t at, const char *text) {
    for (size_t j = 0; text[j]; ++j) {
        assert(sim.uart_size < MAX_BYTES);
        /* One start, eight data and one stop bit at 9600 baud. */
        sim.uart[sim.uart_size++] = (input_byte_t){at + (int64_t)j * 10000000ll / 9600, text[j]};
    }
}
static void sentence(int64_t at, const char *payload, bool corrupt) {
    uint8_t checksum = 0;
    for (const char *p = payload; *p; ++p) checksum ^= (uint8_t)*p;
    char line[256];
    snprintf(line, sizeof(line), "$%s*%02X\r\n", payload, checksum ^ (corrupt ? 1u : 0u));
    uart_text(at, line);
}
static int compare_bytes(const void *a, const void *b) {
    int64_t x = ((const input_byte_t *)a)->at, y = ((const input_byte_t *)b)->at;
    return (x > y) - (x < y);
}

static void configure(const char *name) {
    sim.name = name;
    sim.duration = 245 * SECOND;
    sim.utc_origin = 43190; // 11:59:50: first slot at virtual t=11 s.
    sim.fix_at = 2 * SECOND;
    sim.loss_at = sim.recover_at = sim.move_at = INT64_MAX;
    sim.gps = true;
    sim.usb = true;
    if (!strcmp(name, "no_inputs")) { sim.gps = sim.usb = false; sim.duration = 130 * SECOND; }
    else if (!strcmp(name, "diag_no_data") || !strcmp(name, "diag_uart_errors")) {
        sim.gps = false; sim.duration = 8 * SECOND;
        if (!strcmp(name, "diag_uart_errors")) uart_text(SECOND, "\xff\x55\x7f");
        serial_byte(6 * SECOND, 'd');
    }
    else if (!strcmp(name, "usb_waiting")) { sim.gps = false; sim.duration = 130 * SECOND; }
    else if (!strcmp(name, "gps_no_usb")) sim.usb = false;
    else if (!strcmp(name, "gps_usb")) { }
    else if (!strcmp(name, "serial_only")) {
        sim.gps = false;
        serial_byte(SECOND, 'x'); serial_byte(2 * SECOND, '\n');
        serial_byte(11 * SECOND, 'g'); serial_byte(131 * SECOND, 'g');
        serial_byte(12 * SECOND, 'g'); serial_byte(20 * SECOND, 'g'); serial_byte(120 * SECOND, 'g');
    } else if (!strcmp(name, "simultaneous")) {
        serial_byte(11 * SECOND, 'g'); serial_byte(11 * SECOND + 500, 'g');
        serial_byte(131 * SECOND, 'g');
    } else if (!strcmp(name, "gps_during_serial_frame")) {
        serial_byte(SECOND, 'g'); sim.fix_at = 5 * SECOND;
    } else if (!strcmp(name, "gps_loss")) sim.loss_at = 20 * SECOND;
    else if (!strcmp(name, "gps_silent")) { sim.loss_at = 20 * SECOND; sim.silence = true; }
    else if (!strcmp(name, "gps_reacquire")) {
        sim.loss_at = 20 * SECOND; sim.recover_at = 125 * SECOND; sim.move_at = 125 * SECOND;
    } else if (!strcmp(name, "moving")) { sim.move_at = 30 * SECOND; sim.duration = 725 * SECOND; }
    else if (!strcmp(name, "no_altitude")) sim.no_altitude = true;
    else if (!strcmp(name, "gps_priority")) {
        sim.duration = 725 * SECOND;
        serial_byte(20 * SECOND, 'd'); /* Diagnostics during the first RF frame. */
        serial_byte(10 * SECOND, 'g'); serial_byte(125 * SECOND, 'g');
        serial_byte(245 * SECOND, 'g'); serial_byte(370 * SECOND, 'g');
    } else if (!strcmp(name, "gps_late_reacquire")) {
        sim.loss_at = 20 * SECOND; sim.recover_at = 400 * SECOND;
        sim.move_at = 400 * SECOND; sim.duration = 845 * SECOND;
    }
    else if (!strcmp(name, "midnight")) sim.utc_origin = 86390;
    else if (!strcmp(name, "fractional")) sim.fractional = true;
    else if (!strcmp(name, "near_slot_before")) sim.first_sentence = sim.fix_at = 10600000;
    else if (!strcmp(name, "altitude_after_slot")) sim.first_sentence = sim.fix_at = 10800000;
    else if (!strcmp(name, "near_slot_after")) sim.first_sentence = sim.fix_at = 10980000;
    else if (!strcmp(name, "time_only")) sim.time_only = true;
    else if (!strcmp(name, "location_only")) sim.location_only = true;
    else if (!strcmp(name, "malformed")) sim.malformed = true;
    else if (!strcmp(name, "noisy_uart")) sim.noisy = true;
    else if (!strcmp(name, "usb_reconnect")) sim.usb_cycle = true;
    else if (!strcmp(name, "usb_backpressure")) {
        sim.usb_blocked = true; serial_byte(11 * SECOND, 'g');
    } else if (!strcmp(name, "i2c_init_failure")) sim.fault = "init";
    else if (!strcmp(name, "i2c_start_failure")) sim.fault = "start";
    else if (!strcmp(name, "i2c_symbol_failure")) sim.fault = "symbol";
    else if (!strcmp(name, "i2c_disable_failure")) sim.fault = "disable";
    else if (!strcmp(name, "long_holdover")) {
        sim.usb = false; sim.loss_at = 20 * SECOND; sim.silence = true; sim.duration = 1205 * SECOND;
    } else { fprintf(stderr, "Unknown scenario: %s\n", name); exit(2); }

    if (!strcmp(name, "gps_loss") || !strcmp(name, "gps_silent")) sim.duration = 725 * SECOND;
    if (sim.noisy) {
        uart_text(100000, "noise$GPRMC,truncated");
        char oversized[202]; memset(oversized, 'x', sizeof(oversized));
        oversized[0] = '$'; oversized[200] = '\n'; oversized[201] = 0;
        uart_text(200000, oversized);
        sentence(500000, "GPRMC,115950,A,4807.038,N,01131.000,E,0,0,050926,,,A", true);
    }
    if (sim.gps) {
        int64_t initial = sim.first_sentence ? sim.first_sentence : SECOND;
        if (sim.fractional) initial += 250000;
        for (int64_t at = initial; at + SECOND < sim.duration; at += SECOND) {
            bool lost = at >= sim.loss_at && at < sim.recover_at;
            if (lost && sim.silence) continue;
            bool valid = at >= sim.fix_at && !lost;
            int64_t utc_us = (sim.utc_origin * SECOND + at) % (86400 * SECOND);
            unsigned sec = (unsigned)(utc_us / SECOND);
            char utc[32];
            snprintf(utc, sizeof(utc), "%02u%02u%02u.%06u", sec / 3600, sec / 60 % 60,
                     sec % 60, (unsigned)(utc_us % SECOND));
            const char *coords = at >= sim.move_at ? "4100.000,N,07200.000,W" : "4807.038,N,01131.000,E";
            char payload[220];
            snprintf(payload, sizeof(payload), "GPRMC,%s,%c,%s,0,0,050926,,,A",
                     sim.location_only ? "badtime" : utc, valid ? 'A' : 'V',
                     sim.time_only ? ",,," : sim.malformed ? "nan,N,01131.000,E" : coords);
            sentence(at, payload, false);
            snprintf(payload, sizeof(payload), "GPGGA,%s,%s,%d,08,1.0,%s,M,0,M,,",
                     utc, sim.time_only ? ",,," : sim.malformed ? "nan,N,01131.000,E" : coords,
                     valid ? 1 : 0, sim.no_altitude ? "" : at >= sim.move_at ? "456.5" : "123.0");
            sentence(at + 150000, payload, false);
            /* Repeat stable status and vary SNR each second to test log noise. */
            snprintf(payload, sizeof(payload), "GPGSV,1,1,02,01,45,180,%d,02,40,090,35", sec % 2 ? 30 : 31);
            sentence(at + 300000, payload, false);
            sentence(at + 450000, "GPTXT,01,01,02,ANTENNA OK", false);
        }
    }
    qsort(sim.uart, sim.uart_size, sizeof(sim.uart[0]), compare_bytes);
    qsort(sim.serial, sim.serial_size, sizeof(sim.serial[0]), compare_bytes);
}

static void json_string(const char *s) {
    putchar('"');
    for (; *s; ++s) {
        if (*s == '"' || *s == '\\') putchar('\\');
        if (*s == '\n') fputs("\\n", stdout);
        else if (*s == '\r') fputs("\\r", stdout);
        else if ((unsigned char)*s < 32) printf("\\u%04x", (unsigned char)*s);
        else putchar(*s);
    }
    putchar('"');
}
static void report(void) {
    printf("{\"scenario\":\"%s\",\"duration_us\":%lld,\"uart_bytes\":%zu,\"uart_read\":%zu,"
           "\"uart_overruns\":%zu,\"serial_read\":%zu,\"fault_count\":%u,\"rf_enabled\":%s,"
           "\"dropped_logs\":%u,\"gps_revision\":%u,\"checksum_failures\":%u,"
           "\"outputs_ever_enabled\":%u,\"unused_outputs_ever_powered\":%u,"
           "\"output_disable_mask\":%u,\"callsign\":\"%s\",\"fallback_grid\":\"%s\","
           "\"callsign_fine\":\"%s\",\"callsign_alt\":\"%s\",\"power_dbm\":%u,\"frames\":[",
           sim.name, (long long)sim.duration, sim.uart_size, sim.uart_read, sim.uart_overruns,
           sim.serial_read, sim.fault_count, sim.rf ? "true" : "false", sim.dropped,
           wspr_gps_revision, wspr_gps_shared.checksum_failures,
           sim.outputs_ever_enabled, sim.unused_outputs_ever_powered, sim.registers[3],
           WSPR_CALLSIGN, WSPR_FALLBACK_GRID, WSPR_CALLSIGN_FINE, WSPR_CALLSIGN_ALT, (unsigned)WSPR_POWER_DBM);
    for (unsigned i = 0; i < sim.frame_count; ++i) {
        frame_t *frame = &sim.frames[i];
        printf("%s{\"start_us\":%lld,\"end_us\":%lld,\"tones\":[", i ? "," : "",
               (long long)frame->start, (long long)frame->end);
        for (unsigned j = 0; j < frame->count; ++j) {
            printf("%s[%lld,%.9f]", j ? "," : "", (long long)frame->tones[j].at, frame->tones[j].hz);
        }
        fputs("]}", stdout);
    }
    fputs("],\"logs\":[", stdout);
    for (unsigned i = 0; i < sim.log_count; ++i) {
        printf("%s{\"at_us\":%lld,\"delivered\":%s,\"text\":", i ? "," : "",
               (long long)sim.logs[i].at, sim.logs[i].delivered ? "true" : "false");
        json_string(sim.logs[i].text);
        putchar('}');
    }
    puts("]}");
}

int main(int argc, char **argv) {
    if (argc != 2) { fprintf(stderr, "Usage: %s SCENARIO\n", argv[0]); return 2; }
    configure(argv[1]);
    memset(sim.registers, 0xff, sizeof(sim.registers));
    assert(pthread_mutex_init(&sim.log_lock, NULL) == 0);
    assert(pthread_create(&sim.threads[0], NULL, run_core0, NULL) == 0);
    assert(pthread_join(sim.threads[0], NULL) == 0);
    assert(pthread_join(sim.threads[1], NULL) == 0);
    report();
    return 0;
}
