#ifndef WSPR_BEACON_H
#define WSPR_BEACON_H

#include "pico/multicore.h"
#include "pico/mutex.h"
#include "pico/stdio_usb.h"
#include "wspr.h"
#include "gps_diagnostics.h"

#define WSPR_GPS_FRESH_US 5000000ll
#define WSPR_GPS_PROGRESS_LOG_US 10000000ll

typedef struct {
    bool initialized;
    int uart_state;
    bool fix;
    bool utc;
    int fix_quality;
    int fix_type;
    int seen, tracked, used;
    uint32_t checksum_failures;
    absolute_time_t progress_at;
    char text[96];
} wspr_gps_log_state_t;

static inline void wspr_log_gps_status(
    wspr_gps_log_state_t *log, const c90770_gps_monitor_state_t *gps,
    bool received_any, absolute_time_t last_byte, absolute_time_t now
) {
    int uart_state = !received_any ? 0 :
        (absolute_time_diff_us(last_byte, now) < WSPR_GPS_FRESH_US ? 1 : 2);
    bool fix = uart_state == 1 && gps->have_coordinates;
    bool utc = uart_state == 1 && gps->have_utc_time &&
        absolute_time_diff_us(gps->utc_captured_at, now) < WSPR_GPS_FRESH_US;
    if (!log->initialized || log->uart_state != uart_state || log->fix != fix ||
        log->utc != utc || log->fix_quality != gps->fix_quality || log->fix_type != gps->fix_type) {
        const char *uart_states[] = {"waiting for data", "receiving", "silent (>5 s)"};
        printf("GPS STATE: UART=%s; fix=%s (quality=%d type=%d); UTC=%s\n",
               uart_states[uart_state], fix ? "valid" : "searching/unavailable",
               gps->fix_quality, gps->fix_type, utc ? "valid" : "unavailable/stale");
        log->initialized = true;
        log->uart_state = uart_state;
        log->fix = fix;
        log->utc = utc;
        log->fix_quality = gps->fix_quality;
        log->fix_type = gps->fix_type;
    }
    if (strcmp(log->text, gps->status_text) != 0) {
        memcpy(log->text, gps->status_text, sizeof(log->text));
        printf("GPS MODULE: %s\n", log->text);
    }

    /* Signal strengths fluctuate on every GSV. Report counts only, at most
     * once per ten seconds, and never repeat an unchanged progress summary. */
    if (absolute_time_diff_us(log->progress_at, now) >= 0ll) {
        int seen = c90770_satellites_in_view_reported(gps);
        int tracked = 0;
        for (size_t i = 0; i < gps->satellite_count; ++i) {
            if (gps->satellites[i].has_snr && gps->satellites[i].snr > 0) ++tracked;
        }
        if (seen != log->seen || tracked != log->tracked || gps->satellites_used != log->used ||
            gps->checksum_failures != log->checksum_failures) {
            printf("GPS PROGRESS: in_view=%d tracked=%d used=%d checksum_failures=%lu\n",
                   seen, tracked, gps->satellites_used, (unsigned long)gps->checksum_failures);
            log->seen = seen;
            log->tracked = tracked;
            log->used = gps->satellites_used;
            log->checksum_failures = gps->checksum_failures;
            log->progress_at = delayed_by_us(now, WSPR_GPS_PROGRESS_LOG_US);
        }
    }
}

/* Core 1 owns the UART and parser. Only a short snapshot copy holds this lock;
 * neither printing nor I/O runs under it. Core 0 owns USB input and RF timing. */
static mutex_t wspr_gps_mutex;
static c90770_gps_monitor_state_t wspr_gps_shared;
static uint32_t wspr_gps_revision;
static bool wspr_gps_log_requested;
static bool wspr_gps_diag_requested;

static void wspr_monitor_gps(void) {
    c90770_uart_t uart;
    c90770_uart_init_default(&uart);
    /* Satellite tables exceed the default core-1 stack size. */
    static c90770_gps_monitor_state_t gps = {.quiet = true};
    c90770_nmea_stream_t stream = {0};
    wspr_gps_log_state_t log = {0};
    absolute_time_t last_byte = get_absolute_time();
    bool received_any = false;
    gps_diagnostics_t diag = {0};
    absolute_time_t next_diag = get_absolute_time();

    while (true) {
        if (uart_is_readable(uart.uart)) {
            /* Read UARTDR once, preserving error flags discarded by uart_getc. */
            uint32_t data = uart_get_hw(uart.uart)->dr;
            char byte = (char)(data & 0xffu);
            last_byte = get_absolute_time();
            gps_diagnostics_record(&diag, data, last_byte);
            received_any = true;
            if (c90770_nmea_stream_push(&stream, byte, last_byte)) {
                ++diag.sentences;
                c90770_parse_gps_line_at(&gps, stream.line, stream.started_at);
                mutex_enter_blocking(&wspr_gps_mutex);
                wspr_gps_shared = gps;
                ++wspr_gps_revision;
                mutex_exit(&wspr_gps_mutex);
            }
        } else {
            sleep_us(100u);
        }
        bool report_diag = false;
        if (mutex_try_enter(&wspr_gps_mutex, NULL)) {
            if (wspr_gps_diag_requested && time_reached(next_diag)) {
                wspr_gps_diag_requested = false;
                report_diag = true;
            }
            if (wspr_gps_log_requested) {
                wspr_gps_log_requested = false;
                log.initialized = false;
                log.seen = -1;
                log.progress_at = get_absolute_time();
            }
            mutex_exit(&wspr_gps_mutex);
        }
        if (report_diag) {
            gps_diagnostics_report(&diag, gps.checksum_failures, get_absolute_time());
            next_diag = make_timeout_time_ms(1000u);
        }
        wspr_log_gps_status(&log, &gps, received_any, last_byte, get_absolute_time());
    }
}

typedef struct {
    bool armed;
    bool gps_slot_selected;
    bool gps_mode;
    unsigned phase; /* Next GPS message: coarse, fine, altitude. */
    uint16_t payloads[3];
    char snapshot_grid[5];
    absolute_time_t next_start;
    char grid[5];
} wspr_beacon_schedule_t;

static inline bool wspr_gps_fix_fresh(
    const c90770_gps_monitor_state_t *gps, absolute_time_t now
) {
    int64_t age = absolute_time_diff_us(gps->fix_captured_at, now);
    int64_t utc_age = absolute_time_diff_us(gps->utc_captured_at, now);
    return gps->have_coordinates && gps->have_complete_fix && gps->have_utc_time &&
        age >= 0 && age < WSPR_GPS_FRESH_US &&
        utc_age >= 0 && utc_age < WSPR_GPS_FRESH_US;
}

/* Called only between frames. Once GPS takes over, serial stays ignored even
 * during GPS loss. The current sequence retains its own immutable payloads. */
static inline bool wspr_beacon_use_gps(
    wspr_beacon_schedule_t *schedule, const c90770_gps_monitor_state_t *gps,
    absolute_time_t now
) {
    int64_t age = absolute_time_diff_us(gps->utc_captured_at, now);
    char grid[5];
    if ((!schedule->gps_mode && !wspr_gps_fix_fresh(gps, now)) ||
        !gps->have_coordinates || !gps->have_utc_time || age < 0ll ||
        age >= WSPR_GPS_FRESH_US ||
        !wspr_grid_from_coordinates(gps->latitude_degrees, gps->longitude_degrees, grid)) {
        return false;
    }
    schedule->gps_mode = true;
    memcpy(schedule->grid, grid, sizeof(schedule->grid));
    if (!schedule->gps_slot_selected) {
        schedule->next_start = wspr_next_utc_slot_at(gps, now);
        schedule->armed = true;
        schedule->gps_slot_selected = true;
    }
    return true;
}

static inline bool wspr_beacon_should_start(
    const wspr_beacon_schedule_t *schedule, bool transmitting, bool trigger,
    absolute_time_t now
) {
    return !transmitting && ((trigger && !schedule->gps_mode) || (schedule->armed &&
        absolute_time_diff_us(schedule->next_start, now) >= 0ll));
}

static inline void wspr_beacon_did_start(
    wspr_beacon_schedule_t *schedule, absolute_time_t start
) {
    if (schedule->gps_mode) schedule->phase = (schedule->phase + 1u) % 3u;
    schedule->armed = true;
    schedule->next_start = delayed_by_us(start, WSPR_SLOT_PERIOD_US);
    /* Select the next slot from the newest fix after this frame completes. */
    schedule->gps_slot_selected = false;
}

/* Called at the slot, so a fix that went stale while waiting cannot launch
 * a new sequence. Missing fixes skip this slot without advancing the phase. */
static inline bool wspr_beacon_message(
    wspr_beacon_schedule_t *schedule, const c90770_gps_monitor_state_t *gps,
    absolute_time_t now, const char **callsign, uint16_t *payload
) {
    static const char *const callsigns[] = {
        WSPR_CALLSIGN, WSPR_CALLSIGN_FINE, WSPR_CALLSIGN_ALT
    };
    if (!schedule->gps_mode) {
        *callsign = WSPR_CALLSIGN;
        return wspr_grid_payload(schedule->grid, payload);
    }
    if (schedule->phase == 0u &&
        (!wspr_gps_fix_fresh(gps, now) ||
         !wspr_location_payloads(gps->fix_latitude, gps->fix_longitude,
             gps->fix_altitude, schedule->snapshot_grid, &schedule->payloads[0],
             &schedule->payloads[1], &schedule->payloads[2]))) {
        schedule->next_start = delayed_by_us(schedule->next_start, WSPR_SLOT_PERIOD_US);
        schedule->gps_slot_selected = false;
        return false;
    }
    *callsign = callsigns[schedule->phase];
    *payload = schedule->payloads[schedule->phase];
    return true;
}

static inline void wspr_stop_on_error(si5351a_i2c_t *clock, const char *reason) {
    si5351a_error_t error = clock->last_error;
    uint8_t reg = clock->last_reg;
    bool disabled = si5351a_i2c_set_clk0_enabled(clock, false);
    bool first = true;
    bool was_connected = false;
    while (true) {
        bool connected = stdio_usb_connected();
        if (first || (connected && !was_connected)) {
            printf("WSPR ERROR: %s; Si5351A=%s (reg 0x%02x); RF STATE: %s; beacon halted\n",
                   reason, si5351a_i2c_error_string(error), reg,
                   disabled ? "NOT TRANSMITTING" : "UNKNOWN");
            mutex_enter_blocking(&wspr_gps_mutex);
            wspr_gps_log_requested = true;
            mutex_exit(&wspr_gps_mutex);
        }
        first = false;
        was_connected = connected;
        sleep_ms(100u);
    }
}

/* This loop owns the application while the beacon is enabled. */
static inline void wspr_run_beacon(void) {
    mutex_init(&wspr_gps_mutex);
    multicore_launch_core1(wspr_monitor_gps);

    si5351a_i2c_t clock;
    if (!si5351a_i2c_prepare_output_hz(&clock, WSPR_BASE_HZ) ||
        !si5351a_i2c_set_clk0_enabled(&clock, false)) {
        wspr_stop_on_error(&clock, "initialization failed");
    }

    wspr_beacon_schedule_t schedule = {.grid = WSPR_FALLBACK_GRID};
    wspr_transmitter_t tx = {0};
    const char *last_callsign = WSPR_CALLSIGN;
    uint16_t last_payload = 0u;
    wspr_grid_payload(schedule.grid, &last_payload);
    if (!wspr_prepare(&clock, &tx, WSPR_CALLSIGN, schedule.grid, WSPR_POWER_DBM)) {
        wspr_stop_on_error(&clock, "preparation failed; check station fields and Si5351A");
    }
    printf("WSPR RF STATE: NOT TRANSMITTING; base=%lu Hz callsign=%s power=%u dBm\n",
           (unsigned long)WSPR_BASE_HZ, WSPR_CALLSIGN, (unsigned)WSPR_POWER_DBM);
    printf("WSPR waiting for serial 'g' or GPS UTC + location + altitude; placeholder grid=%s; 'd'=GPS diagnostics\n",
           schedule.grid);

    static c90770_gps_monitor_state_t gps;
    uint32_t revision = 0u;
    bool refresh_gps = false;
    bool usb_connected = false;
    bool request_diag = false;
    while (true) {
        bool connected = stdio_usb_connected();
        if (connected && !usb_connected) {
            /* Boot logs may predate the terminal opening. A connection is a
             * transition too: show the current state once, without heartbeats. */
            printf("WSPR STATUS: RF=%s; base=%lu Hz; last/prepared=%s payload=%u %u dBm; schedule=%s\n",
                   tx.active ? "TRANSMITTING" : "NOT TRANSMITTING",
                   (unsigned long)WSPR_BASE_HZ, last_callsign, (unsigned)last_payload,
                   (unsigned)WSPR_POWER_DBM, schedule.armed ? "armed" : "waiting for 'g' or GPS");
            mutex_enter_blocking(&wspr_gps_mutex);
            wspr_gps_log_requested = true;
            mutex_exit(&wspr_gps_mutex);
        }
        usb_connected = connected;
        /* Drain input even on air, so a simultaneous GPS/serial start or extra
         * 'g' cannot become a queued transmission after this frame. */
        bool trigger = false;
        for (unsigned i = 0; i < 64u; ++i) {
            int byte = getchar_timeout_us(0u);
            if (byte == PICO_ERROR_TIMEOUT) break;
            if (byte == 'g' && !tx.active) trigger = true;
            if (byte == 'd') request_diag = true;
        }
        if (request_diag && mutex_try_enter(&wspr_gps_mutex, NULL)) {
            wspr_gps_diag_requested = true;
            mutex_exit(&wspr_gps_mutex);
            request_diag = false;
        }

        if (tx.active) {
            if (!wspr_poll_transmitter(&clock, &tx)) {
                /* Stop scheduling after an I2C error: RF state is uncertain. */
                wspr_stop_on_error(&clock, "transmission failed");
            }
            if (!tx.active) {
                printf("WSPR RF STATE: NOT TRANSMITTING (frame complete)\n");
                refresh_gps = true;
            }
        } else {
            bool changed = false;
            if (mutex_try_enter(&wspr_gps_mutex, NULL)) {
                if (revision != wspr_gps_revision) {
                    gps = wspr_gps_shared;
                    revision = wspr_gps_revision;
                    changed = true;
                }
                mutex_exit(&wspr_gps_mutex);
            }
            if (changed || refresh_gps) {
                refresh_gps = false;
                char previous_grid[5];
                memcpy(previous_grid, schedule.grid, sizeof(previous_grid));
                bool had_gps_slot = schedule.gps_slot_selected;
                if (wspr_beacon_use_gps(&schedule, &gps, get_absolute_time())) {
                    if (memcmp(previous_grid, schedule.grid, sizeof(previous_grid)) != 0) {
                        printf("WSPR LOCATION: %s -> %s (GPS)\n", previous_grid, schedule.grid);
                    }
                    if (!had_gps_slot) {
                        printf("WSPR GPS synchronized: UTC=%02u:%02u:%02u grid=%s; next slot in %lld ms\n",
                               gps.utc_hour, gps.utc_minute, gps.utc_second, schedule.grid,
                               absolute_time_diff_us(get_absolute_time(), schedule.next_start) / 1000ll);
                    }
                }
            }
            absolute_time_t now = get_absolute_time();
            if (wspr_beacon_should_start(&schedule, false, trigger, now)) {
                const char *source = schedule.gps_mode ? "GPS sequence" : "serial schedule";
                const char *callsign;
                uint16_t payload;
                if (!wspr_beacon_message(&schedule, &gps, now, &callsign, &payload)) {
                    printf("WSPR skipped slot: waiting for fresh GPS UTC + location + altitude\n");
                    continue;
                }
                if (!wspr_prepare_payload(&clock, &tx, callsign, payload, WSPR_POWER_DBM)) {
                    wspr_stop_on_error(&clock, "preparation failed");
                }
                now = get_absolute_time();
                if (!wspr_start(&clock, &tx, now)) {
                    wspr_stop_on_error(&clock, "start failed");
                }
                last_callsign = callsign;
                last_payload = payload;
                printf("WSPR RF STATE: TRANSMITTING (%s): %s type=%u payload=%u %u dBm\n",
                       source, callsign, schedule.gps_mode ? schedule.phase : 0u,
                       (unsigned)payload, (unsigned)WSPR_POWER_DBM);
                wspr_beacon_did_start(&schedule, now);
            }
        }
        sleep_us(100u);
    }
}

#endif
