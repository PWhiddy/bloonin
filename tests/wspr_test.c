/* Host regression checks for the real encoder, parser, scheduler and I2C
 * writes. Run using the command in README.md; no Pico hardware is required. */
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
static unsigned log_lines;
static char last_log[512];
static int test_printf(const char *format, ...) {
    va_list args;
    va_start(args, format);
    int result = vsnprintf(last_log, sizeof(last_log), format, args);
    va_end(args);
    ++log_lines;
    return result;
}
#define printf test_printf
#include "wspr_beacon.h"
#undef printf

static void parse(c90770_gps_monitor_state_t *gps, const char *payload, absolute_time_t captured) {
    char line[128];
    uint8_t checksum = 0;
    for (const char *p = payload; *p; ++p) checksum ^= (uint8_t)*p;
    snprintf(line, sizeof(line), "$%s*%02X\r\n", payload, checksum);
    c90770_parse_gps_line_at(gps, line, captured);
}

static void test_stream(void) {
    c90770_nmea_stream_t stream = {0};
    const char *chunks[] = {"noise$GPR", "MC,123", "*00\r", "\n"};
    unsigned complete = 0;
    for (unsigned i = 0; i < 4; ++i) {
        for (const char *p = chunks[i]; *p; ++p) {
            complete += c90770_nmea_stream_push(&stream, *p, 1000 + i);
        }
    }
    assert(complete == 1);
    assert(strcmp(stream.line, "$GPRMC,123*00\r\n") == 0);
    assert(stream.started_at == 1000);
    c90770_nmea_stream_push(&stream, '$', 2000);
    for (unsigned i = 0; i < 200; ++i) assert(!c90770_nmea_stream_push(&stream, 'x', 2001));
    assert(!c90770_nmea_stream_push(&stream, '\n', 2002));
    assert(!c90770_nmea_stream_push(&stream, '$', 3000));
    assert(c90770_nmea_stream_push(&stream, '\n', 3001));
    assert(stream.started_at == 3000);
}

static void test_gps_and_schedule(void) {
    c90770_gps_monitor_state_t gps = {0};
    wspr_beacon_schedule_t schedule = {.grid = WSPR_FALLBACK_GRID};
    assert(!wspr_beacon_should_start(&schedule, false, false, 1000000000));
    assert(wspr_beacon_should_start(&schedule, false, true, 1000));
    wspr_beacon_did_start(&schedule, 1000);
    assert(schedule.next_start == 120001000);
    assert(!wspr_beacon_should_start(&schedule, true, true, 1000));
    assert(!wspr_beacon_use_gps(&schedule, &gps, 1000));
    assert(strcmp(schedule.grid, WSPR_FALLBACK_GRID) == 0);

    parse(&gps, "GPRMC,120000.250,A,4807.038,N,01131.000,E,0,0,050926,,,A", 10000000);
    assert(gps.have_utc_time && gps.have_coordinates);
    assert(gps.utc_microsecond == 250000);
    assert(gps.utc_captured_at == 10000000);
    assert(wspr_beacon_use_gps(&schedule, &gps, 10100000));
    assert(strcmp(schedule.grid, "JN58") == 0);
    assert(schedule.next_start == 10750000);
    assert(!wspr_beacon_should_start(&schedule, false, false, 10749999));
    assert(wspr_beacon_should_start(&schedule, false, false, 10750000));
    assert(wspr_beacon_should_start(&schedule, false, true, 10750000));

    /* Subsequent NMEA sentences must not keep moving a selected slot. */
    parse(&gps, "GPRMC,120001.000,A,4807.038,N,01131.000,E,0,0,050926,,,A", 10750000);
    assert(wspr_beacon_use_gps(&schedule, &gps, 10750001));
    assert(schedule.next_start == 10750000);
    wspr_beacon_did_start(&schedule, 10750000);
    assert(!wspr_beacon_should_start(&schedule, true, true, 10750001));
    assert(schedule.next_start == 130750000);

    /* Fix loss cannot replace the last usable grid or stop clock holdover. */
    parse(&gps, "GPRMC,120002.0,V,,,,,,,050926,,,N", 11750000);
    assert(!gps.have_utc_time && !gps.have_coordinates);
    assert(!wspr_beacon_use_gps(&schedule, &gps, 11750000));
    assert(strcmp(schedule.grid, "JN58") == 0);
    assert(schedule.armed && schedule.next_start == 130750000);

    parse(&gps, "GPRMC,120003,A,4807.038,N,01131.000,E,0,0,050926,,,A", 12750000);
    assert(!wspr_beacon_use_gps(&schedule, &gps, 17750000)); // stale
    parse(&gps, "GPGGA,120004,,,,,0,00,99.9,,,,,,", 13750000);
    assert(!gps.have_coordinates);
    parse(&gps, "GPRMC,120005,A,nan,N,01131.000,E,0,0,050926,,,A", 14750000);
    assert(!gps.have_coordinates);
    parse(&gps, "GPRMC,120005,A,4807.038,,01131.000,E,0,0,050926,,,A", 14750000);
    assert(!gps.have_coordinates);
    parse(&gps, "GPRMC,120005.bad,A,4807.038,N,01131.000,E,0,0,050926,,,A", 14750000);
    assert(!gps.have_utc_time);

    parse(&gps, "GPRMC,235959.500,A,4807.038,N,01131.000,E,0,0,050926,,,A", 20000000);
    assert(wspr_next_utc_slot_at(&gps, 20000000) == 21500000);
    assert(wspr_next_utc_slot_at(&gps, 21500000) == 141500000);
    assert(wspr_next_utc_slot_at(&gps, 22000000) == 141500000);
    assert(wspr_next_utc_slot_at(&gps, 262000000) == 381500000);
    char invalid[] = "$GPRMC,120000,A*00\n";
    c90770_parse_gps_line(&gps, invalid);
    assert(gps.checksum_failures == 1);
}

static void test_transmitter(void) {
    si5351a_i2c_t clock = {0};
    test_now = 0;
    test_rf_enabled_during_init = false;
    assert(si5351a_i2c_configure_output_hz_enabled(&clock, WSPR_BASE_HZ, false));
    assert(!test_rf_enabled_during_init);
    wspr_transmitter_t tx = {0};
    assert(wspr_prepare(&clock, &tx, "K1ABC", "FN30", 10));
    assert(test_registers[3] & 1u);
    assert(WSPR_BASE_HZ == 14097100u);
    assert(wspr_start(&clock, &tx, 1000000));
    assert(!(test_registers[3] & 1u));
    unsigned writes = test_writes;
    test_now = 1682665;
    assert(wspr_poll_transmitter(&clock, &tx));
    assert(tx.symbol == 0 && test_writes == writes);
    test_now = 1682666;
    assert(wspr_poll_transmitter(&clock, &tx));
    assert(tx.symbol == 1 && test_writes == writes + 1);
    for (unsigned symbol = 2; symbol < 162; ++symbol) {
        test_now = 1000000 + wspr_symbol_offset_us(symbol);
        assert(wspr_poll_transmitter(&clock, &tx));
        assert(tx.symbol == symbol && tx.active);
        /* Decode the actual I2C divider to verify the current symbol's tone. */
        uint32_t p3 = ((test_registers[47] & 0xf0u) << 12) | (test_registers[42] << 8) | test_registers[43];
        uint32_t p1 = ((test_registers[44] & 3u) << 16) | (test_registers[45] << 8) | test_registers[46];
        uint32_t p2 = ((test_registers[47] & 15u) << 16) | (test_registers[48] << 8) | test_registers[49];
        double divider = (p1 + 512.0 + (double)p2 / p3) / 128.0;
        double hz = SI5351A_PLL_HZ / divider;
        double expected = 14097100.0 + tx.symbols[symbol] * 375.0 / 256.0;
        /* The existing 20-bit divider rounding permits about 0.11 Hz error. */
        assert(fabs(hz - expected) < 0.12);
    }
    test_now = 111591999;
    assert(wspr_poll_transmitter(&clock, &tx) && tx.active);
    test_now = 111592000;
    assert(wspr_poll_transmitter(&clock, &tx) && !tx.active);
    assert(test_registers[3] & 1u);

    assert(wspr_prepare(&clock, &tx, "K1ABC", "FN30", 10));
    assert(wspr_start(&clock, &tx, test_now));
    test_now += 682666;
    test_fail_write = true;
    assert(!wspr_poll_transmitter(&clock, &tx));
    assert(!tx.active && (test_registers[3] & 1u));
    assert(!wspr_prepare(&clock, &tx, "BAD", "FN30", 10));
    assert(test_registers[3] & 1u);
}

static void test_transition_logs(void) {
    c90770_gps_monitor_state_t gps = {.quiet = true};
    wspr_gps_log_state_t log = {0};
    log_lines = 0;
    wspr_log_gps_status(&log, &gps, false, 0, 0);
    assert(log_lines == 1 && strstr(last_log, "waiting for data"));
    wspr_log_gps_status(&log, &gps, false, 0, 10000000);
    assert(log_lines == 1);
    wspr_log_gps_status(&log, &gps, true, 10000000, 10000000);
    assert(log_lines == 2 && strstr(last_log, "UART=receiving"));
    parse(&gps, "GPRMC,120000.0,A,4807.038,N,01131.000,E,0,0,050926,,,A", 10000000);
    assert(log_lines == 2); // Quiet parser leaves all logging to the monitor.
    wspr_log_gps_status(&log, &gps, true, 10000000, 10000000);
    assert(log_lines == 3 && strstr(last_log, "fix=valid") && strstr(last_log, "UTC=valid"));
    wspr_log_gps_status(&log, &gps, true, 10000000, 10000100);
    assert(log_lines == 3);
    parse(&gps, "GPGSV,1,1,01,01,45,180,30", 10001000);
    assert(log_lines == 3);
    wspr_log_gps_status(&log, &gps, true, 10001000, 10001000);
    assert(log_lines == 4 && strstr(last_log, "in_view=1 tracked=1"));
    gps.satellites[0].snr = 0;
    wspr_log_gps_status(&log, &gps, true, 10002000, 10002000);
    assert(log_lines == 4); // Progress rate limit.
    gps.utc_captured_at = 20001000;
    wspr_log_gps_status(&log, &gps, true, 20001000, 20001000);
    assert(log_lines == 5 && strstr(last_log, "tracked=0"));
    gps.utc_captured_at = 30001000;
    wspr_log_gps_status(&log, &gps, true, 30001000, 30001000);
    assert(log_lines == 5); // No periodic unchanged summary.
    wspr_log_gps_status(&log, &gps, true, 30001000, 35001000);
    assert(log_lines == 6 && strstr(last_log, "silent") && strstr(last_log, "UTC=unavailable"));
    wspr_log_gps_status(&log, &gps, true, 30001000, 36001000);
    assert(log_lines == 6);
}

int main(void) {
    test_stream();
    test_gps_and_schedule();
    test_transmitter();
    test_transition_logs();
    puts("WSPR regression checks passed");
    return 0;
}
