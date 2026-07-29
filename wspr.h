/* Standard WSPR (type 1) encoder and 10 m transmitter for an Si5351A.
 *
 * The transmitter uses CLK0.  Attach CLK0 to a correctly filtered 10 m RF
 * chain; a Si5351A square-wave output is not legal or suitable as an antenna
 * feed by itself.  A calibrated reference (preferably a GPSDO/TCXO) is needed
 * for reliable weak-signal reception.
 */
#ifndef WSPR_H
#define WSPR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "c90770_uart.h"
#include "pico/stdlib.h"
#include "si5351a_i2c.h"

#define WSPR_SYMBOL_COUNT 162u
#define WSPR_SYMBOL_PERIOD_NUMERATOR_US 8192000000ull
#define WSPR_SYMBOL_PERIOD_DENOMINATOR 12000u
#define WSPR_10M_DIAL_HZ 28126100u
#define WSPR_TONE_DENOMINATOR 256u
#define WSPR_TONE_STEP_NUMERATOR 375u

/* These defaults are syntactically valid WSPR type-1 fields, not station
 * identities.  Set WSPR_CALLSIGN and WSPR_POWER_DBM for the licensed station. */
#ifndef WSPR_CALLSIGN
#define WSPR_CALLSIGN "K1ABC"
#endif
#ifndef WSPR_FALLBACK_GRID
#define WSPR_FALLBACK_GRID "AA00"
#endif
#ifndef WSPR_POWER_DBM
#define WSPR_POWER_DBM 10u
#endif

static inline uint8_t wspr_parity32(uint32_t value) {
    value ^= value >> 16;
    value ^= value >> 8;
    value ^= value >> 4;
    value &= 0x0fu;
    return (uint8_t)((0x6996u >> value) & 1u);
}

static inline uint8_t wspr_reverse8(uint8_t value) {
    value = (uint8_t)((value >> 4) | (value << 4));
    value = (uint8_t)(((value & 0xccu) >> 2) | ((value & 0x33u) << 2));
    return (uint8_t)(((value & 0xaau) >> 1) | ((value & 0x55u) << 1));
}

static inline bool wspr_valid_power(uint8_t dbm) {
    uint8_t remainder = (uint8_t)(dbm % 10u);
    return dbm <= 60u && (remainder == 0u || remainder == 3u || remainder == 7u);
}

static inline char wspr_upper(char c) {
    return c >= 'a' && c <= 'z' ? (char)(c - ('a' - 'A')) : c;
}

/* Convert a normal callsign such as K1ABC to WSPR's fixed six characters.
 * The digit must occupy position three after optional leading-space padding. */
static inline bool wspr_normalize_callsign(const char *callsign, char result[6]) {
    size_t length = strlen(callsign);
    if (length == 0u || length > 6u) {
        return false;
    }

    bool prepend_space = length >= 2u && callsign[1] >= '0' && callsign[1] <= '9';
    if (prepend_space && length == 6u) {
        return false;
    }
    size_t offset = prepend_space ? 1u : 0u;
    for (size_t i = 0u; i < 6u; ++i) {
        result[i] = ' ';
    }
    for (size_t i = 0u; i < length; ++i) {
        char c = wspr_upper(callsign[i]);
        if (!((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9'))) {
            return false;
        }
        result[offset + i] = c;
    }
    if (result[2] < '0' || result[2] > '9') {
        return false;
    }
    for (size_t i = 3u; i < 6u; ++i) {
        if (!((result[i] >= 'A' && result[i] <= 'Z') || result[i] == ' ')) {
            return false;
        }
    }
    return true;
}

static inline uint8_t wspr_call_first_code(char c) {
    if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
    if (c >= 'A' && c <= 'Z') return (uint8_t)(c - 'A' + 10);
    return 36u;
}

static inline bool wspr_pack_message(
    const char *callsign,
    const char grid[4],
    uint8_t power_dbm,
    uint32_t *n1,
    uint32_t *n2
) {
    char call[6];
    if (!wspr_normalize_callsign(callsign, call) || !wspr_valid_power(power_dbm)) {
        return false;
    }
    char g0 = wspr_upper(grid[0]);
    char g1 = wspr_upper(grid[1]);
    char g2 = grid[2];
    char g3 = grid[3];
    if (g0 < 'A' || g0 > 'R' || g1 < 'A' || g1 > 'R' ||
        g2 < '0' || g2 > '9' || g3 < '0' || g3 > '9') {
        return false;
    }

    uint32_t packed_call = wspr_call_first_code(call[0]);
    packed_call = 36u * packed_call + wspr_call_first_code(call[1]);
    packed_call = 10u * packed_call + (uint32_t)(call[2] - '0');
    packed_call = 27u * packed_call + (uint32_t)(call[3] == ' ' ? 26 : call[3] - 'A');
    packed_call = 27u * packed_call + (uint32_t)(call[4] == ' ' ? 26 : call[4] - 'A');
    packed_call = 27u * packed_call + (uint32_t)(call[5] == ' ' ? 26 : call[5] - 'A');

    uint32_t packed_grid = (uint32_t)(179 - 10 * (g0 - 'A') - (g2 - '0')) * 180u;
    packed_grid += (uint32_t)(10 * (g1 - 'A') + (g3 - '0'));
    *n1 = packed_call;
    *n2 = (packed_grid << 7) | 0x40u | power_dbm;
    return true;
}

static inline bool wspr_encode(
    const char *callsign,
    const char grid[4],
    uint8_t power_dbm,
    uint8_t symbols[WSPR_SYMBOL_COUNT]
) {
    static const uint8_t sync[WSPR_SYMBOL_COUNT] = {
        1,1,0,0,0,0,0,0,1,0,0,0,1,1,1,0,0,0,1,0,0,1,
        0,1,1,1,1,0,0,0,0,0,0,0,1,0,0,1,0,1,0,0,0,0,
        0,0,1,0,1,1,0,0,1,1,0,1,0,0,0,1,1,0,1,0,0,0,
        0,1,1,0,1,0,1,0,1,0,1,0,0,1,0,0,1,0,1,1,0,0,
        0,1,1,0,1,0,1,0,0,0,1,0,0,0,0,0,1,0,0,1,0,0,
        1,1,1,0,1,1,0,0,1,1,0,1,0,0,0,1,1,1,0,0,0,0,
        0,1,0,1,0,0,1,1,0,0,0,0,0,0,0,1,1,0,1,0,1,1,
        0,0,0,1,1,0,0,0
    };
    uint32_t n1 = 0u;
    uint32_t n2 = 0u;
    uint8_t convolutional[WSPR_SYMBOL_COUNT];
    if (!wspr_pack_message(callsign, grid, power_dbm, &n1, &n2)) {
        return false;
    }

    uint32_t register_value = 0u;
    for (uint8_t bit_index = 0u; bit_index < 81u; ++bit_index) {
        uint8_t bit = 0u;
        if (bit_index < 28u) {
            bit = (uint8_t)((n1 >> (27u - bit_index)) & 1u);
        } else if (bit_index < 50u) {
            bit = (uint8_t)((n2 >> (49u - bit_index)) & 1u);
        }
        register_value = (register_value << 1) | bit;
        convolutional[2u * bit_index] = wspr_parity32(register_value & 0xf2d05351u);
        convolutional[2u * bit_index + 1u] = wspr_parity32(register_value & 0xe4613c47u);
    }
    uint8_t interleaved[WSPR_SYMBOL_COUNT] = {0};
    uint8_t source = 0u;
    for (uint16_t i = 0u; i < 256u && source < WSPR_SYMBOL_COUNT; ++i) {
        uint8_t destination = wspr_reverse8((uint8_t)i);
        if (destination < WSPR_SYMBOL_COUNT) {
            interleaved[destination] = convolutional[source++];
        }
    }
    for (uint8_t i = 0u; i < WSPR_SYMBOL_COUNT; ++i) {
        symbols[i] = (uint8_t)(sync[i] + 2u * interleaved[i]);
    }
    return true;
}

static inline bool wspr_grid_from_coordinates(double latitude, double longitude, char grid[5]) {
    if (latitude < -90.0 || latitude >= 90.0 || longitude < -180.0 || longitude >= 180.0) {
        return false;
    }
    double lon = longitude + 180.0;
    double lat = latitude + 90.0;
    uint8_t field_lon = (uint8_t)(lon / 20.0);
    uint8_t field_lat = (uint8_t)(lat / 10.0);
    uint8_t square_lon = (uint8_t)((lon - 20.0 * field_lon) / 2.0);
    uint8_t square_lat = (uint8_t)(lat - 10.0 * field_lat);
    if (field_lon > 17u || field_lat > 17u || square_lon > 9u || square_lat > 9u) {
        return false;
    }
    grid[0] = (char)('A' + field_lon);
    grid[1] = (char)('A' + field_lat);
    grid[2] = (char)('0' + square_lon);
    grid[3] = (char)('0' + square_lat);
    grid[4] = '\0';
    return true;
}

/* This is deliberately blocking: preserving every symbol deadline is more
 * important than handling unrelated work during a 110.592 second WSPR frame. */
static inline bool wspr_transmit_10m_at(
    si5351a_i2c_t *clock,
    const char *callsign,
    const char grid[4],
    uint8_t power_dbm,
    absolute_time_t start
) {
    uint8_t symbols[WSPR_SYMBOL_COUNT];
    if (!wspr_encode(callsign, grid, power_dbm, symbols)) {
        return false;
    }

    /* 28.126100 MHz is the conventional 10 m WSPR transmit centre. */
    uint64_t initial_frequency = (uint64_t)WSPR_10M_DIAL_HZ * WSPR_TONE_DENOMINATOR +
                                 (uint64_t)WSPR_TONE_STEP_NUMERATOR * symbols[0];
    uint32_t current_p1 = 0u;
    uint32_t initial_p2 = 0u;
    uint32_t current_p3 = 0u;
    if (!si5351a_i2c_multisynth_from_frequency_ratio(
            initial_frequency, WSPR_TONE_DENOMINATOR, &current_p1, &initial_p2, &current_p3) ||
        !si5351a_i2c_write_multisynth(clock, 42u, current_p1, initial_p2, current_p3, 0u, false)) {
        return false;
    }

    /* Do not radiate the first tone while waiting for the UTC slot. */
    if (!si5351a_i2c_set_clk0_enabled(clock, false)) {
        return false;
    }
    sleep_until(delayed_by_us(start, -10000));
    if (!si5351a_i2c_set_clk0_enabled(clock, true)) {
        return false;
    }

    for (uint32_t i = 0u; i < WSPR_SYMBOL_COUNT; ++i) {
        uint64_t offset_us = ((uint64_t)i * WSPR_SYMBOL_PERIOD_NUMERATOR_US) /
                             WSPR_SYMBOL_PERIOD_DENOMINATOR;
        sleep_until(delayed_by_us(start, (int64_t)offset_us));
        if (i + 1u < WSPR_SYMBOL_COUNT) {
            uint64_t frequency = (uint64_t)WSPR_10M_DIAL_HZ * WSPR_TONE_DENOMINATOR +
                                 (uint64_t)WSPR_TONE_STEP_NUMERATOR * symbols[i + 1u];
            uint32_t next_p1 = 0u;
            uint32_t next_p2 = 0u;
            uint32_t next_p3 = 0u;
            if (!si5351a_i2c_multisynth_from_frequency_ratio(
                    frequency, WSPR_TONE_DENOMINATOR, &next_p1, &next_p2, &next_p3)) {
                return false;
            }
            if (next_p1 == current_p1 && next_p3 == current_p3) {
                /* Only P2 changes for the four 10 m WSPR tones.  Updating
                 * registers 47..49 takes four I2C bytes rather than nine. */
                uint8_t p2_registers[3] = {
                    (uint8_t)((((next_p3 >> 16) & 0x0fu) << 4) | ((next_p2 >> 16) & 0x0fu)),
                    (uint8_t)((next_p2 >> 8) & 0xffu),
                    (uint8_t)(next_p2 & 0xffu),
                };
                if (!si5351a_i2c_write_regs(clock, 47u, p2_registers, sizeof(p2_registers))) {
                    return false;
                }
            } else if (!si5351a_i2c_write_multisynth(clock, 42u, next_p1, next_p2, next_p3, 0u, false)) {
                return false;
            }
            current_p1 = next_p1;
            current_p3 = next_p3;
        }
    }
    uint64_t frame_us = ((uint64_t)WSPR_SYMBOL_COUNT * WSPR_SYMBOL_PERIOD_NUMERATOR_US) /
                        WSPR_SYMBOL_PERIOD_DENOMINATOR;
    sleep_until(delayed_by_us(start, (int64_t)frame_us));
    return si5351a_i2c_set_clk0_enabled(clock, false);
}

/* For test transmissions only.  Normal WSPR operation must call the _at()
 * form with a UTC-slot start (one second into an even UTC minute). */
static inline bool wspr_transmit_10m(
    si5351a_i2c_t *clock,
    const char *callsign,
    const char grid[4],
    uint8_t power_dbm
) {
    return wspr_transmit_10m_at(
        clock, callsign, grid, power_dbm, make_timeout_time_us(20000u));
}

static inline absolute_time_t wspr_next_utc_slot(const c90770_gps_monitor_state_t *gps) {
    /* WSPR starts at hh:mm:01 for every even UTC minute. */
    uint32_t now = 3600u * gps->utc_hour + 60u * gps->utc_minute + gps->utc_second;
    uint32_t slot = (now / 120u) * 120u + 1u;
    if (slot <= now) {
        slot += 120u;
    }
    return delayed_by_us(gps->utc_captured_at, (int64_t)(slot - now) * 1000000ll);
}

/* This loop owns the application while WSPR is enabled.  It is intentionally
 * called before unrelated camera, sweep, or LED work that could disturb a
 * frame's symbol timing. */
static inline void wspr_run_10m_beacon(void) {
    si5351a_i2c_t clock;
    if (!si5351a_i2c_start_output_hz(&clock, WSPR_10M_DIAL_HZ) ||
        !si5351a_i2c_set_clk0_enabled(&clock, false)) {
        printf("Si5351A init failed: %s (reg 0x%02x)\n",
               si5351a_i2c_error_string(clock.last_error), clock.last_reg);
        while (true) {
            sleep_ms(1000u);
        }
    }

    c90770_uart_t uart;
    c90770_uart_init_default(&uart);
    c90770_gps_monitor_state_t gps = {0};
    char line[128];
    printf("WSPR 10m: waiting for valid GPS UTC; callsign=%s power=%u dBm\n",
           WSPR_CALLSIGN, (unsigned)WSPR_POWER_DBM);

    while (true) {
        size_t length = c90770_uart_read_line_timeout(&uart, line, sizeof(line), 100000u);
        if (length == 0u) {
            continue;
        }
        c90770_parse_gps_line(&gps, line);
        if (!gps.have_utc_time) {
            continue;
        }

        char grid[5] = WSPR_FALLBACK_GRID;
        if (gps.have_coordinates &&
            !wspr_grid_from_coordinates(gps.latitude_degrees, gps.longitude_degrees, grid)) {
            memcpy(grid, WSPR_FALLBACK_GRID, sizeof(grid));
        }

        absolute_time_t start = wspr_next_utc_slot(&gps);
        if (absolute_time_diff_us(get_absolute_time(), start) < 20000ll) {
            start = delayed_by_us(start, 120000000ll);
        }
        printf("WSPR TX %s %s %u dBm\n", WSPR_CALLSIGN, grid, (unsigned)WSPR_POWER_DBM);
        if (!wspr_transmit_10m_at(&clock, WSPR_CALLSIGN, grid, WSPR_POWER_DBM, start)) {
            printf("WSPR transmit failed: %s (reg 0x%02x)\n",
                   si5351a_i2c_error_string(clock.last_error), clock.last_reg);
        }
    }
}

#endif
