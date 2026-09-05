/* Standard WSPR (type 1) encoder and transmitter for an Si5351A.
 *
 * The transmitter defaults to 20 m on CLK0.  Attach CLK0 to a correctly filtered RF
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
/* Actual RF tone-zero frequency, not the receiver's USB dial frequency. */
#ifndef WSPR_BASE_HZ
#define WSPR_BASE_HZ 14097100u
#endif
#define WSPR_SLOT_PERIOD_US 120000000ll
#define WSPR_TONE_DENOMINATOR 256u
#define WSPR_TONE_STEP_NUMERATOR 375u

/* These defaults are syntactically valid WSPR type-1 fields, not station
 * identities.  Set WSPR_CALLSIGN and WSPR_POWER_DBM for the licensed station. */
#ifndef WSPR_CALLSIGN
#define WSPR_CALLSIGN "K1ABC"
#endif
#ifndef WSPR_FALLBACK_GRID
#define WSPR_FALLBACK_GRID "FN30"
// "AA00"
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
    if (!isfinite(latitude) || !isfinite(longitude) ||
        latitude < -90.0 || latitude >= 90.0 || longitude < -180.0 || longitude >= 180.0) {
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

typedef struct {
    uint32_t p1, p2, p3;
} wspr_tone_t;

typedef struct {
    uint8_t symbols[WSPR_SYMBOL_COUNT];
    wspr_tone_t tones[4];
    uint32_t symbol;
    absolute_time_t start;
    bool active;
} wspr_transmitter_t;

static inline uint64_t wspr_symbol_offset_us(uint32_t symbol) {
    return ((uint64_t)symbol * WSPR_SYMBOL_PERIOD_NUMERATOR_US) /
           WSPR_SYMBOL_PERIOD_DENOMINATOR;
}

/* Prepare while RF is off so a trigger only needs the output-enable write. */
static inline bool wspr_prepare(
    si5351a_i2c_t *clock, wspr_transmitter_t *tx,
    const char *callsign, const char grid[4], uint8_t power_dbm
) {
    tx->active = false;
    if (!si5351a_i2c_set_clk0_enabled(clock, false) ||
        !wspr_encode(callsign, grid, power_dbm, tx->symbols)) {
        return false;
    }
    for (uint32_t i = 0; i < 4u; ++i) {
        uint64_t frequency = (uint64_t)WSPR_BASE_HZ * WSPR_TONE_DENOMINATOR +
                             (uint64_t)WSPR_TONE_STEP_NUMERATOR * i;
        wspr_tone_t *tone = &tx->tones[i];
        if (!si5351a_i2c_multisynth_from_frequency_ratio(
                frequency, WSPR_TONE_DENOMINATOR, &tone->p1, &tone->p2, &tone->p3)) {
            return false;
        }
    }
    const wspr_tone_t *tone = &tx->tones[tx->symbols[0]];
    return si5351a_i2c_write_multisynth(
        clock, 42u, tone->p1, tone->p2, tone->p3, 0u, false);
}

static inline bool wspr_start(
    si5351a_i2c_t *clock, wspr_transmitter_t *tx, absolute_time_t start
) {
    if (!si5351a_i2c_set_clk0_enabled(clock, true)) {
        return false;
    }
    tx->start = start;
    tx->symbol = 0u;
    tx->active = true;
    return true;
}

static inline absolute_time_t wspr_symbol_deadline(const wspr_transmitter_t *tx) {
    return delayed_by_us(tx->start, (int64_t)wspr_symbol_offset_us(tx->symbol + 1u));
}

/* Service one symbol deadline. GPS and serial reception can run between calls.
 * Symbol zero stays on air until boundary one; symbol 161 ends at 110.592 s. */
static inline bool wspr_poll_transmitter(si5351a_i2c_t *clock, wspr_transmitter_t *tx) {
    if (!tx->active || !time_reached(wspr_symbol_deadline(tx))) {
        return true;
    }
    if (tx->symbol + 1u == WSPR_SYMBOL_COUNT) {
        tx->active = false;
        return si5351a_i2c_set_clk0_enabled(clock, false);
    }
    const wspr_tone_t *previous = &tx->tones[tx->symbols[tx->symbol]];
    const wspr_tone_t *next = &tx->tones[tx->symbols[++tx->symbol]];
    bool ok;
    if (previous->p1 == next->p1 && previous->p3 == next->p3) {
        /* When only P2 changes, three registers suffice on any band. */
        uint8_t registers[3] = {
            (uint8_t)((((next->p3 >> 16) & 0x0fu) << 4) | ((next->p2 >> 16) & 0x0fu)),
            (uint8_t)(next->p2 >> 8),
            (uint8_t)next->p2,
        };
        ok = si5351a_i2c_write_regs(clock, 47u, registers, sizeof(registers));
    } else {
        ok = si5351a_i2c_write_multisynth(
            clock, 42u, next->p1, next->p2, next->p3, 0u, false);
    }
    if (!ok) {
        tx->active = false;
        si5351a_i2c_set_clk0_enabled(clock, false);
    }
    return ok;
}

/* Blocking convenience API; the beacon uses the polling API above. */
static inline bool wspr_transmit_at(
    si5351a_i2c_t *clock, const char *callsign, const char grid[4],
    uint8_t power_dbm, absolute_time_t start
) {
    wspr_transmitter_t tx = {0};
    if (!wspr_prepare(clock, &tx, callsign, grid, power_dbm)) {
        return false;
    }
    sleep_until(start);
    if (!wspr_start(clock, &tx, start)) {
        return false;
    }
    while (tx.active) {
        sleep_until(wspr_symbol_deadline(&tx));
        if (!wspr_poll_transmitter(clock, &tx)) {
            return false;
        }
    }
    return true;
}

static inline bool wspr_transmit(
    si5351a_i2c_t *clock, const char *callsign, const char grid[4], uint8_t power_dbm
) {
    return wspr_transmit_at(clock, callsign, grid, power_dbm, make_timeout_time_us(20000u));
}

static inline absolute_time_t wspr_next_utc_slot_at(
    const c90770_gps_monitor_state_t *gps, absolute_time_t now
) {
    /* Include elapsed monotonic time and fractional NMEA seconds, so an old
     * reading still selects a future hh:mm:01 slot, including across midnight. */
    int64_t utc_us = (3600ll * gps->utc_hour + 60ll * gps->utc_minute +
                      gps->utc_second) * 1000000ll + gps->utc_microsecond;
    utc_us += absolute_time_diff_us(gps->utc_captured_at, now);
    int64_t slot_us = (utc_us / WSPR_SLOT_PERIOD_US) * WSPR_SLOT_PERIOD_US + 1000000ll;
    if (slot_us <= utc_us) {
        slot_us += WSPR_SLOT_PERIOD_US;
    }
    return delayed_by_us(now, slot_us - utc_us);
}

static inline absolute_time_t wspr_next_utc_slot(const c90770_gps_monitor_state_t *gps) {
    return wspr_next_utc_slot_at(gps, get_absolute_time());
}

#endif
