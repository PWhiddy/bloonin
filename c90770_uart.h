#ifndef C90770_UART_H
#define C90770_UART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"


#ifndef C90770_UART_INSTANCE
#define C90770_UART_INSTANCE uart1
#endif

#ifndef C90770_UART_RX_GPIO
#define C90770_UART_RX_GPIO 9u
#endif

#ifndef C90770_UART_BAUD
#define C90770_UART_BAUD 9600u
#endif

typedef struct {
    uart_inst_t *uart;
    uint rx_gpio;
    uint32_t baudrate;
} c90770_uart_t;

static inline void c90770_uart_init(
    c90770_uart_t *uart,
    uart_inst_t *uart_instance,
    uint rx_gpio,
    uint32_t baudrate
) {
    uart->uart = uart_instance;
    uart->rx_gpio = rx_gpio;
    uart->baudrate = baudrate;

    uart_init(uart_instance, baudrate);
    gpio_set_function(rx_gpio, GPIO_FUNC_UART);
    gpio_pull_up(rx_gpio);
    uart_set_format(uart_instance, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart_instance, true);
}

static inline void c90770_uart_init_default(c90770_uart_t *uart) {
    c90770_uart_init(uart, C90770_UART_INSTANCE, C90770_UART_RX_GPIO, C90770_UART_BAUD);
}

static inline bool c90770_uart_read_byte_timeout(
    const c90770_uart_t *uart,
    uint8_t *byte,
    uint32_t timeout_us
) {
    if (!uart_is_readable_within_us(uart->uart, timeout_us)) {
        return false;
    }

    *byte = uart_getc(uart->uart);
    return true;
}

static inline int c90770_uart_read_byte_blocking(const c90770_uart_t *uart) {
    return uart_getc(uart->uart);
}

static inline size_t c90770_uart_read_line_timeout(
    const c90770_uart_t *uart,
    char *buffer,
    size_t buffer_len,
    uint32_t per_byte_timeout_us
) {
    if (buffer_len == 0u) {
        return 0u;
    }

    size_t len = 0;
    while (len + 1u < buffer_len) {
        uint8_t byte = 0;
        if (!c90770_uart_read_byte_timeout(uart, &byte, per_byte_timeout_us)) {
            break;
        }

        buffer[len++] = (char)byte;
        if (byte == '\n') {
            break;
        }
    }

    buffer[len] = '\0';
    return len;
}

#ifndef C90770_GPS_MAX_FIELDS
#define C90770_GPS_MAX_FIELDS 32u
#endif

#ifndef C90770_GPS_MAX_CONSTELLATIONS
#define C90770_GPS_MAX_CONSTELLATIONS 8u
#endif

#ifndef C90770_GPS_MAX_SATELLITES
#define C90770_GPS_MAX_SATELLITES 64u
#endif

typedef struct {
    char talker[3];
    int prn;
    int elevation;
    int azimuth;
    int snr;
    bool has_snr;
} c90770_gps_satellite_t;

typedef struct {
    char talker[3];
    int total_messages;
    int satellites_in_view;
    bool active;
} c90770_gps_gsv_state_t;

typedef struct {
    c90770_gps_satellite_t satellites[C90770_GPS_MAX_SATELLITES];
    size_t satellite_count;
    c90770_gps_gsv_state_t gsv[C90770_GPS_MAX_CONSTELLATIONS];

    bool have_coordinates;
    double latitude_degrees;
    double longitude_degrees;
    double altitude_meters;
    bool have_altitude;
    int fix_quality;
    int satellites_used;
    char fix_source[4];

    uint32_t last_satellite_hash;
    uint32_t last_fix_hash;
    uint32_t checksum_failures;
} c90770_gps_monitor_state_t;

static inline void c90770_trim_line_end(char *line) {
    size_t len = strlen(line);
    while (len > 0u && (line[len - 1u] == '\r' || line[len - 1u] == '\n')) {
        line[--len] = '\0';
    }
}

static inline int c90770_hex_value(char c) {
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'A' && c <= 'F') {
        return 10 + c - 'A';
    }
    if (c >= 'a' && c <= 'f') {
        return 10 + c - 'a';
    }
    return -1;
}

static inline bool c90770_nmea_verify_checksum(char *line) {
    c90770_trim_line_end(line);

    if (line[0] != '$') {
        return false;
    }

    char *star = strchr(line, '*');
    if (star == NULL || star[1] == '\0' || star[2] == '\0') {
        return false;
    }

    uint8_t checksum = 0;
    for (char *p = line + 1; p < star; ++p) {
        checksum ^= (uint8_t)*p;
    }

    int expected_high = c90770_hex_value(star[1]);
    int expected_low = c90770_hex_value(star[2]);
    if (expected_high < 0 || expected_low < 0) {
        return false;
    }

    *star = '\0';
    return checksum == (uint8_t)((expected_high << 4) | expected_low);
}

static inline size_t c90770_split_nmea_fields(char *payload, char **fields, size_t field_count) {
    size_t count = 0;
    fields[count++] = payload;

    for (char *p = payload; *p != '\0' && count < field_count; ++p) {
        if (*p == ',') {
            *p = '\0';
            fields[count++] = p + 1;
        }
    }

    return count;
}

static inline int c90770_parse_int_field(const char *field, int fallback) {
    if (field == NULL || field[0] == '\0') {
        return fallback;
    }
    return (int)strtol(field, NULL, 10);
}

static inline double c90770_parse_double_field(const char *field, double fallback) {
    if (field == NULL || field[0] == '\0') {
        return fallback;
    }
    return strtod(field, NULL);
}

static inline double c90770_nmea_coordinate_to_degrees(const char *value, const char *hemisphere) {
    if (value == NULL || value[0] == '\0' || hemisphere == NULL || hemisphere[0] == '\0') {
        return 0.0;
    }

    double raw = strtod(value, NULL);
    int degrees = (int)(raw / 100.0);
    double minutes = raw - ((double)degrees * 100.0);
    double coordinate = (double)degrees + (minutes / 60.0);

    if (hemisphere[0] == 'S' || hemisphere[0] == 'W') {
        coordinate = -coordinate;
    }

    return coordinate;
}

static inline uint32_t c90770_hash_u32(uint32_t hash, uint32_t value) {
    hash ^= value;
    hash *= 16777619u;
    return hash;
}

static inline uint32_t c90770_hash_i32(uint32_t hash, int value) {
    return c90770_hash_u32(hash, (uint32_t)value);
}

static inline void c90770_hash_double(uint32_t *hash, double value, double scale) {
    *hash = c90770_hash_i32(*hash, (int)(value * scale));
}

static inline c90770_gps_gsv_state_t *c90770_gsv_state_for_talker(
    c90770_gps_monitor_state_t *state,
    const char *sentence
) {
    char talker0 = sentence[0];
    char talker1 = sentence[1];
    c90770_gps_gsv_state_t *empty = NULL;

    for (size_t i = 0; i < C90770_GPS_MAX_CONSTELLATIONS; ++i) {
        if (state->gsv[i].talker[0] == talker0 && state->gsv[i].talker[1] == talker1) {
            return &state->gsv[i];
        }
        if (empty == NULL && state->gsv[i].talker[0] == '\0') {
            empty = &state->gsv[i];
        }
    }

    if (empty == NULL) {
        empty = &state->gsv[0];
    }

    memset(empty, 0, sizeof(*empty));
    empty->talker[0] = talker0;
    empty->talker[1] = talker1;
    empty->talker[2] = '\0';
    return empty;
}

static inline void c90770_remove_satellites_for_talker(c90770_gps_monitor_state_t *state, const char *talker) {
    size_t write = 0;
    for (size_t read = 0; read < state->satellite_count; ++read) {
        if (state->satellites[read].talker[0] == talker[0] && state->satellites[read].talker[1] == talker[1]) {
            continue;
        }
        if (write != read) {
            state->satellites[write] = state->satellites[read];
        }
        ++write;
    }
    state->satellite_count = write;
}

static inline void c90770_add_satellite(
    c90770_gps_monitor_state_t *state,
    const char *talker,
    int prn,
    int elevation,
    int azimuth,
    const char *snr
) {
    if (state->satellite_count >= C90770_GPS_MAX_SATELLITES || prn < 0) {
        return;
    }

    c90770_gps_satellite_t *satellite = &state->satellites[state->satellite_count++];
    satellite->talker[0] = talker[0];
    satellite->talker[1] = talker[1];
    satellite->talker[2] = '\0';
    satellite->prn = prn;
    satellite->elevation = elevation;
    satellite->azimuth = azimuth;
    satellite->has_snr = snr != NULL && snr[0] != '\0';
    satellite->snr = satellite->has_snr ? c90770_parse_int_field(snr, 0) : 0;
}

static inline uint32_t c90770_satellite_hash(const c90770_gps_monitor_state_t *state) {
    uint32_t hash = 2166136261u;
    hash = c90770_hash_u32(hash, (uint32_t)state->satellite_count);

    for (size_t i = 0; i < state->satellite_count; ++i) {
        const c90770_gps_satellite_t *satellite = &state->satellites[i];
        hash = c90770_hash_u32(hash, (uint32_t)satellite->talker[0]);
        hash = c90770_hash_u32(hash, (uint32_t)satellite->talker[1]);
        hash = c90770_hash_i32(hash, satellite->prn);
        hash = c90770_hash_i32(hash, satellite->elevation);
        hash = c90770_hash_i32(hash, satellite->azimuth);
        hash = c90770_hash_i32(hash, satellite->has_snr ? satellite->snr : -1);
    }

    return hash;
}

static inline int c90770_satellites_in_view_reported(const c90770_gps_monitor_state_t *state) {
    int total = 0;
    for (size_t i = 0; i < C90770_GPS_MAX_CONSTELLATIONS; ++i) {
        if (state->gsv[i].talker[0] != '\0') {
            total += state->gsv[i].satellites_in_view;
        }
    }
    return total;
}

static inline uint32_t c90770_fix_hash(const c90770_gps_monitor_state_t *state) {
    uint32_t hash = 2166136261u;
    hash = c90770_hash_u32(hash, state->have_coordinates ? 1u : 0u);
    hash = c90770_hash_i32(hash, state->fix_quality);
    hash = c90770_hash_i32(hash, state->satellites_used);
    c90770_hash_double(&hash, state->latitude_degrees, 1000000.0);
    c90770_hash_double(&hash, state->longitude_degrees, 1000000.0);
    c90770_hash_double(&hash, state->altitude_meters, 10.0);
    return hash;
}

static inline void c90770_log_satellites_if_changed(c90770_gps_monitor_state_t *state) {
    uint32_t hash = c90770_satellite_hash(state);
    if (hash == state->last_satellite_hash) {
        return;
    }
    state->last_satellite_hash = hash;

    int tracked = 0;
    int geometry = 0;
    int reported = c90770_satellites_in_view_reported(state);
    if (reported < (int)state->satellite_count) {
        reported = (int)state->satellite_count;
    }

    for (size_t i = 0; i < state->satellite_count; ++i) {
        if (state->satellites[i].elevation >= 0 && state->satellites[i].azimuth >= 0) {
            ++geometry;
        }
        if (state->satellites[i].has_snr && state->satellites[i].snr > 0) {
            ++tracked;
        }
    }

    printf("gps satellites seen: %u", (unsigned)state->satellite_count);
    for (size_t i = 0; i < state->satellite_count; ++i) {
        const c90770_gps_satellite_t *satellite = &state->satellites[i];
        printf(" %s%02d(el=%d az=%d", satellite->talker, satellite->prn, satellite->elevation, satellite->azimuth);
        if (satellite->has_snr) {
            printf(" snr=%d", satellite->snr);
        } else {
            printf(" snr=?");
        }
        printf(")");
    }
    printf("\n");
    printf(
        "gps almanac progress: geometry=%d/%u tracked=%d/%u\n",
        geometry,
        (unsigned)reported,
        tracked,
        (unsigned)reported
    );
}

static inline void c90770_log_fix_if_changed(c90770_gps_monitor_state_t *state) {
    uint32_t hash = c90770_fix_hash(state);
    if (hash == state->last_fix_hash) {
        return;
    }
    state->last_fix_hash = hash;

    if (!state->have_coordinates) {
        printf("gps coordinates: not derived yet");
        if (state->fix_quality > 0) {
            printf(" fix_quality=%d", state->fix_quality);
        }
        printf("\n");
        return;
    }

    printf(
        "gps coordinates: lat=%.6f lon=%.6f source=%s sats_used=%d",
        state->latitude_degrees,
        state->longitude_degrees,
        state->fix_source,
        state->satellites_used
    );
    if (state->have_altitude) {
        printf(" alt=%.1fm", state->altitude_meters);
    }
    printf("\n");
}

static inline void c90770_handle_gsv(c90770_gps_monitor_state_t *state, char **fields, size_t field_count) {
    if (field_count < 4u) {
        return;
    }

    c90770_gps_gsv_state_t *gsv = c90770_gsv_state_for_talker(state, fields[0]);
    int total_messages = c90770_parse_int_field(fields[1], 0);
    int message_number = c90770_parse_int_field(fields[2], 0);
    int satellites_in_view = c90770_parse_int_field(fields[3], 0);

    if (message_number <= 1) {
        c90770_remove_satellites_for_talker(state, gsv->talker);
        gsv->total_messages = total_messages;
        gsv->satellites_in_view = satellites_in_view;
        gsv->active = true;
    }

    for (size_t field = 4u; field + 3u < field_count; field += 4u) {
        int prn = c90770_parse_int_field(fields[field], -1);
        int elevation = c90770_parse_int_field(fields[field + 1u], -1);
        int azimuth = c90770_parse_int_field(fields[field + 2u], -1);
        c90770_add_satellite(state, gsv->talker, prn, elevation, azimuth, fields[field + 3u]);
    }

    if (total_messages <= 1 || message_number >= total_messages) {
        gsv->active = false;
        c90770_log_satellites_if_changed(state);
    }
}

static inline void c90770_handle_gga(c90770_gps_monitor_state_t *state, char **fields, size_t field_count) {
    if (field_count < 10u) {
        return;
    }

    int fix_quality = c90770_parse_int_field(fields[6], 0);
    state->fix_quality = fix_quality;
    state->satellites_used = c90770_parse_int_field(fields[7], 0);

    if (fix_quality > 0 && fields[2][0] != '\0' && fields[4][0] != '\0') {
        state->latitude_degrees = c90770_nmea_coordinate_to_degrees(fields[2], fields[3]);
        state->longitude_degrees = c90770_nmea_coordinate_to_degrees(fields[4], fields[5]);
        state->have_coordinates = true;
        memcpy(state->fix_source, "GGA", 4u);

        if (fields[9][0] != '\0') {
            state->altitude_meters = c90770_parse_double_field(fields[9], 0.0);
            state->have_altitude = true;
        }
    }

    c90770_log_fix_if_changed(state);
}

static inline void c90770_handle_rmc(c90770_gps_monitor_state_t *state, char **fields, size_t field_count) {
    if (field_count < 7u) {
        return;
    }

    bool valid = fields[2][0] == 'A';
    if (valid && fields[3][0] != '\0' && fields[5][0] != '\0') {
        state->latitude_degrees = c90770_nmea_coordinate_to_degrees(fields[3], fields[4]);
        state->longitude_degrees = c90770_nmea_coordinate_to_degrees(fields[5], fields[6]);
        state->have_coordinates = true;
        memcpy(state->fix_source, "RMC", 4u);
    } else if (!valid) {
        state->have_coordinates = false;
    }

    c90770_log_fix_if_changed(state);
}

static inline void c90770_handle_gsa(c90770_gps_monitor_state_t *state, char **fields, size_t field_count) {
    if (field_count < 3u) {
        return;
    }

    int fix_type = c90770_parse_int_field(fields[2], 1);
    if (fix_type <= 1) {
        state->have_coordinates = false;
    }
    c90770_log_fix_if_changed(state);
}

static inline void c90770_handle_txt(char **fields, size_t field_count) {
    if (field_count < 5u || fields[4][0] == '\0') {
        return;
    }

    if (strcmp(fields[4], "ANTENNA OPEN") == 0 || strcmp(fields[4], "ANTENNA OK") == 0) {
        return;
    }

    printf("gps text: %s\n", fields[4]);
}

static inline void c90770_parse_gps_line(c90770_gps_monitor_state_t *state, char *line) {
    if (!c90770_nmea_verify_checksum(line)) {
        ++state->checksum_failures;
        printf("gps checksum failed: %lu\n", (unsigned long)state->checksum_failures);
        return;
    }

    char *fields[C90770_GPS_MAX_FIELDS];
    size_t field_count = c90770_split_nmea_fields(line + 1, fields, C90770_GPS_MAX_FIELDS);
    if (field_count == 0u || strlen(fields[0]) < 5u) {
        return;
    }

    const char *sentence_type = fields[0] + strlen(fields[0]) - 3u;
    if (strcmp(sentence_type, "GSV") == 0) {
        c90770_handle_gsv(state, fields, field_count);
    } else if (strcmp(sentence_type, "GGA") == 0) {
        c90770_handle_gga(state, fields, field_count);
    } else if (strcmp(sentence_type, "RMC") == 0) {
        c90770_handle_rmc(state, fields, field_count);
    } else if (strcmp(sentence_type, "GSA") == 0) {
        c90770_handle_gsa(state, fields, field_count);
    } else if (strcmp(sentence_type, "TXT") == 0) {
        c90770_handle_txt(fields, field_count);
    }
}

static inline void c90770_uart_monitor_gps_parsed() {

    c90770_uart_t gps;
    c90770_uart_init_default(&gps);
    c90770_gps_monitor_state_t state = {0};

    char line[128];

    while (true) {
        size_t n = c90770_uart_read_line_timeout(&gps, line, sizeof(line), 1000000);

        if (n > 0) {
            c90770_parse_gps_line(&state, line);
        } else {
            printf("timeout trying to read gps\n");
        }
    }

}

static inline void c90770_uart_monitor_gps() {

    c90770_uart_t gps;
    c90770_uart_init_default(&gps);

    char line[128];

    while (true) {
        size_t n = c90770_uart_read_line_timeout(&gps, line, sizeof(line), 1000000);

        if (n > 0) {
            printf("%s", line);

            if (line[n - 1] != '\n') {
                printf("\n");
            }
        } else {
            printf("timeout trying to read gps\n");
        }
    }

}

#endif
