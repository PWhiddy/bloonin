#ifndef GPS_DIAGNOSTICS_H
#define GPS_DIAGNOSTICS_H

#include "c90770_uart.h"

/* Passive diagnostics owned by the GPS core. Keep only the last 32 raw bytes;
 * logging is on request so it cannot flood the UART consumer. */
typedef struct {
    uint32_t bytes, sentences, framing, parity, breaks, overruns;
    uint8_t recent[32];
    unsigned next, count;
    absolute_time_t last_byte;
} gps_diagnostics_t;

static inline void gps_diagnostics_record(
    gps_diagnostics_t *diag, uint32_t data, absolute_time_t now
) {
    ++diag->bytes;
    /* RP2040/RP2350 UARTDR: data[7:0], FE[8], PE[9], BE[10], OE[11]. */
    diag->framing += (data >> 8) & 1u;
    diag->parity += (data >> 9) & 1u;
    diag->breaks += (data >> 10) & 1u;
    diag->overruns += (data >> 11) & 1u;
    diag->recent[diag->next] = (uint8_t)data;
    diag->next = (diag->next + 1u) % sizeof(diag->recent);
    if (diag->count < sizeof(diag->recent)) ++diag->count;
    diag->last_byte = now;
}

static inline void gps_diagnostics_hex(const gps_diagnostics_t *diag, char text[97]) {
    static const char hex[] = "0123456789ABCDEF";
    unsigned first = (diag->next + sizeof(diag->recent) - diag->count) % sizeof(diag->recent);
    for (unsigned i = 0; i < diag->count; ++i) {
        uint8_t byte = diag->recent[(first + i) % sizeof(diag->recent)];
        text[3u * i] = hex[byte >> 4];
        text[3u * i + 1u] = hex[byte & 15u];
        text[3u * i + 2u] = ' ';
    }
    text[3u * diag->count] = '\0';
}

static inline void gps_diagnostics_report(
    const gps_diagnostics_t *diag, uint32_t checksum_failures, absolute_time_t now
) {
    printf("GPS DIAG: RX=GP%u baud=%u bytes=%lu sentences=%lu checksum_failures=%lu "
           "framing=%lu parity=%lu breaks=%lu overruns=%lu last_byte_ms=%lld (-1=never)\n",
           (unsigned)C90770_UART_RX_GPIO, (unsigned)C90770_UART_BAUD,
           (unsigned long)diag->bytes, (unsigned long)diag->sentences,
           (unsigned long)checksum_failures, (unsigned long)diag->framing,
           (unsigned long)diag->parity, (unsigned long)diag->breaks,
           (unsigned long)diag->overruns, diag->count ?
           (long long)(absolute_time_diff_us(diag->last_byte, now) / 1000) : -1ll);
    char text[97];
    gps_diagnostics_hex(diag, text);
    printf("GPS DIAG: recent RX hex (oldest first): %s\n", diag->count ? text : "none");
    /* Read the schematic control pins without changing their configuration. */
    const uint pins[] = {C90770_UART_RX_GPIO, C90770_POWER_ENABLE_GPIO, C90770_RESET_GPIO, 3u};
    const char *const labels[] = {"GPS_RX", "GPS_LSWITCH", "GPS_RESET", "GPS_V_BCKP"};
    for (unsigned i = 0; i < 4u; ++i) {
        uint pin = pins[i];
        printf("GPS DIAG: %s GP%u digital=%u function=%u sio_dir=%s pull_up=%u pull_down=%u\n",
               labels[i], pin, (unsigned)gpio_get(pin), (unsigned)gpio_get_function(pin),
               gpio_get_dir(pin) == GPIO_OUT ? "out" : "in",
               (unsigned)gpio_is_pulled_up(pin), (unsigned)gpio_is_pulled_down(pin));
    }
}

#endif
