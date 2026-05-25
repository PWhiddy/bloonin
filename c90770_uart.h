#ifndef C90770_UART_H
#define C90770_UART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

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
