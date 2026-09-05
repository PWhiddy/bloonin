#ifndef TEST_PICO_H
#define TEST_PICO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef unsigned int uint;
typedef int64_t absolute_time_t;
typedef struct { int unused; } uart_inst_t;
typedef struct { int unused; } i2c_inst_t;
typedef struct { int unused; } mutex_t;
#define uart1 ((uart_inst_t *)1)
#define i2c0 ((i2c_inst_t *)1)
#define GPIO_FUNC_UART 2
#define GPIO_FUNC_I2C 3
#define GPIO_OUT 1
#define UART_PARITY_NONE 0
#define PICO_ERROR_TIMEOUT (-1)

static absolute_time_t test_now;
static uint8_t test_registers[256];
static unsigned test_writes;
static bool test_fail_write;
static bool test_rf_enabled_during_init;

static inline absolute_time_t get_absolute_time(void) { return test_now; }
static inline absolute_time_t delayed_by_us(absolute_time_t t, int64_t us) { return t + us; }
static inline int64_t absolute_time_diff_us(absolute_time_t a, absolute_time_t b) { return b - a; }
static inline bool time_reached(absolute_time_t t) { return test_now >= t; }
static inline absolute_time_t make_timeout_time_us(uint64_t us) { return test_now + us; }
static inline absolute_time_t make_timeout_time_ms(uint32_t ms) { return test_now + ms * 1000ll; }
static inline void sleep_until(absolute_time_t t) { if (t > test_now) test_now = t; }
static inline void sleep_us(uint64_t us) { test_now += us; }
static inline void sleep_ms(uint32_t ms) { test_now += ms * 1000ll; }
static inline int getchar_timeout_us(uint32_t us) { (void)us; return PICO_ERROR_TIMEOUT; }

static inline int i2c_write_blocking(i2c_inst_t *i, uint8_t addr, const uint8_t *data, size_t n, bool stop) {
    (void)i; (void)addr; (void)stop;
    if (test_fail_write) { test_fail_write = false; return -1; }
    ++test_writes;
    for (size_t j = 1; j < n; ++j) {
        test_registers[data[0] + j - 1] = data[j];
        if (data[0] + j - 1 == 3 && !(data[j] & 1u)) test_rf_enabled_during_init = true;
    }
    return (int)n;
}
static inline int i2c_read_blocking(i2c_inst_t *i, uint8_t addr, uint8_t *data, size_t n, bool stop) {
    (void)i; (void)addr; (void)stop;
    memset(data, 0, n);
    return (int)n;
}

/* Unused physical interfaces, present so the production headers compile. */
void gpio_init(uint pin);
void gpio_put(uint pin, bool value);
void gpio_set_dir(uint pin, bool output);
void gpio_set_function(uint pin, uint function);
void gpio_pull_up(uint pin);
uint i2c_init(i2c_inst_t *i, uint baud);
uint uart_init(uart_inst_t *u, uint baud);
void uart_set_format(uart_inst_t *u, uint bits, uint stops, uint parity);
void uart_set_fifo_enabled(uart_inst_t *u, bool enabled);
bool uart_is_readable_within_us(uart_inst_t *u, uint32_t us);
bool uart_is_readable(uart_inst_t *u);
char uart_getc(uart_inst_t *u);
void mutex_init(mutex_t *m);
void mutex_enter_blocking(mutex_t *m);
bool mutex_try_enter(mutex_t *m, uint32_t *owner);
void mutex_exit(mutex_t *m);
void multicore_launch_core1(void (*entry)(void));
bool stdio_usb_connected(void);

#endif
