#ifndef SIM_PICO_H
#define SIM_PICO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <pthread.h>

typedef unsigned int uint;
typedef int64_t absolute_time_t;
typedef struct { int unused; } uart_inst_t;
typedef struct { uint32_t dr; } uart_hw_t;
typedef struct { int unused; } i2c_inst_t;
typedef pthread_mutex_t mutex_t;
#define uart1 ((uart_inst_t *)1)
#define i2c0 ((i2c_inst_t *)1)
#define GPIO_FUNC_UART 2
#define GPIO_FUNC_I2C 3
#define GPIO_OUT 1
#define UART_PARITY_NONE 0
#define PICO_ERROR_TIMEOUT (-1)

absolute_time_t get_absolute_time(void);
static inline absolute_time_t delayed_by_us(absolute_time_t t, int64_t us) { return t + us; }
static inline int64_t absolute_time_diff_us(absolute_time_t a, absolute_time_t b) { return b - a; }
static inline bool time_reached(absolute_time_t t) { return get_absolute_time() >= t; }
static inline absolute_time_t make_timeout_time_us(uint64_t us) { return get_absolute_time() + us; }
static inline absolute_time_t make_timeout_time_ms(uint32_t ms) { return get_absolute_time() + ms * 1000ll; }
void sleep_until(absolute_time_t t);
void sleep_us(uint64_t us);
void sleep_ms(uint32_t ms);
int getchar_timeout_us(uint32_t us);
int i2c_write_blocking(i2c_inst_t *i, uint8_t addr, const uint8_t *data, size_t n, bool nostop);
int i2c_read_blocking(i2c_inst_t *i, uint8_t addr, uint8_t *data, size_t n, bool nostop);
bool gpio_get(uint pin);
uint gpio_get_function(uint pin);
uint gpio_get_dir(uint pin);
bool gpio_is_pulled_up(uint pin);
bool gpio_is_pulled_down(uint pin);
uart_hw_t *uart_get_hw(uart_inst_t *u);
void gpio_init(uint pin);
void gpio_put(uint pin, bool value);
void gpio_set_dir(uint pin, bool output);
void gpio_set_function(uint pin, uint function);
void gpio_pull_up(uint pin);
void gpio_disable_pulls(uint pin);
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
