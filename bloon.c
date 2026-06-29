/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/stdlib.h"
#include "c90770_uart.h"
#include "si5351a_i2c.h"
#include "ov5640_demo.h"

// Pico W devices use a GPIO on the WIFI chip for the LED,
// so when building for Pico W, CYW43_WL_GPIO_LED_PIN will be defined
#ifdef CYW43_WL_GPIO_LED_PIN
#include "pico/cyw43_arch.h"
#endif

#ifndef LED_DELAY_MS
#define LED_DELAY_MS 50
#endif

// Perform initialisation
int pico_led_init(void) {
#if defined(PICO_DEFAULT_LED_PIN)
    // A device like Pico that uses a GPIO for the LED will define PICO_DEFAULT_LED_PIN
    // so we can use normal GPIO functionality to turn the led on and off
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    return PICO_OK;
#elif defined(CYW43_WL_GPIO_LED_PIN)
    // For Pico W devices we need to initialise the driver etc
    return cyw43_arch_init();
#endif
}

// Turn the led on or off
void pico_set_led(bool led_on) {
#if defined(PICO_DEFAULT_LED_PIN)
    // Just set the GPIO on or off
    gpio_put(PICO_DEFAULT_LED_PIN, led_on);
#elif defined(CYW43_WL_GPIO_LED_PIN)
    // Ask the wifi "driver" to set the GPIO on or off
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
#endif
}

int main() {
    stdio_init_all();

    sleep_ms(5000);
    si5351a_i2c_scan_default_bus();

    si5351a_i2c_t clock;
    int clock_output_hz = 28126200u;
    if (!si5351a_i2c_start_output_hz(&clock, clock_output_hz)) {
        printf(
            "Si5351A init failed: %s, reg=0x%02x, status=0x%02x, addr=0x%02x, i2c=%p, sda_gpio=%u, scl_gpio=%u, power_enable_gpio=%u\n",
            si5351a_i2c_error_string(clock.last_error),
            clock.last_reg,
            clock.last_status,
            clock.i2c_addr,
            clock.i2c,
            clock.sda_gpio,
            clock.scl_gpio,
            clock.power_enable_gpio
        );
    } else {
        printf("Si5351A init OK\n");
    }

    // monitor gps
    //c90770_uart_monitor_gps();
    c90770_uart_monitor_gps_parsed();    

    ov5640_demo_run();

    int rc = pico_led_init();
    hard_assert(rc == PICO_OK);
    while (true) {
        pico_set_led(true);
        sleep_ms(LED_DELAY_MS);
        pico_set_led(false);
        sleep_ms(LED_DELAY_MS);
    }
}
