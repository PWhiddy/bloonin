// Configure a Si5351A clock generator over I2C.

#ifndef SI5351A_IC2_H
#define SI5351A_IC2_H

#include <stdbool.h>
#include <stdint.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "pico/types.h"

#ifndef SI5351A_I2C_INSTANCE
#define SI5351A_I2C_INSTANCE i2c0
#endif

#ifndef SI5351A_I2C_ADDR
#define SI5351A_I2C_ADDR 0x60u
#endif

#ifndef SI5351A_I2C_SDA_GPIO
#define SI5351A_I2C_SDA_GPIO 4u
#endif

#ifndef SI5351A_I2C_SCL_GPIO
#define SI5351A_I2C_SCL_GPIO 5u
#endif

#ifndef SI5351A_I2C_BAUD
#define SI5351A_I2C_BAUD 100000u
#endif

#ifndef SI5351A_READY_TIMEOUT_US
#define SI5351A_READY_TIMEOUT_US 100000u
#endif

#define SI5351A_OUTPUT_HZ 28126200u

typedef enum {
    SI5351A_DRIVE_2MA = 0,
    SI5351A_DRIVE_4MA = 1,
    SI5351A_DRIVE_6MA = 2,
    SI5351A_DRIVE_8MA = 3,
} si5351a_drive_t;

#ifndef SI5351A_CLK0_DRIVE
#define SI5351A_CLK0_DRIVE SI5351A_DRIVE_8MA
#endif

typedef struct {
    i2c_inst_t *i2c;
    uint8_t i2c_addr;
    uint sda_gpio;
    uint scl_gpio;
    uint32_t baudrate;
} si5351a_i2c_t;

static inline bool si5351a_i2c_write_reg(const si5351a_i2c_t *clock, uint8_t reg, uint8_t value) {
    uint8_t data[] = {reg, value};
    return i2c_write_blocking(clock->i2c, clock->i2c_addr, data, sizeof(data), false) == (int)sizeof(data);
}

static inline bool si5351a_i2c_write_regs(
    const si5351a_i2c_t *clock,
    uint8_t start_reg,
    const uint8_t *values,
    size_t value_count
) {
    if (value_count > 8u) {
        return false;
    }

    uint8_t data[9];
    data[0] = start_reg;
    for (size_t i = 0; i < value_count; ++i) {
        data[i + 1u] = values[i];
    }

    return i2c_write_blocking(clock->i2c, clock->i2c_addr, data, value_count + 1u, false) == (int)(value_count + 1u);
}

static inline bool si5351a_i2c_read_reg(const si5351a_i2c_t *clock, uint8_t reg, uint8_t *value) {
    if (i2c_write_blocking(clock->i2c, clock->i2c_addr, &reg, 1, true) != 1) {
        return false;
    }

    return i2c_read_blocking(clock->i2c, clock->i2c_addr, value, 1, false) == 1;
}

static inline bool si5351a_i2c_wait_ready(const si5351a_i2c_t *clock, uint32_t timeout_us) {
    absolute_time_t deadline = make_timeout_time_us(timeout_us);
    while (!time_reached(deadline)) {
        uint8_t status = 0xffu;
        if (!si5351a_i2c_read_reg(clock, 0u, &status)) {
            return false;
        }
        if ((status & 0x80u) == 0u) {
            return true;
        }
        sleep_us(1000);
    }

    return false;
}

static inline bool si5351a_i2c_write_multisynth(
    const si5351a_i2c_t *clock,
    uint8_t base_reg,
    uint32_t p1,
    uint32_t p2,
    uint32_t p3,
    uint8_t r_div,
    bool div_by_4
) {
    uint8_t values[8] = {
        (uint8_t)((p3 >> 8) & 0xffu),
        (uint8_t)(p3 & 0xffu),
        (uint8_t)(((r_div & 0x07u) << 4) | (div_by_4 ? 0x0cu : 0x00u) | ((p1 >> 16) & 0x03u)),
        (uint8_t)((p1 >> 8) & 0xffu),
        (uint8_t)(p1 & 0xffu),
        (uint8_t)((((p3 >> 16) & 0x0fu) << 4) | ((p2 >> 16) & 0x0fu)),
        (uint8_t)((p2 >> 8) & 0xffu),
        (uint8_t)(p2 & 0xffu),
    };

    return si5351a_i2c_write_regs(clock, base_reg, values, sizeof(values));
}

static inline void si5351a_i2c_init_bus(
    si5351a_i2c_t *clock,
    i2c_inst_t *i2c,
    uint sda_gpio,
    uint scl_gpio,
    uint32_t baudrate,
    uint8_t i2c_addr
) {
    clock->i2c = i2c;
    clock->i2c_addr = i2c_addr;
    clock->sda_gpio = sda_gpio;
    clock->scl_gpio = scl_gpio;
    clock->baudrate = baudrate;

    i2c_init(i2c, baudrate);
    gpio_set_function(sda_gpio, GPIO_FUNC_I2C);
    gpio_set_function(scl_gpio, GPIO_FUNC_I2C);
    gpio_pull_up(sda_gpio);
    gpio_pull_up(scl_gpio);
}

static inline void si5351a_i2c_init_default_bus(si5351a_i2c_t *clock) {
    si5351a_i2c_init_bus(
        clock,
        SI5351A_I2C_INSTANCE,
        SI5351A_I2C_SDA_GPIO,
        SI5351A_I2C_SCL_GPIO,
        SI5351A_I2C_BAUD,
        SI5351A_I2C_ADDR
    );
}

static inline bool si5351a_i2c_configure_28_1262_mhz(const si5351a_i2c_t *clock) {
    static const uint32_t plla_p1 = 4096u;   // 25 MHz crystal * 36 = 900 MHz PLLA.
    static const uint32_t plla_p2 = 0u;
    static const uint32_t plla_p3 = 1u;

    // 900 MHz / (31 + 46813 / 46877) = 28.1262 MHz.
    static const uint32_t ms0_p1 = 3583u;
    static const uint32_t ms0_p2 = 38685u;
    static const uint32_t ms0_p3 = 46877u;

    if (!si5351a_i2c_wait_ready(clock, SI5351A_READY_TIMEOUT_US)) {
        return false;
    }

    if (!si5351a_i2c_write_reg(clock, 3u, 0xffu)) {
        return false;
    }

    for (uint8_t reg = 16u; reg <= 23u; ++reg) {
        if (!si5351a_i2c_write_reg(clock, reg, 0x80u)) {
            return false;
        }
    }

    if (!si5351a_i2c_write_reg(clock, 15u, 0x00u)) {
        return false;
    }
    if (!si5351a_i2c_write_reg(clock, 183u, 0xd2u)) {
        return false;
    }
    if (!si5351a_i2c_write_reg(clock, 187u, 0xd0u)) {
        return false;
    }
    if (!si5351a_i2c_write_multisynth(clock, 26u, plla_p1, plla_p2, plla_p3, 0u, false)) {
        return false;
    }
    if (!si5351a_i2c_write_multisynth(clock, 42u, ms0_p1, ms0_p2, ms0_p3, 0u, false)) {
        return false;
    }

    if (!si5351a_i2c_write_reg(clock, 16u, (uint8_t)(0x0cu | (SI5351A_CLK0_DRIVE & 0x03u)))) {
        return false;
    }
    if (!si5351a_i2c_write_reg(clock, 177u, 0x20u)) {
        return false;
    }

    return si5351a_i2c_write_reg(clock, 3u, 0xfeu);
}

static inline bool si5351a_i2c_start_28_1262_mhz(si5351a_i2c_t *clock) {
    si5351a_i2c_init_default_bus(clock);
    return si5351a_i2c_configure_28_1262_mhz(clock);
}

#endif
