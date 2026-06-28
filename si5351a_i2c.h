// Configure a Si5351A clock generator over I2C.

#ifndef SI5351A_IC2_H
#define SI5351A_IC2_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

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
#define SI5351A_I2C_SDA_GPIO 0u
#endif

#ifndef SI5351A_I2C_SCL_GPIO
#define SI5351A_I2C_SCL_GPIO 1u
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

typedef enum {
    SI5351A_ERROR_NONE = 0,
    SI5351A_ERROR_WRITE_FAILED,
    SI5351A_ERROR_READ_SELECT_FAILED,
    SI5351A_ERROR_READ_FAILED,
    SI5351A_ERROR_DEVICE_NOT_READY_TIMEOUT,
    SI5351A_ERROR_DEVICE_NOT_READY_READ_FAILED,
    SI5351A_ERROR_DISABLE_OUTPUTS_FAILED,
    SI5351A_ERROR_DISABLE_CLOCK_FAILED,
    SI5351A_ERROR_PLL_INPUT_FAILED,
    SI5351A_ERROR_XTAL_LOAD_FAILED,
    SI5351A_ERROR_PLLA_CONFIG_FAILED,
    SI5351A_ERROR_MS0_CONFIG_FAILED,
    SI5351A_ERROR_CLK0_CONTROL_FAILED,
    SI5351A_ERROR_PLL_RESET_FAILED,
    SI5351A_ERROR_ENABLE_CLK0_FAILED,
} si5351a_error_t;

#ifndef SI5351A_CLK0_DRIVE
#define SI5351A_CLK0_DRIVE SI5351A_DRIVE_8MA
#endif

typedef struct {
    i2c_inst_t *i2c;
    uint8_t i2c_addr;
    uint sda_gpio;
    uint scl_gpio;
    uint32_t baudrate;
    si5351a_error_t last_error;
    uint8_t last_reg;
    uint8_t last_status;
} si5351a_i2c_t;

static inline void si5351a_i2c_set_error(si5351a_i2c_t *clock, si5351a_error_t error, uint8_t reg) {
    clock->last_error = error;
    clock->last_reg = reg;
}

static inline bool si5351a_i2c_write_reg(si5351a_i2c_t *clock, uint8_t reg, uint8_t value) {
    uint8_t data[] = {reg, value};
    bool ok = i2c_write_blocking(clock->i2c, clock->i2c_addr, data, sizeof(data), false) == (int)sizeof(data);
    if (!ok) {
        si5351a_i2c_set_error(clock, SI5351A_ERROR_WRITE_FAILED, reg);
    }
    return ok;
}

static inline bool si5351a_i2c_write_regs(
    si5351a_i2c_t *clock,
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

    bool ok = i2c_write_blocking(clock->i2c, clock->i2c_addr, data, value_count + 1u, false) == (int)(value_count + 1u);
    if (!ok) {
        si5351a_i2c_set_error(clock, SI5351A_ERROR_WRITE_FAILED, start_reg);
    }
    return ok;
}

static inline bool si5351a_i2c_read_reg(si5351a_i2c_t *clock, uint8_t reg, uint8_t *value) {
    if (i2c_write_blocking(clock->i2c, clock->i2c_addr, &reg, 1, true) != 1) {
        si5351a_i2c_set_error(clock, SI5351A_ERROR_READ_SELECT_FAILED, reg);
        return false;
    }

    bool ok = i2c_read_blocking(clock->i2c, clock->i2c_addr, value, 1, false) == 1;
    if (!ok) {
        si5351a_i2c_set_error(clock, SI5351A_ERROR_READ_FAILED, reg);
    }
    return ok;
}

static inline bool si5351a_i2c_wait_ready(si5351a_i2c_t *clock, uint32_t timeout_us) {
    absolute_time_t deadline = make_timeout_time_us(timeout_us);
    while (!time_reached(deadline)) {
        uint8_t status = 0xffu;
        if (!si5351a_i2c_read_reg(clock, 0u, &status)) {
            si5351a_i2c_set_error(clock, SI5351A_ERROR_DEVICE_NOT_READY_READ_FAILED, 0u);
            return false;
        }
        clock->last_status = status;
        if ((status & 0x80u) == 0u) {
            return true;
        }
        sleep_us(1000);
    }

    si5351a_i2c_set_error(clock, SI5351A_ERROR_DEVICE_NOT_READY_TIMEOUT, 0u);
    return false;
}

static inline void si5351a_i2c_scan_bus(i2c_inst_t *i2c, uint sda_gpio, uint scl_gpio, uint32_t baudrate) {
    i2c_init(i2c, baudrate);
    gpio_set_function(sda_gpio, GPIO_FUNC_I2C);
    gpio_set_function(scl_gpio, GPIO_FUNC_I2C);
    gpio_pull_up(sda_gpio);
    gpio_pull_up(scl_gpio);

    printf("I2C scan on sda_gpio=%u scl_gpio=%u\n", sda_gpio, scl_gpio);
    for (uint8_t addr = 0x08u; addr <= 0x77u; ++addr) {
        uint8_t reg = 0u;
        int result = i2c_write_blocking(i2c, addr, &reg, 1, false);
        if (result == 1) {
            printf("I2C device found at 0x%02x\n", addr);
        }
    }
    printf("I2C scan complete\n");
}

static inline void si5351a_i2c_scan_default_bus(void) {
    si5351a_i2c_scan_bus(SI5351A_I2C_INSTANCE, SI5351A_I2C_SDA_GPIO, SI5351A_I2C_SCL_GPIO, SI5351A_I2C_BAUD);
}

static inline bool si5351a_i2c_write_multisynth(
    si5351a_i2c_t *clock,
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
    clock->last_error = SI5351A_ERROR_NONE;
    clock->last_reg = 0u;
    clock->last_status = 0u;

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

static inline bool si5351a_i2c_configure_28_1262_mhz(si5351a_i2c_t *clock) {
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
        si5351a_i2c_set_error(clock, SI5351A_ERROR_DISABLE_OUTPUTS_FAILED, 3u);
        return false;
    }

    for (uint8_t reg = 16u; reg <= 23u; ++reg) {
        if (!si5351a_i2c_write_reg(clock, reg, 0x80u)) {
            si5351a_i2c_set_error(clock, SI5351A_ERROR_DISABLE_CLOCK_FAILED, reg);
            return false;
        }
    }

    if (!si5351a_i2c_write_reg(clock, 15u, 0x00u)) {
        si5351a_i2c_set_error(clock, SI5351A_ERROR_PLL_INPUT_FAILED, 15u);
        return false;
    }
    if (!si5351a_i2c_write_reg(clock, 183u, 0xd2u)) {
        si5351a_i2c_set_error(clock, SI5351A_ERROR_XTAL_LOAD_FAILED, 183u);
        return false;
    }
    if (!si5351a_i2c_write_reg(clock, 187u, 0xd0u)) {
        si5351a_i2c_set_error(clock, SI5351A_ERROR_PLL_INPUT_FAILED, 187u);
        return false;
    }
    if (!si5351a_i2c_write_multisynth(clock, 26u, plla_p1, plla_p2, plla_p3, 0u, false)) {
        si5351a_i2c_set_error(clock, SI5351A_ERROR_PLLA_CONFIG_FAILED, 26u);
        return false;
    }
    if (!si5351a_i2c_write_multisynth(clock, 42u, ms0_p1, ms0_p2, ms0_p3, 0u, false)) {
        si5351a_i2c_set_error(clock, SI5351A_ERROR_MS0_CONFIG_FAILED, 42u);
        return false;
    }

    if (!si5351a_i2c_write_reg(clock, 16u, (uint8_t)(0x0cu | (SI5351A_CLK0_DRIVE & 0x03u)))) {
        si5351a_i2c_set_error(clock, SI5351A_ERROR_CLK0_CONTROL_FAILED, 16u);
        return false;
    }
    if (!si5351a_i2c_write_reg(clock, 177u, 0x20u)) {
        si5351a_i2c_set_error(clock, SI5351A_ERROR_PLL_RESET_FAILED, 177u);
        return false;
    }

    if (!si5351a_i2c_write_reg(clock, 3u, 0xfeu)) {
        si5351a_i2c_set_error(clock, SI5351A_ERROR_ENABLE_CLK0_FAILED, 3u);
        return false;
    }

    clock->last_error = SI5351A_ERROR_NONE;
    return true;
}

static inline bool si5351a_i2c_start_28_1262_mhz(si5351a_i2c_t *clock) {
    si5351a_i2c_init_default_bus(clock);
    return si5351a_i2c_configure_28_1262_mhz(clock);
}

static inline const char *si5351a_i2c_error_string(si5351a_error_t error) {
    switch (error) {
        case SI5351A_ERROR_NONE:
            return "no error";
        case SI5351A_ERROR_WRITE_FAILED:
            return "I2C write failed";
        case SI5351A_ERROR_READ_SELECT_FAILED:
            return "I2C register-select write failed before read";
        case SI5351A_ERROR_READ_FAILED:
            return "I2C read failed";
        case SI5351A_ERROR_DEVICE_NOT_READY_TIMEOUT:
            return "Si5351A SYS_INIT stayed set until timeout";
        case SI5351A_ERROR_DEVICE_NOT_READY_READ_FAILED:
            return "could not read Si5351A status register";
        case SI5351A_ERROR_DISABLE_OUTPUTS_FAILED:
            return "failed to disable Si5351A outputs";
        case SI5351A_ERROR_DISABLE_CLOCK_FAILED:
            return "failed to disable one Si5351A clock output";
        case SI5351A_ERROR_PLL_INPUT_FAILED:
            return "failed to configure Si5351A PLL input/source register";
        case SI5351A_ERROR_XTAL_LOAD_FAILED:
            return "failed to configure Si5351A crystal load capacitance";
        case SI5351A_ERROR_PLLA_CONFIG_FAILED:
            return "failed to configure Si5351A PLLA multisynth registers";
        case SI5351A_ERROR_MS0_CONFIG_FAILED:
            return "failed to configure Si5351A CLK0 multisynth registers";
        case SI5351A_ERROR_CLK0_CONTROL_FAILED:
            return "failed to configure Si5351A CLK0 control register";
        case SI5351A_ERROR_PLL_RESET_FAILED:
            return "failed to reset Si5351A PLLA";
        case SI5351A_ERROR_ENABLE_CLK0_FAILED:
            return "failed to enable Si5351A CLK0 output";
        default:
            return "unknown Si5351A error";
    }
}

#endif
