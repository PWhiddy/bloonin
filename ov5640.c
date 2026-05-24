// jippity ported from https://github.com/adafruit/Adafruit_CircuitPython_OV5640

#include "ov5640.h"

#include <string.h>

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pio_instructions.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

#define OV5640_I2C_ADDR 0x3c
#define REG_DLY 0xffff

#define SYSTEM_CTROL0 0x3008
#define SYSTEM_RESET00 0x3000
#define SYSTEM_RESET02 0x3002
#define CLOCK_ENABLE02 0x3006
#define CHIP_ID_HIGH 0x300a
#define DRIVE_CAPABILITY 0x302c
#define X_ADDR_ST_H 0x3800
#define X_ADDR_END_H 0x3804
#define X_OUTPUT_SIZE_H 0x3808
#define X_TOTAL_SIZE_H 0x380c
#define X_OFFSET_H 0x3810
#define X_INCREMENT 0x3814
#define Y_INCREMENT 0x3815
#define TIMING_TC_REG20 0x3820
#define TIMING_TC_REG21 0x3821
#define ISP_CONTROL_01 0x5001
#define FORMAT_CTRL 0x501f
#define FORMAT_CTRL00 0x4300
#define CLOCK_POL_CONTROL 0x4740
#define PRE_ISP_TEST_SETTING_1 0x503d

typedef struct {
    uint16_t reg;
    uint8_t value;
} ov5640_reg_value_t;

typedef struct {
    uint16_t width;
    uint16_t height;
    uint8_t ratio;
} ov5640_resolution_info_t;

typedef struct {
    uint16_t max_width;
    uint16_t max_height;
    uint16_t start_x;
    uint16_t start_y;
    uint16_t end_x;
    uint16_t end_y;
    uint16_t offset_x;
    uint16_t offset_y;
    uint16_t total_x;
    uint16_t total_y;
} ov5640_ratio_info_t;

static const ov5640_reg_value_t sensor_default_regs[] = {
    {SYSTEM_CTROL0, 0x82}, {REG_DLY, 0x0a}, {SYSTEM_CTROL0, 0x42},
    {0x3103, 0x13}, {0x3017, 0xff}, {0x3018, 0xff}, {DRIVE_CAPABILITY, 0xc3},
    {CLOCK_POL_CONTROL, 0x21}, {0x4713, 0x02}, {ISP_CONTROL_01, 0x83},
    {SYSTEM_RESET00, 0x00}, {SYSTEM_RESET02, 0x1c}, {0x3004, 0xff},
    {CLOCK_ENABLE02, 0xc3}, {0x5000, 0xa7}, {ISP_CONTROL_01, 0xa3},
    {0x5003, 0x08}, {0x370c, 0x02}, {0x3634, 0x40}, {0x3a02, 0x03},
    {0x3a03, 0xd8}, {0x3a08, 0x01}, {0x3a09, 0x27}, {0x3a0a, 0x00},
    {0x3a0b, 0xf6}, {0x3a0d, 0x04}, {0x3a0e, 0x03}, {0x3a0f, 0x30},
    {0x3a10, 0x28}, {0x3a11, 0x60}, {0x3a13, 0x43}, {0x3a14, 0x03},
    {0x3a15, 0xd8}, {0x3a18, 0x00}, {0x3a19, 0xf8}, {0x3a1b, 0x30},
    {0x3a1e, 0x26}, {0x3a1f, 0x14}, {0x3600, 0x08}, {0x3601, 0x33},
    {0x3c01, 0xa4}, {0x3c04, 0x28}, {0x3c05, 0x98}, {0x3c06, 0x00},
    {0x3c07, 0x08}, {0x3c08, 0x00}, {0x3c09, 0x1c}, {0x3c0a, 0x9c},
    {0x3c0b, 0x40}, {0x460c, 0x22}, {0x4001, 0x02}, {0x4004, 0x02},
    {0x5180, 0xff}, {0x5181, 0xf2}, {0x5182, 0x00}, {0x5183, 0x14},
    {0x5184, 0x25}, {0x5185, 0x24}, {0x5186, 0x09}, {0x5187, 0x09},
    {0x5188, 0x09}, {0x5189, 0x75}, {0x518a, 0x54}, {0x518b, 0xe0},
    {0x518c, 0xb2}, {0x518d, 0x42}, {0x518e, 0x3d}, {0x518f, 0x56},
    {0x5190, 0x46}, {0x5191, 0xf8}, {0x5192, 0x04}, {0x5193, 0x70},
    {0x5194, 0xf0}, {0x5195, 0xf0}, {0x5196, 0x03}, {0x5197, 0x01},
    {0x5198, 0x04}, {0x5199, 0x12}, {0x519a, 0x04}, {0x519b, 0x00},
    {0x519c, 0x06}, {0x519d, 0x82}, {0x519e, 0x38}, {0x5381, 0x1e},
    {0x5382, 0x5b}, {0x5383, 0x08}, {0x5384, 0x0a}, {0x5385, 0x7e},
    {0x5386, 0x88}, {0x5387, 0x7c}, {0x5388, 0x6c}, {0x5389, 0x10},
    {0x538a, 0x01}, {0x538b, 0x98}, {0x5300, 0x10}, {0x5301, 0x10},
    {0x5302, 0x18}, {0x5303, 0x19}, {0x5304, 0x10}, {0x5305, 0x10},
    {0x5306, 0x08}, {0x5307, 0x16}, {0x5308, 0x40}, {0x5309, 0x10},
    {0x530a, 0x10}, {0x530b, 0x04}, {0x530c, 0x06}, {0x5480, 0x01},
    {0x5481, 0x00}, {0x5482, 0x1e}, {0x5483, 0x3b}, {0x5484, 0x58},
    {0x5485, 0x66}, {0x5486, 0x71}, {0x5487, 0x7d}, {0x5488, 0x83},
    {0x5489, 0x8f}, {0x548a, 0x98}, {0x548b, 0xa6}, {0x548c, 0xb8},
    {0x548d, 0xca}, {0x548e, 0xd7}, {0x548f, 0xe3}, {0x5490, 0x1d},
    {0x5580, 0x06}, {0x5583, 0x40}, {0x5584, 0x10}, {0x5586, 0x20},
    {0x5587, 0x00}, {0x5588, 0x00}, {0x5589, 0x10}, {0x558a, 0x00},
    {0x558b, 0xf8}, {0x501d, 0x40}, {0x3008, 0x02}, {0x3c00, 0x04},
};

static const ov5640_reg_value_t format_rgb565[] = {
    {FORMAT_CTRL, 0x01}, {FORMAT_CTRL00, 0x61}, {SYSTEM_RESET02, 0x1c}, {CLOCK_ENABLE02, 0xc3},
};
static const ov5640_reg_value_t format_yuv422[] = {
    {FORMAT_CTRL, 0x00}, {FORMAT_CTRL00, 0x30},
};
static const ov5640_reg_value_t format_grayscale[] = {
    {FORMAT_CTRL, 0x00}, {FORMAT_CTRL00, 0x10},
};
static const ov5640_reg_value_t format_jpeg[] = {
    {FORMAT_CTRL, 0x00}, {FORMAT_CTRL00, 0x30}, {SYSTEM_RESET02, 0x00}, {CLOCK_ENABLE02, 0xff},
    {0x471c, 0x50},
};

static const ov5640_resolution_info_t resolutions[OV5640_SIZE_COUNT] = {
    {96, 96, 7}, {160, 120, 0}, {176, 144, 6}, {240, 176, 0}, {240, 240, 7},
    {320, 240, 0}, {400, 296, 0}, {480, 320, 1}, {640, 480, 0}, {800, 600, 0},
    {1024, 768, 0}, {1280, 720, 4}, {1280, 1024, 6}, {1600, 1200, 0},
    {2560, 1440, 4}, {2560, 1600, 2}, {1088, 1920, 8}, {2560, 1920, 0},
};

static const ov5640_ratio_info_t ratios[] = {
    {2560, 1920, 0, 0, 2623, 1951, 32, 16, 2844, 1968},
    {2560, 1704, 0, 110, 2623, 1843, 32, 16, 2844, 1752},
    {2560, 1600, 0, 160, 2623, 1791, 32, 16, 2844, 1648},
    {2560, 1536, 0, 192, 2623, 1759, 32, 16, 2844, 1584},
    {2560, 1440, 0, 240, 2623, 1711, 32, 16, 2844, 1488},
    {2560, 1080, 0, 420, 2623, 1531, 32, 16, 2844, 1128},
    {2400, 1920, 80, 0, 2543, 1951, 32, 16, 2684, 1968},
    {1920, 1920, 320, 0, 2543, 1951, 32, 16, 2684, 1968},
    {1088, 1920, 736, 0, 1887, 1951, 32, 16, 1884, 1968},
};

static bool write_reg(ov5640_t *camera, uint16_t reg, uint8_t value) {
    uint8_t data[] = {reg >> 8, reg & 0xff, value};
    return i2c_write_blocking(camera->config.i2c, camera->config.i2c_addr, data, sizeof(data), false) == sizeof(data);
}

static bool read_reg(ov5640_t *camera, uint16_t reg, uint8_t *value) {
    uint8_t addr[] = {reg >> 8, reg & 0xff};
    if (i2c_write_blocking(camera->config.i2c, camera->config.i2c_addr, addr, sizeof(addr), true) != sizeof(addr)) {
        return false;
    }
    return i2c_read_blocking(camera->config.i2c, camera->config.i2c_addr, value, 1, false) == 1;
}

static bool write_reg16(ov5640_t *camera, uint16_t reg, uint16_t value) {
    return write_reg(camera, reg, value >> 8) && write_reg(camera, reg + 1, value & 0xff);
}

static bool write_addr_reg(ov5640_t *camera, uint16_t reg, uint16_t x_value, uint16_t y_value) {
    return write_reg16(camera, reg, x_value) && write_reg16(camera, reg + 2, y_value);
}

static bool write_regs(ov5640_t *camera, const ov5640_reg_value_t *regs, size_t count) {
    for (size_t i = 0; i < count; i++) {
        if (regs[i].reg == REG_DLY) {
            sleep_ms(regs[i].value);
        } else if (!write_reg(camera, regs[i].reg, regs[i].value)) {
            return false;
        }
    }
    return true;
}

static bool write_reg_bits(ov5640_t *camera, uint16_t reg, uint8_t mask, bool enabled) {
    uint8_t value;
    if (!read_reg(camera, reg, &value)) {
        return false;
    }
    value = enabled ? (value | mask) : (value & (uint8_t)~mask);
    return write_reg(camera, reg, value);
}

static void init_mclk(uint gpio, uint32_t frequency) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(gpio);
    uint32_t sys_hz = clock_get_hz(clk_sys);
    uint32_t divider = 1;
    uint32_t wrap = sys_hz / frequency;
    while (wrap > 65535 && divider < 255) {
        divider++;
        wrap = sys_hz / divider / frequency;
    }
    if (wrap < 2) {
        wrap = 2;
    }
    pwm_set_clkdiv(slice, (float)divider);
    pwm_set_wrap(slice, (uint16_t)(wrap - 1));
    pwm_set_gpio_level(gpio, (uint16_t)(wrap / 2));
    pwm_set_enabled(slice, true);
}

static bool init_capture_pio(ov5640_t *camera) {
    uint16_t instructions[] = {
        pio_encode_wait_gpio(true, camera->config.href_pin),
        pio_encode_wait_gpio(true, camera->config.pclk_pin),
        pio_encode_in(pio_pins, 8),
        pio_encode_wait_gpio(false, camera->config.pclk_pin),
        pio_encode_jmp_pin(1),
        pio_encode_jmp(0),
    };
    const struct pio_program program = {
        .instructions = instructions,
        .length = 6,
        .origin = -1,
    };

    if (!pio_can_add_program(camera->config.pio, &program)) {
        return false;
    }
    camera->pio_offset = pio_add_program(camera->config.pio, &program);
    camera->pio_program_loaded = true;

    for (uint pin = camera->config.data_base_pin; pin < camera->config.data_base_pin + 8; pin++) {
        pio_gpio_init(camera->config.pio, pin);
    }
    pio_gpio_init(camera->config.pio, camera->config.pclk_pin);
    pio_gpio_init(camera->config.pio, camera->config.href_pin);

    pio_sm_config sm_config = pio_get_default_sm_config();
    sm_config_set_in_pins(&sm_config, camera->config.data_base_pin);
    sm_config_set_jmp_pin(&sm_config, camera->config.href_pin);
    sm_config_set_in_shift(&sm_config, true, true, 32);
    sm_config_set_fifo_join(&sm_config, PIO_FIFO_JOIN_RX);
    sm_config_set_clkdiv(&sm_config, 1.0f);
    pio_sm_init(camera->config.pio, camera->config.pio_sm, camera->pio_offset, &sm_config);
    pio_sm_set_consecutive_pindirs(camera->config.pio, camera->config.pio_sm,
                                   camera->config.data_base_pin, 8, false);
    return true;
}

static bool set_image_options(ov5640_t *camera) {
    uint8_t reg20 = 0;
    uint8_t reg21 = 0;
    uint8_t reg4514_test = 0;
    uint8_t reg4514;

    if (camera->colorspace == OV5640_COLOR_JPEG) {
        reg21 |= 0x20;
    }
    if (camera->binning) {
        reg20 |= 0x01;
        reg21 |= 0x01;
        reg4514_test |= 0x04;
    } else {
        reg20 |= 0x40;
    }
    if (camera->flip_y) {
        reg20 |= 0x06;
        reg4514_test |= 0x01;
    }
    if (camera->flip_x) {
        reg21 |= 0x06;
        reg4514_test |= 0x02;
    }

    switch (reg4514_test) {
        case 0: reg4514 = 0x88; break;
        case 1: reg4514 = 0x00; break;
        case 2: reg4514 = 0xbb; break;
        case 3: reg4514 = 0x00; break;
        case 4: reg4514 = 0xaa; break;
        case 5: reg4514 = 0xbb; break;
        case 6: reg4514 = 0xbb; break;
        default: reg4514 = 0xaa; break;
    }

    if (!write_reg(camera, TIMING_TC_REG20, reg20) ||
        !write_reg(camera, TIMING_TC_REG21, reg21) ||
        !write_reg(camera, 0x4514, reg4514)) {
        return false;
    }

    if (camera->binning) {
        return write_reg(camera, 0x4520, 0x0b) &&
               write_reg(camera, X_INCREMENT, 0x31) &&
               write_reg(camera, Y_INCREMENT, 0x31);
    }
    return write_reg(camera, 0x4520, 0x10) &&
           write_reg(camera, X_INCREMENT, 0x11) &&
           write_reg(camera, Y_INCREMENT, 0x11);
}

static bool set_pll(ov5640_t *camera, bool bypass, uint8_t multiplier, uint8_t sys_div,
                    uint8_t pre_div, bool root_2x, uint8_t pclk_root_div,
                    bool pclk_manual, uint8_t pclk_div) {
    return write_reg(camera, 0x3039, bypass ? 0x80 : 0x00) &&
           write_reg(camera, 0x3034, 0x1a) &&
           write_reg(camera, 0x3035, 0x01 | ((sys_div & 0x0f) << 4)) &&
           write_reg(camera, 0x3036, multiplier) &&
           write_reg(camera, 0x3037, (pre_div & 0x0f) | (root_2x ? 0x10 : 0x00)) &&
           write_reg(camera, 0x3108, ((pclk_root_div & 0x03) << 4) | 0x06) &&
           write_reg(camera, 0x3824, pclk_div & 0x1f) &&
           write_reg(camera, 0x460c, pclk_manual ? 0x22 : 0x22) &&
           write_reg(camera, 0x3103, 0x13);
}

static bool set_colorspace_regs(ov5640_t *camera) {
    switch (camera->colorspace) {
        case OV5640_COLOR_RGB:
            return write_regs(camera, format_rgb565, count_of(format_rgb565));
        case OV5640_COLOR_YUV:
            return write_regs(camera, format_yuv422, count_of(format_yuv422));
        case OV5640_COLOR_GRAYSCALE:
            return write_regs(camera, format_grayscale, count_of(format_grayscale));
        case OV5640_COLOR_JPEG:
            return write_regs(camera, format_jpeg, count_of(format_jpeg));
        default:
            return false;
    }
}

static bool apply_size_and_colorspace(ov5640_t *camera) {
    if (camera->size >= OV5640_SIZE_COUNT) {
        return false;
    }

    const ov5640_resolution_info_t *resolution = &resolutions[camera->size];
    const ov5640_ratio_info_t *ratio = &ratios[resolution->ratio];
    camera->width = resolution->width;
    camera->height = resolution->height;
    camera->binning = camera->width <= ratio->max_width / 2 && camera->height <= ratio->max_height / 2;
    camera->scale = !((camera->width == ratio->max_width && camera->height == ratio->max_height) ||
                      (camera->width == ratio->max_width / 2 && camera->height == ratio->max_height / 2));

    if (!write_addr_reg(camera, X_ADDR_ST_H, ratio->start_x, ratio->start_y) ||
        !write_addr_reg(camera, X_ADDR_END_H, ratio->end_x, ratio->end_y) ||
        !write_addr_reg(camera, X_OUTPUT_SIZE_H, camera->width, camera->height)) {
        return false;
    }

    if (!camera->binning) {
        if (!write_addr_reg(camera, X_TOTAL_SIZE_H, ratio->total_x, ratio->total_y) ||
            !write_addr_reg(camera, X_OFFSET_H, ratio->offset_x, ratio->offset_y)) {
            return false;
        }
    } else {
        uint16_t total_x = camera->width > 920 ? ratio->total_x - 200 : 2060;
        if (!write_addr_reg(camera, X_TOTAL_SIZE_H, total_x, ratio->total_y / 2) ||
            !write_addr_reg(camera, X_OFFSET_H, ratio->offset_x / 2, ratio->offset_y / 2)) {
            return false;
        }
    }

    if (!write_reg_bits(camera, ISP_CONTROL_01, 0x20, camera->scale) || !set_image_options(camera)) {
        return false;
    }

    if (camera->colorspace == OV5640_COLOR_JPEG) {
        uint8_t sys_mul = 200;
        if (camera->size < OV5640_SIZE_QVGA) {
            sys_mul = 160;
        }
        if (camera->size < OV5640_SIZE_XGA) {
            sys_mul = 180;
        }
        if (!set_pll(camera, false, sys_mul, 4, 2, false, 2, true, 4)) {
            return false;
        }
    } else if (!set_pll(camera, false, 32, 1, 1, false, 1, true, 4)) {
        return false;
    }

    return set_colorspace_regs(camera);
}

void ov5640_get_default_config(ov5640_config_t *config) {
    memset(config, 0, sizeof(*config));
    config->i2c = i2c0;
    config->i2c_addr = OV5640_I2C_ADDR;
    config->scl_pin = 9;
    config->sda_pin = 8;
    config->data_base_pin = 12;
    config->pclk_pin = 11;
    config->vsync_pin = 7;
    config->href_pin = 21;
    config->mclk_pin = 20;
    config->reset_pin = 10;
    config->shutdown_pin = -1;
    config->i2c_baudrate = 100000;
    config->mclk_frequency = 20000000;
    config->pio = pio0;
    config->pio_sm = 0;
}

bool ov5640_init(ov5640_t *camera, const ov5640_config_t *config, ov5640_size_t size) {
    memset(camera, 0, sizeof(*camera));
    camera->config = *config;
    camera->dma_channel = -1;
    camera->colorspace = OV5640_COLOR_RGB;
    camera->size = size;

    if (config->mclk_pin >= 0) {
        init_mclk((uint)config->mclk_pin, config->mclk_frequency);
    }
    if (config->shutdown_pin >= 0) {
        gpio_init((uint)config->shutdown_pin);
        gpio_set_dir((uint)config->shutdown_pin, GPIO_OUT);
        gpio_put((uint)config->shutdown_pin, 1);
        sleep_ms(5);
        gpio_put((uint)config->shutdown_pin, 0);
    }
    if (config->reset_pin >= 0) {
        gpio_init((uint)config->reset_pin);
        gpio_set_dir((uint)config->reset_pin, GPIO_OUT);
        gpio_put((uint)config->reset_pin, 0);
        sleep_ms(1);
        gpio_put((uint)config->reset_pin, 1);
        sleep_ms(20);
    }

    i2c_init(config->i2c, config->i2c_baudrate);
    gpio_set_function(config->sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(config->scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(config->sda_pin);
    gpio_pull_up(config->scl_pin);
    gpio_init(config->vsync_pin);
    gpio_set_dir(config->vsync_pin, GPIO_IN);

    camera->dma_channel = dma_claim_unused_channel(false);
    if (camera->dma_channel < 0 || !write_regs(camera, sensor_default_regs, count_of(sensor_default_regs))) {
        ov5640_deinit(camera);
        return false;
    }
    if (!init_capture_pio(camera) || !apply_size_and_colorspace(camera)) {
        ov5640_deinit(camera);
        return false;
    }
    return true;
}

void ov5640_deinit(ov5640_t *camera) {
    if (camera->dma_channel >= 0) {
        dma_channel_abort(camera->dma_channel);
        dma_channel_unclaim(camera->dma_channel);
        camera->dma_channel = -1;
    }
    if (camera->pio_program_loaded) {
        pio_sm_set_enabled(camera->config.pio, camera->config.pio_sm, false);
        pio_remove_program(camera->config.pio, &(const struct pio_program){
            .instructions = NULL,
            .length = 6,
            .origin = -1,
        }, camera->pio_offset);
        camera->pio_program_loaded = false;
    }
    if (camera->config.mclk_pin >= 0) {
        pwm_set_enabled(pwm_gpio_to_slice_num((uint)camera->config.mclk_pin), false);
    }
}

uint16_t ov5640_chip_id(ov5640_t *camera) {
    uint8_t high = 0;
    uint8_t low = 0;
    read_reg(camera, CHIP_ID_HIGH, &high);
    read_reg(camera, CHIP_ID_HIGH + 1, &low);
    return ((uint16_t)high << 8) | low;
}

bool ov5640_set_size(ov5640_t *camera, ov5640_size_t size) {
    camera->size = size;
    return apply_size_and_colorspace(camera);
}

bool ov5640_set_colorspace(ov5640_t *camera, ov5640_colorspace_t colorspace) {
    camera->colorspace = colorspace;
    return apply_size_and_colorspace(camera);
}

bool ov5640_set_flip(ov5640_t *camera, bool flip_x, bool flip_y) {
    camera->flip_x = flip_x;
    camera->flip_y = flip_y;
    return set_image_options(camera);
}

bool ov5640_set_test_pattern(ov5640_t *camera, bool enabled) {
    return write_reg(camera, PRE_ISP_TEST_SETTING_1, enabled ? 0x80 : 0x00);
}

size_t ov5640_capture_buffer_size(const ov5640_t *camera) {
    if (camera->colorspace == OV5640_COLOR_GRAYSCALE) {
        return camera->width * camera->height;
    }
    if (camera->colorspace == OV5640_COLOR_JPEG) {
        return 2 * (camera->width * camera->height / 10);
    }
    return camera->width * camera->height * 2;
}

bool ov5640_capture(ov5640_t *camera, uint8_t *buffer, size_t buffer_len) {
    size_t expected = ov5640_capture_buffer_size(camera);
    if (buffer_len < expected || camera->dma_channel < 0 || (expected & 3u) != 0) {
        return false;
    }

    pio_sm_set_enabled(camera->config.pio, camera->config.pio_sm, false);
    pio_sm_clear_fifos(camera->config.pio, camera->config.pio_sm);
    pio_sm_restart(camera->config.pio, camera->config.pio_sm);

    absolute_time_t timeout = make_timeout_time_ms(1000);
    while (!gpio_get(camera->config.vsync_pin)) {
        if (absolute_time_diff_us(get_absolute_time(), timeout) <= 0) {
            return false;
        }
    }
    while (gpio_get(camera->config.vsync_pin)) {
        if (absolute_time_diff_us(get_absolute_time(), timeout) <= 0) {
            return false;
        }
    }

    dma_channel_config dma_config = dma_channel_get_default_config(camera->dma_channel);
    channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_32);
    channel_config_set_read_increment(&dma_config, false);
    channel_config_set_write_increment(&dma_config, true);
    channel_config_set_dreq(&dma_config, pio_get_dreq(camera->config.pio, camera->config.pio_sm, false));

    dma_channel_configure(camera->dma_channel, &dma_config, buffer,
                          &camera->config.pio->rxf[camera->config.pio_sm],
                          expected / 4, false);
    pio_sm_set_enabled(camera->config.pio, camera->config.pio_sm, true);
    dma_channel_start(camera->dma_channel);
    timeout = make_timeout_time_ms(1000);
    while (dma_channel_is_busy(camera->dma_channel)) {
        if (absolute_time_diff_us(get_absolute_time(), timeout) <= 0) {
            dma_channel_abort(camera->dma_channel);
            pio_sm_set_enabled(camera->config.pio, camera->config.pio_sm, false);
            return false;
        }
    }
    pio_sm_set_enabled(camera->config.pio, camera->config.pio_sm, false);
    return true;
}
