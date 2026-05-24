// jippity ported from https://github.com/adafruit/Adafruit_CircuitPython_OV5640

#ifndef OV5640_H
#define OV5640_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "pico/types.h"

typedef enum {
    OV5640_COLOR_RGB = 0,
    OV5640_COLOR_YUV = 1,
    OV5640_COLOR_GRAYSCALE = 2,
    OV5640_COLOR_JPEG = 3,
} ov5640_colorspace_t;

typedef enum {
    OV5640_SIZE_96X96 = 0,
    OV5640_SIZE_QQVGA,
    OV5640_SIZE_QCIF,
    OV5640_SIZE_HQVGA,
    OV5640_SIZE_240X240,
    OV5640_SIZE_QVGA,
    OV5640_SIZE_CIF,
    OV5640_SIZE_HVGA,
    OV5640_SIZE_VGA,
    OV5640_SIZE_SVGA,
    OV5640_SIZE_XGA,
    OV5640_SIZE_HD,
    OV5640_SIZE_SXGA,
    OV5640_SIZE_UXGA,
    OV5640_SIZE_QHDA,
    OV5640_SIZE_WQXGA,
    OV5640_SIZE_PFHD,
    OV5640_SIZE_QSXGA,
    OV5640_SIZE_COUNT,
} ov5640_size_t;

typedef struct {
    i2c_inst_t *i2c;
    uint8_t i2c_addr;
    uint sda_pin;
    uint scl_pin;
    uint data_base_pin;
    uint pclk_pin;
    uint vsync_pin;
    uint href_pin;
    int mclk_pin;
    int reset_pin;
    int shutdown_pin;
    uint32_t i2c_baudrate;
    uint32_t mclk_frequency;
    PIO pio;
    uint pio_sm;
} ov5640_config_t;

typedef struct {
    ov5640_config_t config;
    int dma_channel;
    uint pio_offset;
    uint width;
    uint height;
    ov5640_size_t size;
    ov5640_colorspace_t colorspace;
    bool flip_x;
    bool flip_y;
    bool binning;
    bool scale;
    bool pio_program_loaded;
} ov5640_t;

void ov5640_get_default_config(ov5640_config_t *config);
bool ov5640_init(ov5640_t *camera, const ov5640_config_t *config, ov5640_size_t size);
void ov5640_deinit(ov5640_t *camera);

uint16_t ov5640_chip_id(ov5640_t *camera);
bool ov5640_set_size(ov5640_t *camera, ov5640_size_t size);
bool ov5640_set_colorspace(ov5640_t *camera, ov5640_colorspace_t colorspace);
bool ov5640_set_flip(ov5640_t *camera, bool flip_x, bool flip_y);
bool ov5640_set_test_pattern(ov5640_t *camera, bool enabled);
size_t ov5640_capture_buffer_size(const ov5640_t *camera);
bool ov5640_capture(ov5640_t *camera, uint8_t *buffer, size_t buffer_len);

#endif
