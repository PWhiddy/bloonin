#include "ov5640_demo.h"

#include <stdio.h>
#include <stdlib.h>

#include "ov5640.h"
#include "pico/stdlib.h"

void ov5640_demo_run(void) {
    ov5640_config_t config;
    ov5640_t camera;
    ov5640_get_default_config(&config);
    // lower clock to mitigate wire interference
    config.mclk_frequency = 10000000;
    printf("construct camera\n");
    if (!ov5640_init(&camera, &config, OV5640_SIZE_QQVGA)) {
        printf("OV5640 init failed\n");
        return;
    }

    printf("chip id: 0x%04x\n", ov5640_chip_id(&camera));
    ov5640_set_colorspace(&camera, OV5640_COLOR_YUV);
    ov5640_set_flip(&camera, true, true);
    ov5640_set_test_pattern(&camera, false);

    size_t buffer_size = ov5640_capture_buffer_size(&camera);
    uint8_t *buffer = malloc(buffer_size);
    if (!buffer) {
        printf("capture buffer allocation failed: %u bytes\n", (unsigned)buffer_size);
        ov5640_deinit(&camera);
        return;
    }

    static const char chars[] = " .':-+=*%$#";
    char *row = malloc(camera.width + 1);
    if (!row) {
        free(buffer);
        ov5640_deinit(&camera);
        return;
    }
    row[camera.width] = '\0';

    printf("capturing\n");
    if (!ov5640_capture(&camera, buffer, buffer_size)) {
        printf("capture failed\n");
        free(row);
        free(buffer);
        ov5640_deinit(&camera);
        return;
    }
    printf("capture complete\n");
    printf("\033[2J");

    while (true) {
        if (ov5640_capture(&camera, buffer, buffer_size)) {
            for (uint y = 0; y < camera.height; y += 2) {
                printf("\033[%uH", (unsigned)(y / 2));
                for (uint x = 0; x < camera.width; x++) {
                    uint8_t luma = buffer[2 * (camera.width * y + x)];
                    row[x] = chars[luma * (sizeof(chars) - 2) / 255];
                }
                printf("%s\033[K", row);
            }
            printf("\033[J");
        }
        sleep_ms(100);
    }
}
