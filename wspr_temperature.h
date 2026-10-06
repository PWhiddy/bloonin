#ifndef WSPR_TEMPERATURE_H
#define WSPR_TEMPERATURE_H

#include "hardware/adc.h"

/* Nominal Pico ADC reference; override with a measured value for calibration. */
#ifndef WSPR_ADC_VREF_VOLTS
#define WSPR_ADC_VREF_VOLTS 3.3
#endif

static inline void wspr_temperature_init(void) {
    adc_init();
    adc_set_temp_sensor_enabled(true);
}

/* Core 0 owns the ADC. Called once when a new GPS sequence is snapshotted.
 * This measures the RP2040 die temperature, using the Pico SDK's conversion. */
static inline double wspr_read_temperature(void) {
    adc_select_input(ADC_TEMPERATURE_CHANNEL_NUM);
    (void)adc_read(); /* Discard the first conversion after changing channels. */
    uint32_t sum = 0u;
    for (unsigned i = 0; i < 16u; ++i) sum += adc_read();
    double voltage = (double)sum * WSPR_ADC_VREF_VOLTS / (16.0 * 4096.0);
    return 27.0 - (voltage - 0.706) / 0.001721;
}

#endif
