#include "battery_logic.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define TAG "BATTERY"

// Assuming GPIO0 for ADC (ADC1_CHANNEL_0). User needs to confirm/adjust this.
#define BATTERY_ADC_CHAN ADC_CHANNEL_0

static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t adc1_cali_handle;
static bool do_calibration = false;

void battery_logic_init(void) {
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, BATTERY_ADC_CHAN, &config));

    // Optional: calibration
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_handle) == ESP_OK) {
        do_calibration = true;
    }
}

uint8_t get_battery_percentage(void) {
    int raw;
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, BATTERY_ADC_CHAN, &raw));

    int voltage_mv = 0;
    if (do_calibration) {
        adc_cali_raw_to_voltage(adc1_cali_handle, raw, &voltage_mv);
    } else {
        // Fallback approximation for 12 attenuation
        voltage_mv = raw * 3300 / 4095;
    }

    // Assuming a 100k/100k voltage divider, the actual battery voltage is 2x the measured voltage.
    // Replace with correct multiplier for your specific hardware.
    float actual_voltage = (voltage_mv * 2.0f) / 1000.0f;

    if (actual_voltage >= 4.2f) return 100;
    if (actual_voltage <= 3.2f) return 0;

    // Simple linear interpolation
    float percentage = ((actual_voltage - 3.2f) / (4.2f - 3.2f)) * 100.0f;
    if (percentage > 100.0f) percentage = 100.0f;
    if (percentage < 0.0f) percentage = 0.0f;

    return (uint8_t)percentage;
}
