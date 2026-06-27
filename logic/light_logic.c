/* SPDX-License-Identifier: MIT */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "led_strip.h"
#include "connect_logic.h"
#include "status_logic.h"
#include "gps_logic.h"
#include "driver/gpio.h"

#define TAG "LOGIC_LIGHT"

#define LED_STRIP_GPIO GPIO_NUM_8
#define LED_STRIP_MAX_LEDS 1

static led_strip_handle_t led_strip;

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink addressable LED!");
    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_GPIO,
        .max_leds = LED_STRIP_MAX_LEDS,
        .led_model = LED_MODEL_WS2812,
        .flags.invert_out = false,
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    led_strip_clear(led_strip);
}

static void light_monitor_task(void *arg) {
    bool led_on = true;
    while (1) {
        connect_state_t state = connect_logic_get_state();
        bool gps = is_current_gps_data_valid_ui();
        bool is_recording = is_camera_recording();
        if (state < PROTOCOL_CONNECTED) {
            // Not connected: pulse based on GPS state
            if (led_on) {
                if (gps) {
                    led_strip_set_pixel(led_strip, 0, 0, 50, 0); // Green pulse (GPS fixed)
                } else {
                    led_strip_set_pixel(led_strip, 0, 10, 0, 0); // Dim red pulse (No GPS)
                }
                led_strip_refresh(led_strip);
            } else {
                led_strip_clear(led_strip);
            }
        } else {
            // Connected
            uint8_t r = 0, g = 0, b = 0;
            if (gps) {
                // Green: Connected with GPS
                r = 0; g = 50; b = 0;
            } else {
                // Blue: Connected, no GPS
                r = 0; g = 0; b = 50;
            }

            if (is_recording) {
                if (led_on) {
                    led_strip_set_pixel(led_strip, 0, r, g, b);
                    led_strip_refresh(led_strip);
                } else {
                    led_strip_clear(led_strip);
                }
            } else {
                led_strip_set_pixel(led_strip, 0, r, g, b);
                led_strip_refresh(led_strip);
            }
        }
        
        led_on = !led_on; // Toggle state every 500ms
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

int init_light_logic(void) {
    configure_led();
    xTaskCreate(light_monitor_task, "light_monitor", 2048, NULL, 1, NULL);
    return 0;
}
