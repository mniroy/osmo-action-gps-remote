/* SPDX-License-Identifier: MIT */
/*
 * Copyright (C) 2025 SZ DJI Technology Co., Ltd.
 *
 * light_logic.c — Stub implementation for ESP32-C6 Super Mini
 *
 * The Super Mini has no onboard RGB LED. LED state is reported via
 * serial log only. No WS2812 / RMT / led_strip dependency.
 *
 * To add a real LED later, wire an LED+resistor to a free GPIO and
 * replace the ESP_LOGI calls with gpio_set_level() calls.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "connect_logic.h"
#include "status_logic.h"
#include "gps_logic.h"

#define TAG "LOGIC_LIGHT"

/* State colors (for log readability) */
static const char* state_color_name(connect_state_t state, bool gps_valid) {
    switch (state) {
        case BLE_INIT_COMPLETE:
            return "YELLOW (BLE ready, waiting for long-press to connect)";
        case BLE_SEARCHING:
            return "BLUE-BLINK (scanning for Osmo camera)";
        case BLE_CONNECTED:
            return "BLUE (BLE connected)";
        case PROTOCOL_CONNECTED:
            return gps_valid ? "PURPLE (protocol connected + GPS fix)" : "GREEN (protocol connected)";
        default:
            return "RED (initializing)";
    }
}

/**
 * @brief Light monitor task — logs current state periodically
 */
static void light_monitor_task(void *arg) {
    connect_state_t last_state = -1;
    bool last_gps = false;

    while (1) {
        connect_state_t state = connect_logic_get_state();
        bool gps = is_current_gps_data_valid();

        if (state != last_state || gps != last_gps) {
            last_state = state;
            last_gps = gps;
            ESP_LOGI(TAG, "[STATUS] %s", state_color_name(state, gps));
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/**
 * @brief Initialize light logic (stub — no hardware LED)
 * @return 0 on success
 */
int init_light_logic(void) {
    ESP_LOGI(TAG, "Light logic init (stub — no LED hardware on Super Mini)");
    ESP_LOGI(TAG, "State changes will be reported via serial log");

    xTaskCreate(light_monitor_task, "light_monitor", 2048, NULL, 1, NULL);
    return 0;
}
