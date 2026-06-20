/* SPDX-License-Identifier: MIT */
/*
 * Copyright (C) 2025 SZ DJI Technology Co., Ltd.
 *
 * Modified for ESP32-C6 Super Mini with momentary switch on GPIO 20.
 * Button behavior:
 *   Short press  : Record / Shutter toggle
 *   Double press : Quick Switch (QS) mode change  (< DOUBLE_PRESS_WINDOW between presses)
 *   Long press   : BLE scan & connect to nearest Osmo camera
 */

#include <time.h>
#include "key_logic.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "data.h"
#include "enums_logic.h"
#include "connect_logic.h"
#include "command_logic.h"
#include "status_logic.h"
#include "dji_protocol_data_structures.h"

static const char *TAG = "LOGIC_KEY";

// Current key event
static key_event_t current_key_event = KEY_EVENT_NONE;

// Key state
static bool key_pressed = false;
static TickType_t key_press_start_time = 0;

// Double-press detection state
static TickType_t last_release_time = 0;    // time of last button release
static bool waiting_for_double = false;     // true after first single press, waiting to see if double follows

// Thresholds
#define LONG_PRESS_THRESHOLD  pdMS_TO_TICKS(1000)  // > 1 s  → long press
#define KEY_SCAN_INTERVAL     pdMS_TO_TICKS(20)    // 20 ms polling interval
#define DOUBLE_PRESS_WINDOW   pdMS_TO_TICKS(400)   // < 400 ms between releases → double press
#define DEBOUNCE_MS           pdMS_TO_TICKS(30)    // 30 ms debounce

/* -----------------------------------------------------------------------
 * Handler: Long press — BLE scan & protocol connect
 * ----------------------------------------------------------------------- */
static void handle_long_press(void) {
    /* Data layer is initialized at boot — just re-trigger BLE connect */

    /* Disconnect existing BLE if connected */
    connect_state_t current_state = connect_logic_get_state();
    if (current_state >= BLE_INIT_COMPLETE) {
        ESP_LOGI(TAG, "State=%d, disconnecting BLE first...", current_state);
        int res = connect_logic_ble_disconnect();
        if (res == -1) {
            ESP_LOGE(TAG, "BLE disconnect failed");
            return;
        }
    }

    /* Scan & connect to nearest Osmo camera */
    ESP_LOGI(TAG, "Long press: starting BLE scan for nearest Osmo camera...");
    int res = connect_logic_ble_connect(false);
    if (res == -1) {
        ESP_LOGE(TAG, "BLE connect failed");
        return;
    }
    ESP_LOGI(TAG, "BLE connected");

    /* Protocol handshake */
    uint32_t g_device_id = 0x12345678;
    uint8_t g_mac_addr_len = 6;
    int8_t g_mac_addr[6] = {0x38, 0x34, 0x56, 0x78, 0x9A, 0xBC};
    uint32_t g_fw_version = 0x00;
    uint8_t g_verify_mode = 0;
    uint16_t g_verify_data = 0;
    uint8_t g_camera_reserved = 0;

    srand((unsigned int)time(NULL));
    g_verify_data = (uint16_t)(rand() % 10000);
    res = connect_logic_protocol_connect(
        g_device_id, g_mac_addr_len, g_mac_addr,
        g_fw_version, g_verify_mode, g_verify_data, g_camera_reserved
    );
    if (res == -1) {
        ESP_LOGE(TAG, "Protocol connect failed");
        return;
    }
    ESP_LOGI(TAG, "Protocol connected");

    /* Query version */
    version_query_response_frame_t *ver = command_logic_get_version();
    if (ver != NULL) free(ver);

    /* Subscribe to camera status at 2 Hz */
    res = subscript_camera_status(PUSH_MODE_PERIODIC_WITH_STATE_CHANGE, PUSH_FREQ_2HZ);
    if (res == -1) {
        ESP_LOGE(TAG, "Subscribe camera status failed");
    } else {
        ESP_LOGI(TAG, "Subscribed to camera status at 2 Hz");
    }
}

/* -----------------------------------------------------------------------
 * Handler: Single press — Record / Shutter toggle
 * ----------------------------------------------------------------------- */
static void handle_single_press(void) {
    camera_status_t current_status = current_camera_status;
    camera_mode_t current_mode = current_camera_mode;

    ESP_LOGI(TAG, "Single press: mode=%d status=%d", current_mode, current_status);

    if (current_mode == CAMERA_MODE_PHOTO || current_status == CAMERA_STATUS_LIVE_STREAMING) {
        ESP_LOGI(TAG, "Starting recording...");
        record_control_response_frame_t *resp = command_logic_start_record();
        if (resp != NULL) {
            ESP_LOGI(TAG, "Recording started");
            free(resp);
        } else {
            ESP_LOGE(TAG, "Start record failed — attempting wakeup");
            connect_logic_ble_wakeup();
        }
    } else if (is_camera_recording()) {
        ESP_LOGI(TAG, "Stopping recording...");
        record_control_response_frame_t *resp = command_logic_stop_record();
        if (resp != NULL) {
            ESP_LOGI(TAG, "Recording stopped");
            free(resp);
        } else {
            ESP_LOGE(TAG, "Stop record failed");
        }
    } else {
        ESP_LOGI(TAG, "Camera not in recordable state, ignoring");
    }
}

/* -----------------------------------------------------------------------
 * Handler: Double press — Quick Switch (QS) mode
 * ----------------------------------------------------------------------- */
static void handle_double_press(void) {
    ESP_LOGI(TAG, "Double press: Quick Switch mode");
    key_report_response_frame_t *resp = command_logic_key_report_qs();
    if (resp != NULL) {
        ESP_LOGI(TAG, "QS mode switched");
        free(resp);
    } else {
        ESP_LOGE(TAG, "QS mode switch failed");
    }
}

/* -----------------------------------------------------------------------
 * Key scan task
 *
 * The momentary switch connects GPIO 20 to GND when pressed.
 * Internal pull-up is enabled → idle = HIGH (1), pressed = LOW (0).
 *
 * State machine:
 *   PRESSED edge:  record start time
 *   HELD:          if > LONG_PRESS_THRESHOLD → fire long press (once)
 *   RELEASE edge:  check duration
 *     - Short press:
 *         if waiting_for_double → fire double press
 *         else enter double-wait window
 *   DOUBLE WINDOW TIMEOUT: fire single press
 * ----------------------------------------------------------------------- */
static void key_scan_task(void *arg) {
    while (1) {
        bool btn_low = (gpio_get_level(BOOT_KEY_GPIO) == 0);  // active low

        if (btn_low && !key_pressed) {
            /* ---- PRESS EDGE ---- */
            key_pressed = true;
            key_press_start_time = xTaskGetTickCount();
            current_key_event = KEY_EVENT_NONE;

        } else if (btn_low && key_pressed) {
            /* ---- HELD ---- */
            TickType_t duration = xTaskGetTickCount() - key_press_start_time;
            if (duration >= LONG_PRESS_THRESHOLD && current_key_event != KEY_EVENT_LONG_PRESS) {
                current_key_event = KEY_EVENT_LONG_PRESS;
                waiting_for_double = false;  // cancel any pending double-press window
                handle_long_press();
            }

        } else if (!btn_low && key_pressed) {
            /* ---- RELEASE EDGE ---- */
            key_pressed = false;
            TickType_t duration = xTaskGetTickCount() - key_press_start_time;

            if (duration < LONG_PRESS_THRESHOLD) {
                /* Short press released */
                if (waiting_for_double) {
                    /* Second short press within window → double press */
                    waiting_for_double = false;
                    current_key_event = KEY_EVENT_DOUBLE;
                    ESP_LOGI(TAG, "Double press detected");
                    handle_double_press();
                } else {
                    /* First short press — start double-press window */
                    waiting_for_double = true;
                    last_release_time = xTaskGetTickCount();
                }
            }
            /* If long press was fired, current_key_event is already set */

        } else {
            /* ---- IDLE ---- */
            if (waiting_for_double) {
                TickType_t elapsed = xTaskGetTickCount() - last_release_time;
                if (elapsed > DOUBLE_PRESS_WINDOW) {
                    /* Window expired without second press → single press */
                    waiting_for_double = false;
                    current_key_event = KEY_EVENT_SINGLE;
                    ESP_LOGI(TAG, "Single press detected");
                    handle_single_press();
                }
            }
        }

        vTaskDelay(KEY_SCAN_INTERVAL);
    }
}

/* -----------------------------------------------------------------------
 * Public: Initialize key logic
 * ----------------------------------------------------------------------- */
void key_logic_init(void) {
    /* GPIO 20, internal pull-up, momentary switch to GND */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BOOT_KEY_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,   // active-low button
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    ESP_LOGI(TAG, "Key logic init: GPIO %d (pull-up, active-low)", BOOT_KEY_GPIO);
    ESP_LOGI(TAG, "  Short press  -> record/shutter toggle");
    ESP_LOGI(TAG, "  Double press -> quick switch mode");
    ESP_LOGI(TAG, "  Long press   -> BLE scan & connect");

    xTaskCreate(key_scan_task, "key_scan_task", 4096, NULL, 2, NULL);
}

/* -----------------------------------------------------------------------
 * Public: Get current key event (clears it)
 * ----------------------------------------------------------------------- */
key_event_t key_logic_get_event(void) {
    key_event_t event = current_key_event;
    current_key_event = KEY_EVENT_NONE;
    return event;
}
