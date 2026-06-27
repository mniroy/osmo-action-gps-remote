#include "key_logic.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "connect_logic.h"
#include "status_logic.h"
#include "command_logic.h"
#include "time.h"

#define TAG "LOGIC_KEY"
#define LONG_PRESS_THRESHOLD pdMS_TO_TICKS(1500)
#define KEY_SCAN_INTERVAL    pdMS_TO_TICKS(20)

static bool key1_pressed = false;
static TickType_t key1_press_start_time = 0;

static void handle_btn1_long(void) {
    ESP_LOGI(TAG, "BTN1 Long Press: BLE Scan & Connect");
    connect_state_t current_state = connect_logic_get_state();
    if (current_state >= BLE_INIT_COMPLETE) {
        connect_logic_ble_disconnect();
    }
    if (connect_logic_ble_connect(false) == -1) return;
    
    uint32_t g_device_id = 0x12345678;
    uint8_t g_mac_addr_len = 6;
    int8_t g_mac_addr[6] = {0x38, 0x34, 0x56, 0x78, 0x9A, 0xBC};
    srand((unsigned int)time(NULL));
    connect_logic_protocol_connect(g_device_id, g_mac_addr_len, g_mac_addr, 0, 0, (uint16_t)(rand() % 10000), 0);
    
    version_query_response_frame_t *ver = command_logic_get_version();
    if (ver != NULL) free(ver);
    subscript_camera_status(PUSH_MODE_PERIODIC_WITH_STATE_CHANGE, PUSH_FREQ_2HZ);
}

static void handle_btn1_short(void) {
    if (current_camera_mode == CAMERA_MODE_PHOTO || current_camera_status == CAMERA_STATUS_LIVE_STREAMING) {
        ESP_LOGI(TAG, "Starting recording/photo...");
        record_control_response_frame_t *resp = command_logic_start_record();
        if (resp) free(resp);
        else connect_logic_ble_wakeup();
    } else if (is_camera_recording()) {
        ESP_LOGI(TAG, "Stopping recording...");
        record_control_response_frame_t *resp = command_logic_stop_record();
        if (resp) free(resp);
    }
}

static void handle_btn2_short(void) {
    ESP_LOGI(TAG, "BTN2 Short Press: Quick Switch Mode");
    key_report_response_frame_t *resp = command_logic_key_report_qs();
    if (resp) free(resp);
}

static void handle_btn3_short(void) {
    ESP_LOGI(TAG, "BTN3 Short Press: Wakeup Camera");
    connect_logic_ble_wakeup();
}

static void key_scan_task(void *arg) {
    bool btn2_pressed_prev = false;
    bool btn3_pressed_prev = false;

    while (1) {
        bool btn1_low = (gpio_get_level(BOOT_KEY_GPIO) == 0);
        bool btn2_low = (gpio_get_level(BTN2_KEY_GPIO) == 0);
        bool btn3_low = (gpio_get_level(BTN3_KEY_GPIO) == 0);

        // BTN1
        if (btn1_low && !key1_pressed) {
            key1_pressed = true;
            key1_press_start_time = xTaskGetTickCount();
        } else if (btn1_low && key1_pressed) {
            TickType_t dur = xTaskGetTickCount() - key1_press_start_time;
            if (dur >= LONG_PRESS_THRESHOLD) {
                handle_btn1_long();
                key1_pressed = false; // Prevent re-trigger until release
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        } else if (!btn1_low && key1_pressed) {
            TickType_t dur = xTaskGetTickCount() - key1_press_start_time;
            if (dur < LONG_PRESS_THRESHOLD) {
                handle_btn1_short();
            }
            key1_pressed = false;
        }

        // BTN2
        if (btn2_low && !btn2_pressed_prev) {
            handle_btn2_short();
            btn2_pressed_prev = true;
        } else if (!btn2_low) {
            btn2_pressed_prev = false;
        }

        // BTN3
        if (btn3_low && !btn3_pressed_prev) {
            handle_btn3_short();
            btn3_pressed_prev = true;
        } else if (!btn3_low) {
            btn3_pressed_prev = false;
        }

        vTaskDelay(KEY_SCAN_INTERVAL);
    }
}

void key_logic_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BOOT_KEY_GPIO) | (1ULL << BTN2_KEY_GPIO) | (1ULL << BTN3_KEY_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    
    ESP_LOGI(TAG, "Keys initialized. BTN1(20), BTN2(1), BTN3(0)");
    xTaskCreate(key_scan_task, "key_scan_task", 4096, NULL, 2, NULL);
}
