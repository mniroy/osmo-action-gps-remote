/* SPDX-License-Identifier: MIT */
/*
 * Copyright (C) 2025 SZ DJI Technology Co., Ltd.
 *  
 * All information contained herein is, and remains, the property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI’s authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 */

#include <string.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_random.h"

#include "ble.h"
#include "data.h"
#include "enums_logic.h"
#include "connect_logic.h"
#include "command_logic.h"
#include "status_logic.h"
#include "dji_protocol_data_structures.h"

#define TAG "LOGIC_CONNECT"

static connect_state_t connect_state = BLE_NOT_INIT;

/* -----------------------------------------------------------------------
 * Auto-reconnect state
 * Set to true when the camera drops unexpectedly (power cycle, sleep, etc.)
 * The reconnect_task will keep retrying until the camera comes back.
 * Set to false when the user intentionally disconnects (long press).
 * ----------------------------------------------------------------------- */
static volatile bool s_should_auto_reconnect = false;
static TaskHandle_t s_reconnect_task_handle = NULL;

/* Forward declarations */
static void reconnect_task(void *arg);
static int do_full_connect_sequence(bool is_reconnecting);

/**
 * @brief Get current connection state
 *        获取当前连接状态
 * 
 * @return connect_state_t Returns current connection state
 *                        返回当前的连接状态
 */
connect_state_t connect_logic_get_state(void) {
    return connect_state;
}

/**
 * @brief Handle camera disconnection (callback function)
 *        处理相机断开连接（回调函数）
 * 
 * Perform operations according to current connection state and reset connection state to BLE initialization complete (BLE_INIT_COMPLETE).
 * 根据当前连接状态进行相应的操作，并将连接状态重置为 BLE 初始化完成（BLE_INIT_COMPLETE）。
 */
void receive_camera_disconnect_handler() {
    switch (connect_state) {
        case BLE_SEARCHING:
            break;
        case BLE_INIT_COMPLETE:
            ESP_LOGI(TAG, "Already in DISCONNECTED state.");
            break;
        case BLE_DISCONNECTING: {
            /* Intentional disconnect (user long-pressed) — do NOT auto-reconnect */
            ESP_LOGI(TAG, "Normal (intentional) disconnection.");
            s_should_auto_reconnect = false;
            connect_state = BLE_INIT_COMPLETE;
            camera_status_initialized = false;
            break;
        }
        case BLE_CONNECTED:
        case PROTOCOL_CONNECTED:
        default: {
            /* Unexpected disconnect — camera may have powered off or gone to sleep.
             * Signal the reconnect task to start retrying. */
            ESP_LOGW(TAG, "Unexpected disconnect from state %d — will auto-reconnect", connect_state);
            connect_state = BLE_INIT_COMPLETE;
            camera_status_initialized = false;
            s_should_auto_reconnect = true;
            /* Notify the reconnect task if it is suspended */
            if (s_reconnect_task_handle != NULL) {
                xTaskNotifyGive(s_reconnect_task_handle);
            }
            break;
        }
    }
}

/**
 * @brief Background task — retries the full connect+protocol sequence
 *        until the camera comes back, then parks itself.
 *
 * Strategy:
 *   - Wait 5 s after disconnect (camera may be rebooting)
 *   - Try BLE scan+connect (up to 30 s per attempt)
 *   - On BLE success, run protocol handshake
 *   - On protocol success, subscribe and go back to sleep
 *   - On any failure, wait RETRY_INTERVAL_MS and try again
 *   - If intentional disconnect clears the flag, stop immediately
 */
#define RETRY_INTERVAL_MS 10000   /* 10 s between attempts */
#define REBOOT_GRACE_MS   5000    /* wait 5 s after disconnect before first try */

static int do_full_connect_sequence(bool is_reconnecting) {
    /* Use NVS saved MAC for auto-reconnect, or fresh scan for long press. */
    int ret = connect_logic_ble_connect(is_reconnecting);
    if (ret != 0) {
        return ret;
    }

    /* Protocol handshake */
    uint32_t g_device_id   = 0x12345678;
    uint8_t  g_mac_addr_len = 6;
    int8_t   g_mac_addr[6] = {0x38, 0x34, 0x56, 0x78, 0x9A, 0xBC};
    uint32_t g_fw_version  = 0x00;
    uint8_t  g_verify_mode = 0;
    uint16_t g_verify_data = (uint16_t)(esp_random() % 10000);
    uint8_t  g_camera_reserved = 0;

    if (connect_logic_protocol_connect(
            g_device_id, g_mac_addr_len, g_mac_addr,
            g_fw_version, g_verify_mode, g_verify_data, g_camera_reserved) != 0) {
        return -1;
    }

    /* Version query */
    version_query_response_frame_t *ver = command_logic_get_version();
    if (ver != NULL) free(ver);

    /* Subscribe camera status */
    if (subscript_camera_status(PUSH_MODE_PERIODIC_WITH_STATE_CHANGE, PUSH_FREQ_2HZ) != 0) {
        ESP_LOGW(TAG, "subscribe camera status failed — continuing anyway");
    }
    return 0;
}

static void reconnect_task(void *arg) {
    ESP_LOGI(TAG, "Auto-reconnect task started");
    while (1) {
        /* Park here until we're signalled to reconnect */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (!s_should_auto_reconnect) {
            continue;   /* spurious wake, go back to sleep */
        }

        ESP_LOGI(TAG, "Camera disconnected — waiting %d ms before reconnect attempt", REBOOT_GRACE_MS);
        vTaskDelay(pdMS_TO_TICKS(REBOOT_GRACE_MS));

        int attempt = 0;
        while (s_should_auto_reconnect) {
            attempt++;
            ESP_LOGI(TAG, "Auto-reconnect attempt #%d...", attempt);

            int res = do_full_connect_sequence(true);
            if (res == 0) {
                ESP_LOGI(TAG, "Auto-reconnect successful after %d attempt(s)", attempt);
                s_should_auto_reconnect = false;
                break;
            } else if (res == -2) {
                ESP_LOGI(TAG, "No saved camera found in NVS, stopping auto-reconnect loop until user pairs via long press.");
                s_should_auto_reconnect = false;
                break;
            }

            ESP_LOGW(TAG, "Attempt #%d failed — retrying in %d ms", attempt, RETRY_INTERVAL_MS);
            /* Wait before next attempt, but wake early if state changes */
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(RETRY_INTERVAL_MS));
        }
    }
}

/**
 * @brief Initialize BLE connection
 *        初始化 BLE 连接
 * 
 * Initialize BLE and set state to BLE initialization complete (BLE_INIT_COMPLETE).
 * 初始化 BLE，并设置状态为 BLE 初始化完成（BLE_INIT_COMPLETE）。
 * 
 * @return int Returns 0 on success, -1 on failure
 *             成功返回 0，失败返回 -1
 */
int connect_logic_ble_init() {
    esp_err_t ret;

    /* 1. Initialize BLE (specify target device name to search and connect)
     * 1. 初始化 BLE（指定要搜索并连接的目标设备名） */
    ret = ble_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BLE, error: %s", esp_err_to_name(ret));
        return -1;
    }

    connect_state = BLE_INIT_COMPLETE;

    /* Spawn the persistent auto-reconnect background task */
    xTaskCreate(reconnect_task, "reconnect_task", 4096, NULL, 3, &s_reconnect_task_handle);

    ESP_LOGI(TAG, "BLE init successfully");
    return 0;
}

/**
 * @brief Trigger initial auto-connect on boot (no button press required).
 *        Called from app_main after BLE init. Reuses the reconnect task so
 *        the same code path handles first boot AND all future reconnects.
 */
void connect_logic_start_auto_connect(void) {
    ESP_LOGI(TAG, "Boot: starting initial auto-connect to camera...");
    s_should_auto_reconnect = true;
    if (s_reconnect_task_handle != NULL) {
        xTaskNotifyGive(s_reconnect_task_handle);
    }
}

/**
 * @brief Connect to BLE device
 *        连接到 BLE 设备
 * 
 * Execute the following steps: set callbacks, start scanning and attempt connection, wait for connection completion and characteristic handle discovery.
 * 执行以下步骤：设置回调、启动扫描并尝试连接、等待连接完成和特征句柄发现。
 * 
 * If connection fails, returns error and resets connection state.
 * 如果连接失败，会返回错误并重置连接状态。
 * 
 * @return int Returns 0 on success, -1 on failure
 *             成功返回 0，失败返回 -1
 */
int connect_logic_ble_connect(bool is_reconnecting) {
    connect_state = BLE_SEARCHING;

    esp_err_t ret;

    /* 1. Set a global Notify callback for receiving remote data and protocol parsing */
    /* 设置一个全局 Notify 回调，用于接收远端数据并进行协议解析 */
    ble_set_notify_callback(receive_camera_notify_handler);
    ble_set_state_callback(receive_camera_disconnect_handler);

    /* 2. Start scanning and attempt connection */
    /* 开始扫描并尝试连接 */
    ble_set_reconnecting(is_reconnecting);
    ret = ble_start_scanning_and_connect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start scanning and connect, error: 0x%x", ret);
        connect_state = BLE_INIT_COMPLETE;
        return (is_reconnecting && ret == ESP_FAIL) ? -2 : -1;
    }

    /* 3. Wait up to 30 seconds to ensure BLE connection success */
    /* 等待最多 30 秒以确保 BLE 连接成功 */
    ESP_LOGI(TAG, "Waiting up to 30s for BLE to connect...");
    bool connected = false;
    for (int i = 0; i < 300; i++) { // 300 * 100ms = 30s
        if (s_ble_profile.connection_status.is_connected) {
            ESP_LOGI(TAG, "BLE connected successfully");
            connected = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (!connected) {
        ESP_LOGW(TAG, "BLE connection timed out");
        connect_state = BLE_INIT_COMPLETE;
        return -1;
    }

    /* 4. Wait for characteristic handle discovery completion (up to 30 seconds) */
    /* 等待特征句柄查找完成（最多等待30秒） */
    ESP_LOGI(TAG, "Waiting up to 30s for characteristic handles discovery...");
    bool handles_found = false;
    for (int i = 0; i < 300; i++) { // 300 * 100ms = 30s
        if (s_ble_profile.handle_discovery.notify_char_handle_found && 
            s_ble_profile.handle_discovery.write_char_handle_found) {
            ESP_LOGI(TAG, "Required characteristic handles found");
            handles_found = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (!handles_found) {
        ESP_LOGW(TAG, "Characteristic handles not found within timeout");
        connect_state = BLE_INIT_COMPLETE;
        return -1;
    }

    /* 5. Register notification */
    /* 注册通知 */
    ret = ble_register_notify(s_ble_profile.conn_id, s_ble_profile.notify_char_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register notify, error: %s", esp_err_to_name(ret));
        connect_state = BLE_INIT_COMPLETE;
        return -1;
    }

    // Update state to BLE connected
    // 更新状态为 BLE 已连接
    connect_state = BLE_CONNECTED;

    // Delay RGB light display
    // 延迟展示氛围灯
    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP_LOGI(TAG, "BLE connect successfully");
    return 0;
}

/**
 * @brief Disconnect BLE connection
 *        断开 BLE 连接
 * 
 * Attempt to disconnect from BLE device.
 * 尝试断开与 BLE 设备的连接。
 * 
 * @return int Returns 0 on success, -1 on failure
 *             成功返回 0，失败返回 -1
 */
int connect_logic_ble_disconnect(void) {
    connect_state_t old_state = connect_state;
    connect_state = BLE_DISCONNECTING;
    
    ESP_LOGI(TAG, "Disconnecting camera...");

    // Call BLE layer's ble_disconnect function
    // 调用 BLE 层的 ble_disconnect 函数
    esp_err_t ret = ble_disconnect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disconnect camera, BLE error: %s", esp_err_to_name(ret));
        connect_state = old_state;
        return -1;
    }

    ESP_LOGI(TAG, "Camera disconnected successfully");
    return 0;
}

/**
 * @brief Protocol connection function
 *        协议连接函数
 * 
 * This function is responsible for establishing protocol connection, including the following steps:
 * 该函数负责建立协议连接，包含以下步骤：
 * 
 * 1. Send connection request command to camera.
 *    向相机发送连接请求命令。
 * 2. Wait for camera's response and verify.
 *    等待相机的响应并进行验证。
 * 3. Send connection response according to camera's returned command.
 *    根据相机返回的命令发送连接应答。
 * 4. Set connection state to protocol connected.
 *    设置连接状态为协议连接。
 * 
 * @param device_id Device ID
 *                  设备ID
 * @param mac_addr_len MAC address length
 *                     MAC地址长度
 * @param mac_addr Pointer to MAC address
 *                 指向MAC地址的指针
 * @param fw_version Firmware version
 *                   固件版本
 * @param verify_mode Verification mode
 *                    验证模式
 * @param verify_data Verification data
 *                    验证数据
 * @param camera_reserved Camera reserved field
 *                        相机保留字段
 * 
 * @return int Returns 0 on success, -1 on failure
 *             成功返回 0，失败返回 -1
 */
int connect_logic_protocol_connect(uint32_t device_id, uint8_t mac_addr_len, const int8_t *mac_addr,
                                   uint32_t fw_version, uint8_t verify_mode, uint16_t verify_data,
                                   uint8_t camera_reserved) {
    ESP_LOGI(TAG, "%s: Starting protocol connection", __FUNCTION__);
    uint16_t seq = generate_seq();

    // Construct connection request command frame
    // 构造连接请求命令帧
    connection_request_command_frame connection_request = {
        .device_id = device_id,
        .mac_addr_len = mac_addr_len,
        .fw_version = fw_version,
        .verify_mode = verify_mode,
        .verify_data = verify_data,
    };
    memcpy(connection_request.mac_addr, mac_addr, mac_addr_len);


    // STEP1: Send connection request command to camera
    // 相机发送连接请求命令
    ESP_LOGI(TAG, "Sending connection request to camera...");
    CommandResult result = send_command(0x00, 0x19, CMD_WAIT_RESULT, &connection_request, seq, 1000);

    /**** Connection issue: camera may return either response frame or command frame ****/
    /****************** 连接问题，这里相机可能返回 应答帧 也可能返回 命令帧 ******************/

    if (result.structure == NULL) {
        // If a command frame is sent, execute this block of code
        // 如果发命令帧，走这里的代码

        // Directly call data_wait_for_result_by_cmd(0x00, 0x19, 30000, &received_seq, &parse_result, &parse_result_length);
        // 这里直接去 esp_err_t ret = data_wait_for_result_by_cmd(0x00, 0x19, 30000, &received_seq, &parse_result, &parse_result_length);
        
        // If != OK, it means no message was received, timeout occurred
        // 如果 != OK 说明确实没有收到消息，超时
        
        // Otherwise, GOTO wait_for_camera_command label
        // 否则 GOTO 到 wait_for_camera_command 标识
        void *parse_result = NULL;
        size_t parse_result_length = 0;
        uint16_t received_seq = 0;
        esp_err_t ret = data_wait_for_result_by_cmd(0x00, 0x19, 1000, &received_seq, &parse_result, &parse_result_length);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Timeout or error waiting for camera connection command, GOTO Failed.");
            connect_logic_ble_disconnect();
            return -1;
        } else {
            // If data is received, skip parsing camera response and directly enter STEP3
            // 如果能收到数据，跳过解析相机返回响应，直接进入STEP3
            goto wait_for_camera_command;
        }
    }

    // STEP2: Parse the response returned from camera
    // 解析相机返回的响应
    connection_request_response_frame *response = (connection_request_response_frame *)result.structure;
    if (response->ret_code != 0) {
        ESP_LOGE(TAG, "Connection handshake failed: unexpected response from camera, ret_code: %d", response->ret_code);
        free(response);
        connect_logic_ble_disconnect();
        return -1;
    }

    ESP_LOGI(TAG, "Handshake successful, waiting for the camera to actively send the connection command frame...");
    free(response);

    // STEP3: Wait for camera to send connection request
    // 等待相机主动发送连接请求
wait_for_camera_command:
    void *parse_result = NULL;
    size_t parse_result_length = 0;
    uint16_t received_seq = 0;
    esp_err_t ret = data_wait_for_result_by_cmd(0x00, 0x19, 30000, &received_seq, &parse_result, &parse_result_length);

    if (ret != ESP_OK || parse_result == NULL) {
        ESP_LOGE(TAG, "Timeout or error waiting for camera connection command");
        connect_logic_ble_disconnect();
        return -1;
    }

    // Parse the connection request command sent by camera
    // 解析相机发送的连接请求命令
    connection_request_command_frame *camera_request = (connection_request_command_frame *)parse_result;

    if (camera_request->verify_mode != 2) {
        ESP_LOGE(TAG, "Unexpected verify_mode from camera: %d", camera_request->verify_mode);
        free(parse_result);
        connect_logic_ble_disconnect();
        return -1;
    }

    if (camera_request->verify_data == 0) {
        ESP_LOGI(TAG, "Camera approved the connection, sending response...");

        // Construct connection response frame
        // 构造连接应答帧
        connection_request_response_frame connection_response = {
            .device_id = device_id,
            .ret_code = 0,
        };
        memset(connection_response.reserved, 0, sizeof(connection_response.reserved));
        connection_response.reserved[0] = camera_reserved;

        ESP_LOGI(TAG, "Constructed connection response, sending...");

        // STEP4: Send connection response frame
        // 发送连接应答帧
        send_command(0x00, 0x19, ACK_NO_RESPONSE, &connection_response, received_seq, 5000);

        // Set connection state to protocol connected
        // 设置连接状态为协议连接
        connect_state = PROTOCOL_CONNECTED;

        ESP_LOGI(TAG, "Connection successfully established with camera.");
        ble_save_connected_device_to_nvs();
        free(parse_result);
        return 0;
    } else {
        ESP_LOGW(TAG, "Camera rejected the connection, closing Bluetooth link...");
        free(parse_result);
        connect_logic_ble_disconnect();
        return -1;
    }
}

int connect_logic_ble_wakeup(void) {
    ESP_LOGI(TAG, "Attempting to wake up camera via BLE advertising");

    esp_err_t ret = ble_start_advertising();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start BLE advertising: %s", esp_err_to_name(ret));
        return -1;
    }

    ESP_LOGI(TAG, "BLE advertising started, attempting to wake up camera");
    return 0;
}
