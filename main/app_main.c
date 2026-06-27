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

#include "freertos/FreeRTOS.h"

#include "connect_logic.h"
#include "gps_logic.h"
#include "key_logic.h"
#include "light_logic.h"
#include "data.h"
#include "status_logic.h"
#include "web_logic.h"
#include "oled_logic.h"

/**
 * @brief Main application function, performs initialization and task loop
 * 应用主函数，执行初始化和任务循环
 *
 * This function initializes the RGB light, GPS module, Bluetooth connection, 
 * and key logic in sequence, and starts a loop task for periodic operations.
 * 
 * 在此函数中，依次初始化氛围灯、GPS模块、蓝牙模块和按键逻辑，
 * 并启动一个循环任务，周期性进行操作。
 */
void app_main(void) {

    int res = 0;

    /* Initialize Web Server and Wi-Fi AP */
    web_logic_init();

    /* Initialize OLED display to show boot status immediately */
    oled_logic_init();

    /* Initialize status/LED logger */
    res = init_light_logic();
    if (res != 0) {
        return;
    }

    /* Initialize GPS module (starts streaming to camera once connected) */
    initSendGpsDataToCameraTask();

    vTaskDelay(pdMS_TO_TICKS(2000));

    /* Initialize data layer (needed for BLE protocol) */
    data_init();
    data_register_status_update_callback(update_camera_state_handler);
    data_register_new_status_update_callback(update_new_camera_state_handler);

    /* Initialize Bluetooth stack */
    res = connect_logic_ble_init();
    if (res != 0) {
        return;
    }

    /* Initialize key logic (short/double/long press on GPIO 20) */
    key_logic_init();

    /* Auto-connect on boot — no button press needed.
     * The reconnect task will keep retrying until the camera comes online,
     * and will automatically reconnect any time the camera power-cycles. */
    connect_logic_start_auto_connect();

    /* Idle loop */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
