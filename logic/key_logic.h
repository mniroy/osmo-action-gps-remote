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
 * If you receive this source code without DJI's authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 */

#ifndef KEY_LOGIC_H
#define KEY_LOGIC_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

// Momentary switch connected to GPIO 20 on ESP32-C6 Super Mini
// 瞬动开关连接到 ESP32-C6 Super Mini 的 GPIO 20
#define BOOT_KEY_GPIO   GPIO_NUM_20

// Key Events
// 按键事件
typedef enum {
    KEY_EVENT_NONE = 0,    // No event / 无事件
    KEY_EVENT_SINGLE,      // Single click: record/shutter toggle / 单击：录制/快门切换
    KEY_EVENT_DOUBLE,      // Double click: quick switch mode / 双击：快速切换模式
    KEY_EVENT_LONG_PRESS,  // Long press: BLE scan & connect / 长按：BLE扫描并连接
    KEY_EVENT_ERROR        // Error event / 错误事件
} key_event_t;

void key_logic_init(void);

key_event_t key_logic_get_event(void);

#endif