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

#ifndef __GPS_LOGIC_H__
#define __GPS_LOGIC_H__

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"

#define UBYTE   uint8_t
#define UWORD   uint16_t
#define UDOUBLE uint32_t

// UART Configuration — GY-GPSV3-NEO-M8M on ESP32-C6 Super Mini
// UART 配置 — GY-GPSV3-NEO-M8M in ESP32-C6 Super Mini
//   GPS module TX -> ESP GPIO 2 (RX)
//   GPS module RX -> ESP GPIO 3 (TX)
#define UART_GPS_TXD_PIN (GPIO_NUM_3)
#define UART_GPS_RXD_PIN (GPIO_NUM_2)
#define UART_GPS_PORT    UART_NUM_1
#define RX_BUF_SIZE 800

typedef struct {
    // Time
    // 时间
    uint8_t Year;             // Year
                              // 年
    uint8_t Month;            // Month
                              // 月
    uint8_t Day;              // Day
                              // 日
    uint8_t Hour;             // Hour
                              // 时
    uint8_t Minute;           // Minute
                              // 分
    double Second;            // Second
                              // 秒

    // Position
    // 位置
    double Latitude;          // Latitude
                              // 纬度
    char Lat_Indicator;       // N/S
    double Longitude;         // Longitude
                              // 经度
    char Lon_Indicator;       // E/W

    // Other Information
    // 其他信息
    double Speed_knots;       // Ground Speed (knots)
                              // 地面速度 (节)
    double Course;            // Course (degrees)
                              // 航向 (度)
    double Altitude;          // Altitude (meters)
                              // 海拔高度 (米)
    uint8_t Num_Satellites;   // Number of Visible Satellites
                              // 可见卫星数量

    // Calculated Velocity Components
    // 计算后的速度分量
    double Velocity_North;    // Northward Velocity (m/s)
                              // 向北速度 (米/秒)
    double Velocity_East;     // Eastward Velocity (m/s)
                              // 向东速度 (米/秒)
    double Velocity_Descend;  // Descent Velocity (m/s)
                              // 下降速度 (米/秒)

    // Status
    // 状态
    uint8_t Status;          // 1: Both RMC and GGA valid, 0: Other cases
                             // 1: RMC和GGA都有效, 0: 其他情况
    uint8_t RMC_Valid;       // Whether RMC data is valid
                             // RMC 数据是否有效
    uint8_t GGA_Valid;       // Whether GGA data is valid
                             // GGA 数据是否有效
    double RMC_Latitude;     // Latitude from RMC
                             // RMC 的纬度
    double RMC_Longitude;    // Longitude from RMC
                             // RMC 的经度
    double GGA_Latitude;     // Latitude from GGA
                             // GGA 的纬度
    double GGA_Longitude;    // Longitude from GGA
                             // GGA 的经度
} GPS_Data_t;

void initSendGpsDataToCameraTask(void);

bool is_gps_found(void);

// Check if current GPS data is valid
// 检查当前 GPS 数据是否有效
bool is_current_gps_data_valid(void);

// Check if GPS is valid with a debounce for UI so it doesn't flicker
bool is_current_gps_data_valid_ui(void);

GPS_Data_t get_current_gps_data(void);

#endif