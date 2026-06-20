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
#include <stdio.h>
#include <math.h>
#include <ctype.h>

#include "gps_logic.h"
#include "connect_logic.h"
#include "command_logic.h"
#include "dji_protocol_data_structures.h"

#define TAG "LOGIC_GPS"

// Define buffer to store received data
// 定义缓冲区用于存储接收到的数据
static char buff_t[RX_BUF_SIZE]={0};

// Initialize GPS data structure
// 初始化 GPS 数据结构
static GPS_Data_t GPS_Data;

// Counter for consecutive invalid GPS readings
// GPS连续无效次数计数器
static uint8_t gps_invalid_count = 0;

/**
 * @brief Initialize GPS data structure
 *        初始化 GPS 数据结构
 * 
 * Reset all fields in GPS data structure to initial values.
 * 将 GPS 数据结构的所有字段重置为初始值。
 */
static void init_gps_data(void) {
    GPS_Data.Year = 0;
    GPS_Data.Month = 0;
    GPS_Data.Day = 0;
    GPS_Data.Hour = 0;
    GPS_Data.Minute = 0;
    GPS_Data.Second = 0.0;

    GPS_Data.Latitude = 0.0;
    GPS_Data.Lat_Indicator = 'N';
    GPS_Data.Longitude = 0.0;
    GPS_Data.Lon_Indicator = 'E';

    GPS_Data.Speed_knots = 0.0;
    GPS_Data.Course = 0.0;
    GPS_Data.Altitude = 0.0;
    GPS_Data.Num_Satellites = 0;

    GPS_Data.Velocity_North = 0.0;
    GPS_Data.Velocity_East = 0.0;
    GPS_Data.Velocity_Descend = 0.0;

    // GPS_Data.Status = 0;
    GPS_Data.RMC_Valid = 0;
    GPS_Data.GGA_Valid = 0;
    GPS_Data.RMC_Latitude = 0.0;
    GPS_Data.RMC_Longitude = 0.0;
    GPS_Data.GGA_Latitude = 0.0;
    GPS_Data.GGA_Longitude = 0.0;
}

/**
 * @brief Check if GPS signal is found
 *        检查 GPS 是否已找到信号
 * 
 * @return bool Returns true if consecutive invalid count is less than 10, false otherwise
 *              如果 GPS 连续无效次数小于10，返回 true；否则返回 false
 */
bool is_gps_found(void) {
    return (gps_invalid_count < 10);
}

/**
 * @brief Check if current GPS data is valid
 *        检查当前 GPS 数据是否有效
 * 
 * @return bool Returns true if GPS status is valid, false otherwise
 *              如果 GPS 状态为有效，返回 true；否则返回 false
 */
bool is_current_gps_data_valid(void) {
    if (GPS_Data.Status == 1) {
        return true;
    }
    return false;
}

GPS_Data_t get_current_gps_data(void) {
    return GPS_Data;
}

// Store previous altitude and time for velocity calculation
// 用于存储前一时刻的高度和时间，用于计算速度
static double Previous_Altitude = 0.0;
static double Previous_Time = 0.0;

// Used to store the previous latitude and longitude for outlier removal
// 用于存储前一时刻的纬度和经度，用于剔除异常值
static double Previous_Latitude = 0.0;
static double Previous_Longitude = 0.0;

/**
 * @brief Convert NMEA format coordinates to decimal degrees
 *        将 NMEA 格式的经纬度转换为十进制度
 * 
 * @param nmea NMEA format coordinate string
 *             NMEA 格式的经纬度字符串
 * @param direction Direction character ('N', 'S', 'E', 'W')
 *                  方向字符（'N', 'S', 'E', 'W'）
 * 
 * @return double Converted decimal degree value
 *                转换后的十进制度值
 */
double Convert_NMEA_To_Degree(const char *nmea, char direction) {
    double deg = 0.0;
    double min = 0.0;

    // Determine integer and decimal parts
    // 确定整数部分和小数部分
    const char *dot = nmea;
    while (*dot && *dot != '.') { // Find decimal point position
                                  // 找到小数点的位置
        dot++;
    }

    // Calculate integer part
    // 计算整数部分
    int int_part = 0;
    const char *ptr = nmea;
    while (ptr < dot && isdigit((unsigned char)*ptr)) {
        int_part = int_part * 10 + (*ptr - '0'); // Accumulate digits
                                                 // 累积数字
        ptr++;
    }

    // Calculate decimal part
    // 计算小数部分
    double frac_part = 0.0;
    double divisor = 10.0;
    ptr = dot + 1; // Skip decimal point
                   // 跳过小数点
    while (*ptr && isdigit((unsigned char)*ptr)) {
        frac_part += (*ptr - '0') / divisor;
        divisor *= 10.0;
        ptr++;
    }

    // Combine integer and decimal parts
    // 合并整数部分和小数部分
    deg = int_part + frac_part;

    // Separate degrees and minutes
    // 分离度和分
    min = deg - ((int)(deg / 100)) * 100;
    deg = ((int)(deg / 100)) + min / 60.0;

    // Adjust sign based on direction
    // 根据方向调整正负
    if (direction == 'S' || direction == 'W') {
        deg = -deg;
    }

    return deg;
}

/**
 * @brief Parse GNRMC sentence, e.g.: $GNRMC,074700.000,A,2234.732734,N,11356.317512,E,1.67,285.57,150125,,,A,V*03
 *        解析 GNRMC 语句，例如：$GNRMC,074700.000,A,2234.732734,N,11356.317512,E,1.67,285.57,150125,,,A,V*03
 * 
 * Parse GNRMC sentence to extract GPS data including time, status, latitude, longitude, speed, course, etc.
 * (There may be data accuracy issues that can be optimized as needed)
 * 解析 GNRMC 语句，提取 GPS 数据，包括时间、状态、纬度、经度、速度、航向等信息（可能存在数据不精确问题，可自行优化）。
 * 
 * @param sentence Input GNRMC sentence string
 *                 输入的 GNRMC 语句字符串
 */
void Parse_GNRMC(char *sentence) {
    char *token;
    char *rest = sentence;
    int field = 0;

    double temp_latitude = 0.0;
    double temp_longitude = 0.0;

    while ((token = strsep(&rest, ",")) != NULL) {
        field++;
        if (token[0] == '\0') continue; // Skip empty fields, but keep field counter incremented!
        
        switch (field) {
            case 2: {
                // 时间 hhmmss.sss
                // Time hhmmss.sss
                uint8_t hour = 0, minute = 0;
                double second = 0.0;

                // 手动解析时间
                // Manually parse time
                if (strlen(token) >= 6) {
                    const char *ptr = token;
                    hour = (ptr[0] - '0') * 10 + (ptr[1] - '0');
                    minute = (ptr[2] - '0') * 10 + (ptr[3] - '0');
                    second = atof(ptr + 4);

                    GPS_Data.Hour = hour;
                    GPS_Data.Minute = minute;
                    GPS_Data.Second = second;
                }
                break;
            }
            case 3:
                // 状态 A/V
                // Status A/V
                GPS_Data.RMC_Valid = (token[0] == 'A') ? 1 : 0;
                break;
            case 4:
                // 纬度（暂时存储）
                // Latitude (temporary storage)
                temp_latitude = Convert_NMEA_To_Degree(token, 'N');
                break;
            case 5:
                // 更新纬度方向
                // Update latitude direction
                GPS_Data.Lat_Indicator = token[0];
                GPS_Data.RMC_Latitude = (GPS_Data.Lat_Indicator == 'S') ? -temp_latitude : temp_latitude;
                break;
            case 6:
                // 经度（暂时存储）
                // Longitude (temporary storage)
                temp_longitude = Convert_NMEA_To_Degree(token, 'E');
                break;
            case 7:
                // 更新经度方向
                // Update longitude direction
                GPS_Data.Lon_Indicator = token[0];
                GPS_Data.RMC_Longitude = (GPS_Data.Lon_Indicator == 'W') ? -temp_longitude : temp_longitude;
                break;
            case 8:
                // 地面速度 (节)
                // Ground speed (knots)
                GPS_Data.Speed_knots = atof(token);
                break;
            case 9:
                // 航向 (度)
                // Course (degrees)
                GPS_Data.Course = atof(token);
                break;
            case 10: {
                // 日期 ddmmyy
                // Date ddmmyy
                uint8_t day = 0, month = 0, year = 0;

                // 手动解析日期
                // Manually parse date
                if (strlen(token) >= 6) {
                    const char *ptr = token;
                    day = (ptr[0] - '0') * 10 + (ptr[1] - '0');
                    month = (ptr[2] - '0') * 10 + (ptr[3] - '0');
                    year = (ptr[4] - '0') * 10 + (ptr[5] - '0');

                    GPS_Data.Day = day;
                    GPS_Data.Month = month;
                    GPS_Data.Year = year;
                }
                break;
            }
            default:
                break;
        }
    }

    // 计算向北和向东的速度分量 (米/秒)，节转米/秒
    // Calculate velocity components to north and east (m/s), convert from knots to m/s
    double speed_m_s = GPS_Data.Speed_knots * 0.514444;
    GPS_Data.Velocity_North = speed_m_s * cos(GPS_Data.Course * M_PI / 180.0);
    GPS_Data.Velocity_East = speed_m_s * sin(GPS_Data.Course * M_PI / 180.0);
}

/**
 * @brief Parse GNGGA sentence, e.g.: $GNGGA,074700.000,2234.732734,N,11356.317512,E,1,7,1.31,47.379,M,-2.657,M,,*65
 *        解析 GNGGA 语句，例如：$GNGGA,074700.000,2234.732734,N,11356.317512,E,1,7,1.31,47.379,M,-2.657,M,,*65
 * 
 * Parse GNGGA sentence to extract GPS data including time, latitude, longitude, number of satellites, altitude, etc.
 * (There may be data accuracy issues that can be optimized as needed)
 * 解析 GNGGA 语句，提取 GPS 数据，包括时间、纬度、经度、卫星数量、海拔高度等信息（可能存在数据不精确问题，可自行优化）。
 * 
 * @param sentence Input GNGGA sentence string
 *                 输入的 GNGGA 语句字符串
 */
void Parse_GNGGA(char *sentence) {
    char *token;
    char *rest = sentence;
    int field = 0;

    double temp_latitude = 0.0;
    double temp_longitude = 0.0;

    while ((token = strsep(&rest, ",")) != NULL) {
        field++;
        if (token[0] == '\0') continue; // Skip empty fields
        
        switch (field) {
            case 1:
                // $GNGGA
                break;
            case 2:
                // 时间 hhmmss.sss，可与GNRMC中的时间对比，确保同步
                // Time hhmmss.sss, can be compared with GNRMC time for synchronization
                break;
            case 3:
                // 纬度
                // Latitude
                temp_latitude = Convert_NMEA_To_Degree(token, 'N');
                break;
            case 4:
                // N/S
                // North/South indicator
                GPS_Data.Lat_Indicator = token[0];
                GPS_Data.GGA_Latitude = (GPS_Data.Lat_Indicator == 'S') ? -temp_latitude : temp_latitude;
                break;
            case 5:
                // 经度
                // Longitude
                temp_longitude = Convert_NMEA_To_Degree(token, 'E');
                break;
            case 6:
                // E/W
                // East/West indicator
                GPS_Data.Lon_Indicator = token[0];
                GPS_Data.GGA_Longitude = (GPS_Data.Lon_Indicator == 'W') ? -temp_longitude : temp_longitude;
                break;
            case 7:
                // 定位质量
                // Position fix quality
                if (token[0] != '\0') {
                    int quality = atoi(token);
                    GPS_Data.GGA_Valid = (quality > 0) ? 1 : 0;
                }
                break;
            case 8:
                // 可见卫星数量
                // Number of satellites in view
                GPS_Data.Num_Satellites = (uint8_t) atoi(token);
                break;
            case 9:
                // HDOP，可根据需要解析
                // HDOP, can be parsed if needed
                break;
            case 10:
                // 海拔高度 (米)
                // Altitude (meters)
                GPS_Data.Altitude = atof(token);
                // 计算下降速度 (需要上一高度和时间)
                // Calculate descent velocity (needs previous altitude and time)
                if (Previous_Time > 0.0) {
                    double current_time = GPS_Data.Hour * 3600 + GPS_Data.Minute * 60 + GPS_Data.Second;
                    double delta_time = current_time - Previous_Time;
                    
                    // 处理跨天情况
                    // Handle day crossover
                    if (delta_time < -43200) {  // 如果时间差小于-12小时，说明跨天了
                                                // If time difference is less than -12 hours, day has changed
                        delta_time += 86400;    // 加上24小时
                                                // Add 24 hours
                    } else if (delta_time > 43200) {  // 如果时间差大于12小时，说明是前一天的数据
                                                      // If time difference is more than 12 hours, it's previous day's data
                        delta_time -= 86400;
                    }
                    
                    if (delta_time > 0 && delta_time < 10) {  // 只处理合理的时间差（比如小于10秒）
                                                              // Only process reasonable time differences (e.g., less than 10 seconds)
                        double delta_altitude = GPS_Data.Altitude - Previous_Altitude;
                        // 过滤异常值（比如高度差太大）
                        // Filter abnormal values (e.g., too large altitude differences)
                        if (fabs(delta_altitude) < 100) {  // 假设最大垂直速度不超过100m/s
                                                           // Assume maximum vertical speed doesn't exceed 100m/s
                            GPS_Data.Velocity_Descend = -delta_altitude / delta_time;  // 注意符号：上升为负，下降为正
                                                                                       // Note: negative for ascent, positive for descent
                        }
                    }
                }
                Previous_Altitude = GPS_Data.Altitude;
                Previous_Time = GPS_Data.Hour * 3600 + GPS_Data.Minute * 60 + GPS_Data.Second;
                break;
            // 其他字段可根据需要解析
            // Other fields can be parsed as needed
            default:
                break;
        }
    }
}

void Process_Parsed_GPS_Data() {
    if (GPS_Data.RMC_Valid && GPS_Data.GGA_Valid) {
        GPS_Data.Status = 1;
        gps_invalid_count = 0;  // 重置计数器
                                // Reset counter
        // 计算平均值
        // Calculate average
        GPS_Data.Latitude = (GPS_Data.RMC_Latitude + GPS_Data.GGA_Latitude) / 2.0;
        GPS_Data.Longitude = (GPS_Data.RMC_Longitude + GPS_Data.GGA_Longitude) / 2.0;

        // 初始化前一时刻的经纬度
        if (Previous_Latitude == 0.0 && Previous_Longitude == 0.0) {
            Previous_Latitude = GPS_Data.Latitude;
            Previous_Longitude = GPS_Data.Longitude;
        }

        // 与前一时刻的纬度和经度做对比
        // Compare with previous latitude and longitude
        if (fabs(GPS_Data.Latitude - Previous_Latitude) > 0.009 || fabs(GPS_Data.Longitude - Previous_Longitude) > 0.0127) {
            // 超过阈值，认为定位异常
            GPS_Data.Status = 0;
        }

        // 更新前一时刻的经纬度
        // Update the previous latitude and longitude
        Previous_Latitude = GPS_Data.Latitude;
        Previous_Longitude = GPS_Data.Longitude;
    } else {
        GPS_Data.Status = 0;
        if (gps_invalid_count < UINT8_MAX) {  // 防止溢出
            gps_invalid_count++;
        }
    }
}

/**
 * @brief 打印当前的 GPS 数据
 *        Print current GPS data
 * 
 * 将当前的 GPS 数据以日志的形式输出。
 * Output current GPS data in log format.
 */
void print_gps_data() {
    ESP_LOGI(TAG, 
        "GPS Data: Time=%02d:%02d:%06.3f, Date=%02d-%02d-20%02d, "
        "Lat=%f %c, Lon=%f %c, Speed=%.2f knots, Course=%.2f deg, "
        "Altitude=%.2f m, Satellites=%d, V_North=%.2f m/s, V_East=%.2f m/s, V_Descend=%.2f m/s",
        GPS_Data.Hour, GPS_Data.Minute, GPS_Data.Second,
        GPS_Data.Day, GPS_Data.Month, GPS_Data.Year,
        GPS_Data.Latitude, GPS_Data.Lat_Indicator,
        GPS_Data.Longitude, GPS_Data.Lon_Indicator,
        GPS_Data.Speed_knots, GPS_Data.Course,
        GPS_Data.Altitude, GPS_Data.Num_Satellites,
        GPS_Data.Velocity_North, GPS_Data.Velocity_East,
        GPS_Data.Velocity_Descend
    );
}

/**
 * @brief 推送 GPS 数据到相机
 *        Push GPS data to camera
 * 
 * 将当前的 GPS 数据转换为指定格式，并通过命令逻辑推送到相机。
 * Convert current GPS data to specified format and push to camera through command logic.
 */
void gps_push_data() {
    // 时间转换
    // Time conversion
    int32_t year_month_day = (GPS_Data.Year + 2000) * 10000 + GPS_Data.Month * 100 + GPS_Data.Day;
    int32_t hour_minute_second = (GPS_Data.Hour + 8) * 10000 + GPS_Data.Minute * 100 + (int32_t)GPS_Data.Second;

    // 经纬度转换
    // Longitude and latitude conversion
    int32_t gps_longitude = (int32_t)(GPS_Data.Longitude * 1e7);
    int32_t gps_latitude = (int32_t)(GPS_Data.Latitude * 1e7);

    // 高度转换
    // Height conversion
    int32_t height = (int32_t)(GPS_Data.Altitude * 1000);    // 单位 mm
                                                             // Unit: mm

    // 速度转换
    // Speed conversion
    float speed_to_north = GPS_Data.Velocity_North * 100;    // m/s 转换为 cm/s
                                                             // Convert m/s to cm/s
    float speed_to_east = GPS_Data.Velocity_East * 100;      // m/s 转换为 cm/s
                                                             // Convert m/s to cm/s
    float speed_to_wnward = GPS_Data.Velocity_Descend * 100; // m/s 转换为 cm/s
                                                             // Convert m/s to cm/s

    // 卫星数量
    // Number of satellites
    uint32_t satellite_number = GPS_Data.Num_Satellites;

    // 创建 GPS 数据帧
    // Create GPS data frame
    gps_data_push_command_frame gps_frame = {
        .year_month_day = year_month_day,
        .hour_minute_second = hour_minute_second,
        .gps_longitude = gps_longitude,
        .gps_latitude = gps_latitude,
        .height = height,
        .speed_to_north = speed_to_north,
        .speed_to_east = speed_to_east,
        .speed_to_wnward = speed_to_wnward,
        .vertical_accuracy = 1000,    // 垂直默认精度为 1000 mm
                                      // Default vertical accuracy is 1000 mm
        .horizontal_accuracy = 1000,  // 水平精度为 1000 mm
                                      // Horizontal accuracy is 1000 mm
        .speed_accuracy = 10,         // 速度精度为 10 cm/s
                                      // Speed accuracy is 10 cm/s
        .satellite_number = satellite_number
    };

    // 推送 GPS 数据到相机，无应答，默认返回 NULL
    // Push GPS data to camera, no response, returns NULL by default
    gps_data_push_response_frame *response = command_logic_push_gps_data(&gps_frame);
    if (response != NULL) {
        free(response);
    }
}

/**
 * @brief 发送 UBX 命令
 *        Send UBX command
 */
static void send_ubx_cmd(uint8_t msg_class, uint8_t msg_id, const uint8_t *payload, uint16_t len) {
    uint8_t header[6] = {
        0xB5, 0x62, 
        msg_class, msg_id, 
        (uint8_t)(len & 0xFF), (uint8_t)(len >> 8)
    };
    uint8_t ck_a = 0, ck_b = 0;
    for (int i = 2; i < 6; i++) {
        ck_a += header[i];
        ck_b += ck_a;
    }
    for (int i = 0; i < len; i++) {
        ck_a += payload[i];
        ck_b += ck_a;
    }
    uint8_t tail[2] = {ck_a, ck_b};
    
    uart_write_bytes(UART_GPS_PORT, (const char*)header, 6);
    if (len > 0) {
        uart_write_bytes(UART_GPS_PORT, (const char*)payload, len);
    }
    uart_write_bytes(UART_GPS_PORT, (const char*)tail, 2);
}

/**
 * @brief 配置 GPS 模块 (波特率 115200 和 10Hz 刷新率)
 *        Configure GPS module (Baud 115200, 10Hz)
 */
static void configure_gps_module() {
    ESP_LOGI(TAG, "Configuring GPS Module to 115200 baud and 10Hz...");

    // Send 115200 baud command at current baud (assuming 9600 default)
    uint8_t cfg_prt_payload[20] = {
        0x01, 0x00, 0x00, 0x00, 
        0xD0, 0x08, 0x00, 0x00, 
        0x00, 0xC2, 0x01, 0x00, // 115200 = 0x0001C200
        0x07, 0x00, 
        0x03, 0x00, 
        0x00, 0x00, 
        0x00, 0x00
    };
    send_ubx_cmd(0x06, 0x00, cfg_prt_payload, 20);

    // Wait for the message to be transmitted completely
    vTaskDelay(pdMS_TO_TICKS(100));

    // Reconfigure ESP32 UART to 115200
    uart_set_baudrate(UART_GPS_PORT, 115200);
    uart_flush(UART_GPS_PORT);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Send the baud command again at 115200 just in case the module was already at 115200
    send_ubx_cmd(0x06, 0x00, cfg_prt_payload, 20);
    vTaskDelay(pdMS_TO_TICKS(50));

    // Now configure 10Hz update rate
    uint8_t cfg_rate_payload[6] = {
        0x64, 0x00, // measRate = 100 ms (10Hz)
        0x01, 0x00, // navRate = 1
        0x01, 0x00  // timeRef = 1 (GPS)
    };
    send_ubx_cmd(0x06, 0x08, cfg_rate_payload, 6);
    vTaskDelay(pdMS_TO_TICKS(50));

    ESP_LOGI(TAG, "GPS Module configured for 10Hz!");
}

/**
 * @brief 初始化 GPS UART
 *        Initialize GPS UART
 * 
 * 配置并初始化 GPS UART，用于接收 GPS 数据。
 * Configure and initialize GPS UART for receiving GPS data.
 */
static void initUartGps(void)
{
    // 初始以 9600 波特率连接，然后发送命令切换到 115200
    // Initially connect at 9600, then send UBX to switch to 115200
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UART_GPS_PORT, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_GPS_PORT, &uart_config);
    uart_set_pin(UART_GPS_PORT, UART_GPS_TXD_PIN, UART_GPS_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // 自动配置 GPS 模块
    // Automatically configure GPS module
    configure_gps_module();
}

/**
 * @brief GPS 数据接收任务
 *        GPS data receiving task
 * 
 * 从 GPS UART 端口读取数据，按行缓冲并解析 NMEA 数据。
 * Read data from GPS UART port, buffer line-by-line and parse NMEA data.
 * 
 * @param arg 任务参数
 */
static void rx_task_GPS(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK_GPS";
    uint8_t data[128];
    char line_buffer[256];
    int line_pos = 0;

    init_gps_data();

    while (1) {
        const int rxBytes = uart_read_bytes(UART_GPS_PORT, data, sizeof(data), 20 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            // 给看门狗喂狗的机会
            // Give watchdog a chance to reset
            vTaskDelay(pdMS_TO_TICKS(5));

            // Process byte by byte to extract complete lines
            for (int i = 0; i < rxBytes; i++) {
                char c = (char)data[i];
                if (c == '\n') {
                    line_buffer[line_pos] = '\0';
                    
                    // A complete line has been received, parse it
                    if (strncmp(line_buffer, "$GNRMC", 6) == 0 || strncmp(line_buffer, "$GPRMC", 6) == 0) {
                        Parse_GNRMC(line_buffer);
                    } else if (strncmp(line_buffer, "$GNGGA", 6) == 0 || strncmp(line_buffer, "$GPGGA", 6) == 0) {
                        Parse_GNGGA(line_buffer);
                        // Typically GGA comes after RMC in a 1Hz burst, so process state here
                        Process_Parsed_GPS_Data();
                    }

                    line_pos = 0; // Reset line buffer for next line
                } else if (c != '\r') {
                    if (line_pos < sizeof(line_buffer) - 1) {
                        line_buffer[line_pos++] = c;
                    }
                }
            }

            static uint32_t log_counter = 0;
            if (log_counter++ % 20 == 0) { 
                if (is_current_gps_data_valid()) {
                    print_gps_data();
                } else {
                    ESP_LOGI(RX_TASK_TAG, "GPS is NOT locked yet. Searching for satellites...");
                }
            }

            if(connect_logic_get_state() == PROTOCOL_CONNECTED && is_current_gps_data_valid()){
                gps_push_data();
            }
        }
        // 如果没有数据读取，休眠一小段时间，避免任务占用 CPU
        // If no data is read, sleep for a short time to avoid CPU occupation
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief 初始化并启动 GPS 数据接收任务
 *        Initialize and start GPS data receiving task
 * 
 * 初始化 GPS UART 和相关任务，以定期接收 GPS 数据。
 * Initialize GPS UART and related tasks to periodically receive GPS data.
 */
void initSendGpsDataToCameraTask(void) {
    initUartGps(); // Starts at 9600 baud
    
    // We will intentionally NOT change the baudrate or update rate yet.
    // Let's run it at the default 9600 baud, 1Hz rate.
    // This will prove if the module can get a lock without being overloaded
    // or suffering from a baudrate mismatch.
    ESP_LOGI(TAG, "GPS UART started at default 9600 baud, 1Hz. Waiting for lock...");

    xTaskCreate(rx_task_GPS, "uart_rx_task_GPS", 1024 * 4, NULL, 0, NULL);
    ESP_LOGI(TAG, "uart_rx_task_GPS running\n");
}
