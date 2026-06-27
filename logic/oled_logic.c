#include "oled_logic.h"
#include "oled_font.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>

#include "connect_logic.h"
#include "gps_logic.h"
#include "status_logic.h"
#include "command_logic.h" // For current_camera_mode etc.

#define TAG "OLED"

static spi_device_handle_t spi;
static uint8_t oled_buffer[OLED_WIDTH * OLED_HEIGHT / 8];

static void oled_send_cmd(uint8_t cmd) {
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = &cmd;
    gpio_set_level(OLED_PIN_DC, 0); // Command mode
    ret = spi_device_polling_transmit(spi, &t);
    assert(ret == ESP_OK);
}

static void oled_send_data(const uint8_t *data, int len) {
    esp_err_t ret;
    spi_transaction_t t;
    if (len == 0) return;
    memset(&t, 0, sizeof(t));
    t.length = len * 8;
    t.tx_buffer = data;
    gpio_set_level(OLED_PIN_DC, 1); // Data mode
    ret = spi_device_polling_transmit(spi, &t);
    assert(ret == ESP_OK);
}

void oled_clear(void) {
    memset(oled_buffer, 0, sizeof(oled_buffer));
}

void oled_update(void) {
    for (int i = 0; i < 8; i++) {
        oled_send_cmd(0xB0 + i); // Page address
        oled_send_cmd(0x00);     // Lower column address
        oled_send_cmd(0x10);     // Higher column address
        oled_send_data(&oled_buffer[OLED_WIDTH * i], OLED_WIDTH);
    }
}

void oled_draw_string(int x, int y_page, const char *str) {
    if (y_page > 7) return;
    while (*str) {
        if (x >= OLED_WIDTH - 5) break;
        char c = *str;
        if (c < 32 || c > 126) c = 32;
        int font_idx = (c - 32) * 5;
        
        for (int i = 0; i < 5; i++) {
            oled_buffer[y_page * OLED_WIDTH + x + i] = font5x7[font_idx + i];
        }
        x += 6; // 5 pixels wide + 1 pixel space
        str++;
    }
}

static void oled_hardware_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << OLED_PIN_DC) | (1ULL << OLED_PIN_RST),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    spi_bus_config_t buscfg = {
        .miso_io_num = -1,
        .mosi_io_num = OLED_PIN_MOSI,
        .sclk_io_num = OLED_PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 1024
    };
    
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000, // 1 MHz (much safer for all OLEDs)
        .mode = 0,
        .spics_io_num = OLED_PIN_CS,
        .queue_size = 7
    };
    
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi));

    // Reset OLED
    gpio_set_level(OLED_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(OLED_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(50));

    // SSD1306 Init Sequence
    oled_send_cmd(0xAE); // Display OFF
    oled_send_cmd(0x20); // Set Memory Addressing Mode
    oled_send_cmd(0x00); // Horizontal Addressing Mode
    oled_send_cmd(0xB0); // Set Page Start Address for Page Addressing Mode
    oled_send_cmd(0xC8); // Set COM Output Scan Direction
    oled_send_cmd(0x00); // Set low column address
    oled_send_cmd(0x10); // Set high column address
    oled_send_cmd(0x40); // Set start line address
    oled_send_cmd(0x81); // Set contrast control register
    oled_send_cmd(0xFF);
    oled_send_cmd(0xA1); // Set segment re-map 0 to 127
    oled_send_cmd(0xA6); // Set normal display
    oled_send_cmd(0xA8); // Set multiplex ratio(1 to 64)
    oled_send_cmd(0x3F); 
    oled_send_cmd(0xA4); // Output follows RAM content
    oled_send_cmd(0xD3); // Set display offset
    oled_send_cmd(0x00); // No offset
    oled_send_cmd(0xD5); // Set display clock divide ratio/oscillator frequency
    oled_send_cmd(0xF0); // Set divide ratio
    oled_send_cmd(0xD9); // Set pre-charge period
    oled_send_cmd(0x22); 
    oled_send_cmd(0xDA); // Set com pins hardware configuration
    oled_send_cmd(0x12);
    oled_send_cmd(0xDB); // Set vcomh
    oled_send_cmd(0x20); // 0.77xVcc
    oled_send_cmd(0x8D); // Set DC-DC enable
    oled_send_cmd(0x14);
    oled_send_cmd(0xAF); // Display ON
}

static void oled_ui_task(void *arg) {
    char buf_line1[32];
    char buf_line2[32];
    char buf_line3[32];
    char buf_line4[32];

    while (1) {
        oled_clear();

        // 1. Connection Status & Camera Mode
        connect_state_t conn = connect_logic_get_state();
        const char *conn_str = "DISCONN";
        if (conn == PROTOCOL_CONNECTED) conn_str = "CONNECTED";
        else if (conn >= BLE_INIT_COMPLETE) conn_str = "SEARCHING";
        
        const char *mode_str = "N/A";
        if (conn == PROTOCOL_CONNECTED) {
            if (current_camera_mode == CAMERA_MODE_NORMAL) mode_str = "VIDEO";
            else if (current_camera_mode == CAMERA_MODE_PHOTO) mode_str = "PHOTO";
        }
        snprintf(buf_line1, sizeof(buf_line1), "%s | %s", conn_str, mode_str);

        // 2. Camera Status (Recording or not)
        if (conn == PROTOCOL_CONNECTED) {
            if (is_camera_recording()) {
                snprintf(buf_line2, sizeof(buf_line2), "CAM: RECORDING");
            } else {
                snprintf(buf_line2, sizeof(buf_line2), "CAM: STANDBY");
            }
        } else {
            snprintf(buf_line2, sizeof(buf_line2), "CAM: OFFLINE");
        }

        // 3. GPS Status
        if (is_current_gps_data_valid()) {
            snprintf(buf_line3, sizeof(buf_line3), "GPS: 3D FIX");
            
            // 4. GPS Coords
            GPS_Data_t gps = get_current_gps_data();
            snprintf(buf_line4, sizeof(buf_line4), "Lat:%.4f", gps.Latitude);
            oled_draw_string(0, 6, buf_line4);
            snprintf(buf_line4, sizeof(buf_line4), "Lon:%.4f", gps.Longitude);
            oled_draw_string(0, 7, buf_line4);
        } else {
            snprintf(buf_line3, sizeof(buf_line3), "GPS: SEARCHING...");
            oled_draw_string(0, 6, "No Coordinates");
        }

        // Draw the text
        oled_draw_string(0, 0, buf_line1);
        oled_draw_string(0, 2, buf_line2);
        oled_draw_string(0, 4, buf_line3);

        oled_update();
        vTaskDelay(pdMS_TO_TICKS(500)); // Update twice a second
    }
}

void oled_logic_init(void) {
    ESP_LOGI(TAG, "Initializing SPI OLED");
    oled_hardware_init();
    oled_clear();
    oled_draw_string(0, 3, "BOOTING...");
    oled_update();
    
    xTaskCreate(oled_ui_task, "oled_ui_task", 4096, NULL, 3, NULL);
}
