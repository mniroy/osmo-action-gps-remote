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
#include "command_logic.h"

#define TAG "OLED"

static spi_device_handle_t spi;
static uint8_t oled_buffer[OLED_WIDTH * OLED_HEIGHT / 8];

// ---------------------------------------------------------------------------
// Low-level SPI helpers
// ---------------------------------------------------------------------------

static void oled_send_cmd(uint8_t cmd) {
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = &cmd;
    gpio_set_level(OLED_PIN_DC, 0); // DC LOW = command
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &t));
}

static void oled_send_data(const uint8_t *data, int len) {
    if (len == 0) return;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = len * 8;
    t.tx_buffer = data;
    gpio_set_level(OLED_PIN_DC, 1); // DC HIGH = data
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &t));
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void oled_clear(void) {
    memset(oled_buffer, 0, sizeof(oled_buffer));
}

void oled_update(void) {
    for (int page = 0; page < 8; page++) {
        oled_send_cmd(0xB0 + page); // Set page address
        oled_send_cmd(0x00);        // SSD1306: column start = 0
        oled_send_cmd(0x10);        // Set high column to 0
        oled_send_data(&oled_buffer[OLED_WIDTH * page], OLED_WIDTH);
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
        x += 6;
        str++;
    }
}

// ---------------------------------------------------------------------------
// Hardware init
// ---------------------------------------------------------------------------

static void oled_hardware_init(void) {
    // Configure DC and RST as outputs
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << OLED_PIN_DC) | (1ULL << OLED_PIN_RST),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    spi_bus_config_t buscfg = {
        .miso_io_num   = -1,
        .mosi_io_num   = OLED_PIN_MOSI,
        .sclk_io_num   = OLED_PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 1024
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000, // 1 MHz
        .mode           = 0,
        .spics_io_num   = OLED_PIN_CS,
        .queue_size     = 7
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi));

    // Hardware reset
    gpio_set_level(OLED_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(OLED_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Init sequence — compatible with SSD1306 and SH1106
    oled_send_cmd(0xAE); // Display OFF
    oled_send_cmd(0xD5); // Clock divide ratio
    oled_send_cmd(0x80);
    oled_send_cmd(0xA8); // Multiplex ratio
    oled_send_cmd(0x3F); // 64 rows
    oled_send_cmd(0xD3); // Display offset
    oled_send_cmd(0x00);
    oled_send_cmd(0x40); // Start line = 0
    oled_send_cmd(0x8D); // Charge pump
    oled_send_cmd(0x14); // Enable internal VCC
    oled_send_cmd(0xA1); // Segment remap
    oled_send_cmd(0xC8); // COM scan direction: reversed (fixes upside-down)
    oled_send_cmd(0xDA); // COM pins config
    oled_send_cmd(0x12);
    oled_send_cmd(0x81); // Contrast
    oled_send_cmd(0xCF);
    oled_send_cmd(0xD9); // Pre-charge period
    oled_send_cmd(0xF1);
    oled_send_cmd(0xDB); // VCOMH deselect level
    oled_send_cmd(0x40);
    oled_send_cmd(0xA4); // Output follows RAM
    oled_send_cmd(0xA6); // Normal polarity

    // Clear GDDRAM before Display ON to prevent power-on noise
    static uint8_t zero_page[128];
    memset(zero_page, 0x00, sizeof(zero_page));
    for (int page = 0; page < 8; page++) {
        oled_send_cmd(0xB0 + page);
        oled_send_cmd(0x00); // SSD1306: column start = 0
        oled_send_cmd(0x10);
        oled_send_data(zero_page, 128);
    }

    oled_send_cmd(0xAF); // Display ON
}

// ---------------------------------------------------------------------------
// UI task
// ---------------------------------------------------------------------------

static void oled_ui_task(void *arg) {
    char line1[32], line2[32], line3[32], line4[32];

    while (1) {
        oled_clear();

        // Line 0 — BLE connection + camera mode
        connect_state_t conn = connect_logic_get_state();
        const char *conn_str = "DISCONN";
        if      (conn == PROTOCOL_CONNECTED)     conn_str = "CONNECTED";
        else if (conn >= BLE_INIT_COMPLETE)      conn_str = "SEARCHING";

        const char *mode_str = "N/A";
        if (conn == PROTOCOL_CONNECTED) {
            if      (current_camera_mode == CAMERA_MODE_NORMAL) mode_str = "VIDEO";
            else if (current_camera_mode == CAMERA_MODE_PHOTO)  mode_str = "PHOTO";
        }
        snprintf(line1, sizeof(line1), "%s | %s", conn_str, mode_str);

        // Line 2 — Camera state
        if (conn == PROTOCOL_CONNECTED) {
            snprintf(line2, sizeof(line2),
                     is_camera_recording() ? "CAM: RECORDING" : "CAM: STANDBY");
        } else {
            snprintf(line2, sizeof(line2), "CAM: OFFLINE");
        }

        // Lines 4/6/7 — GPS
        if (is_current_gps_data_valid()) {
            snprintf(line3, sizeof(line3), "GPS: 3D FIX");
            GPS_Data_t gps = get_current_gps_data();
            snprintf(line4, sizeof(line4), "Lat:%.4f", gps.Latitude);
            oled_draw_string(0, 6, line4);
            snprintf(line4, sizeof(line4), "Lon:%.4f", gps.Longitude);
            oled_draw_string(0, 7, line4);
        } else {
            snprintf(line3, sizeof(line3), "GPS: SEARCHING...");
            oled_draw_string(0, 6, "No Coordinates");
        }

        oled_draw_string(0, 0, line1);
        oled_draw_string(0, 2, line2);
        oled_draw_string(0, 4, line3);

        oled_update();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

void oled_logic_init(void) {
    ESP_LOGI(TAG, "Initializing SPI OLED");
    oled_hardware_init();
    oled_clear();
    oled_draw_string(32, 3, "BOOTING...");
    oled_update();
    xTaskCreate(oled_ui_task, "oled_ui_task", 4096, NULL, 3, NULL);
}
