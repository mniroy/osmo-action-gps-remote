#include "oled_logic.h"
#include "oled_font.h"
#include "driver/i2c.h"
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

#define I2C_PORT I2C_NUM_0
static uint8_t oled_buffer[OLED_WIDTH * OLED_HEIGHT / 8];

// ---------------------------------------------------------------------------
// Low-level SPI helpers
// ---------------------------------------------------------------------------

static void oled_send_cmd(uint8_t cmd) {
    uint8_t buf[2] = {0x00, cmd}; // 0x00 = Command
    i2c_master_write_to_device(I2C_PORT, OLED_I2C_ADDR, buf, 2, pdMS_TO_TICKS(100));
}

static void oled_send_data(const uint8_t *data, int len) {
    if (len == 0) return;
    uint8_t *buf = malloc(len + 1);
    if (!buf) return;
    buf[0] = 0x40; // 0x40 = Data
    memcpy(buf + 1, data, len);
    i2c_master_write_to_device(I2C_PORT, OLED_I2C_ADDR, buf, len + 1, pdMS_TO_TICKS(100));
    free(buf);
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
    while (*str) {
        if (x > OLED_WIDTH - 5) break;
        char c = *str++;
        if (c < 32 || c > 126) c = 32;
        int font_idx = (c - 32) * 5;
        for (int i = 0; i < 5; i++) {
            oled_buffer[y_page * OLED_WIDTH + x + i] = font5x7[font_idx + i];
        }
        x += 6;
    }
}

void oled_draw_string_x2(int x, int y_page, const char *str) {
    while (*str) {
        if (x > OLED_WIDTH - 10) break;
        char c = *str++;
        if (c < 32 || c > 126) c = 32;
        int font_idx = (c - 32) * 5;
        for (int i = 0; i < 5; i++) {
            uint8_t col = font5x7[font_idx + i];
            uint16_t out_col = 0;
            for(int bit = 0; bit < 7; bit++) {
                if(col & (1<<bit)) {
                    out_col |= (3 << (bit*2));
                }
            }
            uint8_t top = out_col & 0xFF;
            uint8_t bottom = out_col >> 8;
            
            if(y_page < 8) oled_buffer[y_page * OLED_WIDTH + x + i*2] = top;
            if(y_page + 1 < 8) oled_buffer[(y_page+1) * OLED_WIDTH + x + i*2] = bottom;
            
            if(y_page < 8) oled_buffer[y_page * OLED_WIDTH + x + i*2 + 1] = top;
            if(y_page + 1 < 8) oled_buffer[(y_page+1) * OLED_WIDTH + x + i*2 + 1] = bottom;
        }
        x += 12; // 10 for char, 2 for spacing
    }
}

void oled_draw_string_x3(int x, int y_page, const char *str) {
    while (*str) {
        if (x > OLED_WIDTH - 15) break;
        char c = *str++;
        if (c < 32 || c > 126) c = 32;
        int font_idx = (c - 32) * 5;
        for (int i = 0; i < 5; i++) {
            uint8_t col = font5x7[font_idx + i];
            uint32_t out_col = 0;
            for(int bit = 0; bit < 7; bit++) {
                if(col & (1<<bit)) {
                    out_col |= (7 << (bit*3));
                }
            }
            uint8_t p0 = out_col & 0xFF;
            uint8_t p1 = (out_col >> 8) & 0xFF;
            uint8_t p2 = (out_col >> 16) & 0xFF;
            
            for(int k=0; k<3; k++) {
                if(y_page < 8) oled_buffer[y_page * OLED_WIDTH + x + i*3 + k] = p0;
                if(y_page + 1 < 8) oled_buffer[(y_page+1) * OLED_WIDTH + x + i*3 + k] = p1;
                if(y_page + 2 < 8) oled_buffer[(y_page+2) * OLED_WIDTH + x + i*3 + k] = p2;
            }
        }
        x += 18; // 15 for char, 3 for spacing
    }
}

void oled_draw_bitmap(int x, int y_page, int w, int h_pages, const uint8_t *bitmap) {
    for (int p = 0; p < h_pages; p++) {
        if (y_page + p > 7) continue;
        for (int i = 0; i < w; i++) {
            if (x + i >= OLED_WIDTH) break;
            oled_buffer[(y_page + p) * OLED_WIDTH + x + i] = bitmap[p * w + i];
        }
    }
}

// ---------------------------------------------------------------------------
// Hardware init
// ---------------------------------------------------------------------------

static void oled_hardware_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = OLED_I2C_SDA,
        .scl_io_num = OLED_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };
    i2c_param_config(I2C_PORT, &conf);
    i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);

    vTaskDelay(pdMS_TO_TICKS(50));


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
    char temp_str[32];

    while (1) {
        oled_clear();

        connect_state_t conn = connect_logic_get_state();
        
        // --- Top Left: SD Card / Mode ---
        oled_draw_bitmap(0, 0, 16, 2, icon_sd_card);
        if (conn == PROTOCOL_CONNECTED) {
            if (current_camera_mode == 0x05) { // Photo Mode
                if (current_remain_photo_num >= 1000) {
                    snprintf(temp_str, sizeof(temp_str), "%luK", (unsigned long)current_remain_photo_num / 1000);
                } else {
                    snprintf(temp_str, sizeof(temp_str), "%lu", (unsigned long)current_remain_photo_num);
                }
            } else { // Video modes
                uint32_t mins = current_remain_time / 60;
                uint32_t hours = mins / 60;
                mins = mins % 60;
                if (hours > 0) {
                    snprintf(temp_str, sizeof(temp_str), "%luh%02lu", (unsigned long)hours, (unsigned long)mins);
                } else {
                    snprintf(temp_str, sizeof(temp_str), "%lum", (unsigned long)mins);
                }
            }
        } else {
            snprintf(temp_str, sizeof(temp_str), "DIS");
        }
        oled_draw_string(20, 0, temp_str);

        // --- Top Right: GPS & Battery ---
        if (is_current_gps_data_valid_ui()) {
            GPS_Data_t gps = get_current_gps_data();
            snprintf(temp_str, sizeof(temp_str), "SAT: %d", gps.Num_Satellites);
            oled_draw_string(60, 0, temp_str);
        } else {
            oled_draw_string(60, 0, "SAT: --");
        }

        oled_draw_bitmap(112, 0, 16, 2, icon_batt_full);

        // --- Center Display ---
        if (conn == PROTOCOL_CONNECTED && current_power_mode != 3) {
            bool is_rec = is_camera_recording();
            if (is_rec) {
                // Show Speed big, and Timer small below it
                GPS_Data_t gps = get_current_gps_data();
                int speed_kmh = (int)(gps.Speed_knots * 1.852);
                snprintf(temp_str, sizeof(temp_str), "%d", speed_kmh);
                
                int speed_text_len = strlen(temp_str);
                int speed_width = speed_text_len * 18;
                int kmh_width = 4 * 6; // "KM/H" length = 4 chars
                int total_width = speed_width + kmh_width + 4; // Add a small 4px gap
                
                int x_pos = (128 - total_width) / 2;
                
                // Draw 3x speed
                oled_draw_string_x3(x_pos, 2, temp_str);
                
                // Draw "KM/H" next to it at the bottom alignment (page 2+2=4)
                oled_draw_string(x_pos + speed_width + 4, 4, "KM/H");
                
                uint16_t time = current_record_time;
                snprintf(temp_str, sizeof(temp_str), "%02d:%02d", time / 60, time % 60);
                // Center timer text
                int text_len = strlen(temp_str);
                x_pos = (128 - (text_len * 6)) / 2;
                oled_draw_string(x_pos, 6, temp_str);
            } else {
                // Show Mode
                const uint8_t *mode_icon = icon_video;
                const char *mode_text = "VIDEO";
                
                switch(current_camera_mode) {
                    case 0x00: mode_text = "SLOW MOTION"; break;
                    case 0x01: mode_text = "VIDEO"; break;
                    case 0x02: mode_text = "TIMELAPSE"; break;
                    case 0x05: mode_text = "PHOTO"; mode_icon = icon_photo; break;
                    case 0x0A: mode_text = "HYPERLAPSE"; break;
                    case 0x1A: mode_text = "LIVE STREAM"; break;
                    case 0x23: mode_text = "UVC LIVE"; break;
                    case 0x28: mode_text = "LOW LIGHT"; break;
                    default:   mode_text = "UNKNOWN"; break;
                }
                
                oled_draw_bitmap(48, 2, 32, 4, mode_icon);
                
                // Center text manually, assume 5x7 font (5 chars = 30px, 7 chars = 42px)
                int text_len = strlen(mode_text);
                int x_pos = (128 - (text_len * 6)) / 2; // Center horizontally
                oled_draw_string(x_pos, 6, mode_text);
            }
        } else {
            // Not connected or in sleep mode
            oled_draw_bitmap(48, 2, 32, 4, icon_moon);
            oled_draw_string(24, 6, "WAKE UP CAMERA");
        }

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
