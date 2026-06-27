#ifndef OLED_LOGIC_H
#define OLED_LOGIC_H

#include "driver/gpio.h"
#include "esp_err.h"

// Hardware pins
#define OLED_PIN_SCK  GPIO_NUM_6
#define OLED_PIN_MOSI GPIO_NUM_7
#define OLED_PIN_DC   GPIO_NUM_5
#define OLED_PIN_CS   GPIO_NUM_14
#define OLED_PIN_RST  GPIO_NUM_4

// OLED display dimensions
#define OLED_WIDTH  128
#define OLED_HEIGHT 64

// Initialize the OLED display and SPI bus, and start the UI update task
void oled_logic_init(void);

// Clear the display buffer
void oled_clear(void);

// Update the display with the current buffer
void oled_update(void);

// Draw a string at (x, y) coordinates. y is in pages (0-7), x is 0-127
void oled_draw_string(int x, int y, const char *str);

#endif // OLED_LOGIC_H
