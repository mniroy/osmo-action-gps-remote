#ifndef __KEY_LOGIC_H__
#define __KEY_LOGIC_H__

#include "driver/gpio.h"

// Key mapping
#define BOOT_KEY_GPIO   GPIO_NUM_20   // BTN1: record / shutter
#define BTN2_KEY_GPIO   GPIO_NUM_1    // BTN2: quick switch
#define BTN3_KEY_GPIO   GPIO_NUM_0    // BTN3: wake/sleep

typedef enum {
    KEY_EVENT_NONE,
    KEY_EVENT_SINGLE,
    KEY_EVENT_DOUBLE,
    KEY_EVENT_LONG_PRESS
} key_event_t;

void key_logic_init(void);

#endif
