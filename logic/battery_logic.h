#ifndef BATTERY_LOGIC_H
#define BATTERY_LOGIC_H

#include <stdint.h>

void battery_logic_init(void);
uint8_t get_battery_percentage(void);

#endif // BATTERY_LOGIC_H
