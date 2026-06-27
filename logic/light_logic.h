/* SPDX-License-Identifier: MIT */
/*
 * light_logic.h — WS2812B RGB status LED driver for ESP32-C6 Super Mini
 *
 * LED State:
 *   Scanning / not connected  : dim red pulse
 *   BLE connected, no GPS     : blue solid
 *   BLE connected, GPS fixed  : green solid
 *   Recording, no GPS         : blue blink 500ms
 *   Recording, GPS fixed      : green blink 500ms
 *
 * Hardware: WS2812B DIN -> GPIO 8 (via 330Ω series resistor)
 */

#ifndef LIGHT_LOGIC_H
#define LIGHT_LOGIC_H

int init_light_logic(void);

#endif