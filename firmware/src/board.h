#pragma once
#include <cstdint>

enum class LedMode { Advertising, Connected, LowBattery };

// Configures the LED pin and battery ADC. Call once from setup().
void board_begin();

// Battery voltage in mV: 16 x analogReadMilliVolts(GPIO4) averaged, x2 for
// the 1:2 divider. Takes ~1 ms; call at most a few times per second.
uint16_t read_battery_mv();

// Drives the LED for `mode`: Advertising = 1 Hz slow blink, Connected = solid
// on, LowBattery = 5 Hz blink. Blinking is computed from millis(), so call
// this every loop iteration.
void led_set(LedMode mode);
