#include "board.h"

#include <Arduino.h>

#include "config.h"

void board_begin() {
  pinMode(kPinLed, OUTPUT);
  digitalWrite(kPinLed, HIGH);  // active low: off
  // 11 dB attenuation (the Arduino default) covers the ~2.1 V divider output
  // of a full 4.2 V cell; set explicitly so it does not depend on the core.
  analogSetPinAttenuation(kPinBattery, ADC_11db);
}

uint16_t read_battery_mv() {
  uint32_t sum = 0;
  for (int i = 0; i < 16; ++i) sum += analogReadMilliVolts(kPinBattery);
  return static_cast<uint16_t>((sum / 16) * 2);
}

void led_set(LedMode mode) {
  bool on;
  switch (mode) {
    case LedMode::Connected:
      on = true;
      break;
    case LedMode::LowBattery:
      on = (millis() % 200) < 100;  // 5 Hz
      break;
    case LedMode::Advertising:
    default:
      on = (millis() % 1000) < 500;  // 1 Hz
      break;
  }
  digitalWrite(kPinLed, on ? LOW : HIGH);
}
