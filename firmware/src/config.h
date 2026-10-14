#pragma once
#include <cstddef>
#include <cstdint>

// Board wiring (ESP32-C3 SuperMini + BMI160 breakout).
// GPIO2/8/9 are strapping pins: only the on-board LED (GPIO8) uses one.
constexpr int kPinSda = 6;
constexpr int kPinScl = 7;
constexpr int kPinImuInt1 = 3;
constexpr int kPinBattery = 4;  // ADC via 2 x 100 kOhm divider (Vbat / 2)
constexpr int kPinLed = 8;      // on-board LED, active low

constexpr uint32_t kI2cClockHz = 400000;
constexpr uint8_t kBmi160Addr = 0x68;  // SDO low; 0x69 when SDO high

// Default sensor configuration.
constexpr uint16_t kDefaultOdrHz = 400;
constexpr uint8_t kDefaultAccRangeG = 16;
constexpr uint16_t kDefaultGyroRangeDps = 2000;
constexpr uint8_t kFifoWatermarkFrames = 20;

// Main loop timing / buffering.
constexpr size_t kRingSamples = 1600;        // 4 s at 400 Hz
constexpr uint32_t kFifoPollMs = 25;         // fallback when INT1 edge is missed
constexpr uint32_t kSendMaxAgeMs = 50;       // flush a partial packet after this
constexpr int kMaxPacketsPerLoop = 4;
constexpr uint32_t kStatusPeriodMs = 1000;
constexpr uint16_t kLowBatteryMv = 3500;

// BLE.
constexpr const char* kDeviceName = "PowerKettle";
constexpr uint16_t kPreferredMtu = 247;
#define PK_UUID(n) "f3a1000" #n "-6c1e-4b8e-9d2a-5e7c0b4a1d00"
constexpr const char* kUuidService = PK_UUID(1);
constexpr const char* kUuidImuStream = PK_UUID(2);
constexpr const char* kUuidControl = PK_UUID(3);
constexpr const char* kUuidStatus = PK_UUID(4);
#undef PK_UUID
