#pragma once
#include <cstddef>
#include <cstdint>

#include "proto.h"

using CommandHandler = void (*)(const pk::Command&);

// GATT peripheral "PowerKettle": imu_stream (notify), control (write),
// status (read + notify). One central at a time.
//
// NimBLE callbacks run on the NimBLE host task, not the Arduino loop task:
// the CommandHandler is called from there, so it must only hand the command
// over (e.g. through a FreeRTOS queue). connected()/max_samples() are safe to
// call from the loop.
class BleLink {
 public:
  void begin(CommandHandler on_command);
  bool connected() const;

  // Samples per IMU packet for the current connection's ATT MTU (0 if none).
  size_t max_samples() const;

  // Notifies an IMU packet. Returns false if it could not be queued (e.g. no
  // free mbufs); the caller keeps the data and retries later.
  bool send_imu(const uint8_t* data, size_t len);

  // Updates the readable status value and notifies it (best effort).
  void send_status(const uint8_t* data, size_t len);
};
