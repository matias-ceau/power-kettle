#pragma once
#include <Wire.h>

#include <cstddef>
#include <cstdint>

#include "proto.h"

// Minimal Bosch BMI160 driver over I2C: accel + gyro in normal mode, samples
// collected through the 1024-byte FIFO in headerless mode (12-byte frames),
// FIFO watermark interrupt routed to INT1 (push-pull, active high).
class Bmi160 {
 public:
  static constexpr size_t kFrameBytes = 12;
  static constexpr size_t kFifoBytes = 1024;
  static constexpr size_t kMaxFifoFrames = kFifoBytes / kFrameBytes;  // 85

  // Expects `wire` to be started already. Checks the chip ID, soft-resets,
  // powers accel + gyro to normal mode and sets up the INT1 watermark IRQ.
  bool begin(TwoWire& wire, uint8_t addr = 0x68);

  // Sets ODR (100/200/400/800 Hz, same for both sensors), accel range
  // (2/4/8/16 g) and gyro range (125..2000 dps), enables the FIFO and flushes
  // it. Returns false on an unsupported value or a sensor/I2C error.
  bool configure(uint16_t odr_hz, uint8_t acc_g, uint16_t gyro_dps);

  void flush_fifo();

  // Reads up to `max` complete frames from the FIFO (oldest first) into `out`
  // as {ax, ay, az, gx, gy, gz}. Pass max >= kMaxFifoFrames to drain it, so
  // that last_read_time_us() refers to the newest sample read.
  size_t read_frames(pk::Sample* out, size_t max);

  // Raw TEMPERATURE register: 0 = 23 degC, 1/512 K per LSB.
  int16_t read_temp_raw();

  // esp_timer time at which the FIFO length was sampled by the last
  // read_frames() call; the last frame read was produced at or before it.
  // After an I2C error it is the time of the resynchronising flush.
  uint64_t last_read_time_us() const { return last_read_time_us_; }

  // True if the last read_frames() call saw a nearly full FIFO (>= 1000
  // bytes; stream mode discards the oldest frames on overflow) or hit an I2C
  // error mid-burst (FIFO flushed, 0 frames returned): frames may have been
  // lost before the ones returned.
  bool last_read_lost_data() const { return last_read_lost_data_; }

 private:
  bool write_reg(uint8_t reg, uint8_t val);
  bool read_regs(uint8_t reg, uint8_t* buf, size_t len);

  TwoWire* wire_ = nullptr;
  uint8_t addr_ = 0x68;
  uint64_t last_read_time_us_ = 0;
  bool last_read_lost_data_ = false;
};
