#include "bmi160.h"

#include <Arduino.h>
#include <esp_timer.h>

#include "config.h"

// Register map (Bosch BST-BMI160-DS000).
namespace reg {
constexpr uint8_t kChipId = 0x00;      // reads 0xD1
constexpr uint8_t kErr = 0x02;         // bits 4:1 err_code
constexpr uint8_t kPmuStatus = 0x03;   // acc bits 5:4, gyr bits 3:2 (01 = normal)
constexpr uint8_t kTemperature = 0x20;  // 0x20..0x21, i16 LE
constexpr uint8_t kFifoLength = 0x22;  // 0x22..0x23, 11-bit byte count
constexpr uint8_t kFifoData = 0x24;
constexpr uint8_t kAccConf = 0x40;
constexpr uint8_t kAccRange = 0x41;
constexpr uint8_t kGyrConf = 0x42;
constexpr uint8_t kGyrRange = 0x43;
constexpr uint8_t kFifoConfig0 = 0x46;  // watermark, in 4-byte units
constexpr uint8_t kFifoConfig1 = 0x47;
constexpr uint8_t kIntEn1 = 0x51;
constexpr uint8_t kIntOutCtrl = 0x53;
constexpr uint8_t kIntMap1 = 0x56;
constexpr uint8_t kCmd = 0x7E;
}  // namespace reg

namespace cmd {
constexpr uint8_t kSoftReset = 0xB6;
constexpr uint8_t kAccNormal = 0x11;
constexpr uint8_t kGyrNormal = 0x15;
constexpr uint8_t kFifoFlush = 0xB0;
}  // namespace cmd

constexpr uint8_t kChipIdValue = 0xD1;
// Above this FIFO fill level the 1024-byte FIFO may already have overflowed
// (stream mode silently discards the oldest data).
constexpr uint16_t kFifoOverflowBytes = 1000;
// Arduino Wire buffer is 128 bytes; read whole frames only (10 x 12 = 120) so
// a chunk never ends mid-frame.
constexpr size_t kChunkFrames = 120 / Bmi160::kFrameBytes;

namespace {

// ACC_CONF / GYR_CONF odr field (bits 3:0); both sensors use the same codes.
bool odr_code(uint16_t hz, uint8_t* code) {
  switch (hz) {
    case 100: *code = 0x08; return true;
    case 200: *code = 0x09; return true;
    case 400: *code = 0x0A; return true;
    case 800: *code = 0x0B; return true;
    default: return false;
  }
}

bool acc_range_code(uint8_t g, uint8_t* code) {
  switch (g) {
    case 2: *code = 0x03; return true;
    case 4: *code = 0x05; return true;
    case 8: *code = 0x08; return true;
    case 16: *code = 0x0C; return true;
    default: return false;
  }
}

bool gyr_range_code(uint16_t dps, uint8_t* code) {
  switch (dps) {
    case 2000: *code = 0; return true;
    case 1000: *code = 1; return true;
    case 500: *code = 2; return true;
    case 250: *code = 3; return true;
    case 125: *code = 4; return true;
    default: return false;
  }
}

inline int16_t le16(const uint8_t* p) {
  return static_cast<int16_t>(static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8));
}

}  // namespace

bool Bmi160::write_reg(uint8_t r, uint8_t val) {
  wire_->beginTransmission(addr_);
  wire_->write(r);
  wire_->write(val);
  return wire_->endTransmission() == 0;
}

bool Bmi160::read_regs(uint8_t r, uint8_t* buf, size_t len) {
  wire_->beginTransmission(addr_);
  wire_->write(r);
  if (wire_->endTransmission(false) != 0) return false;  // repeated start
  size_t got = wire_->requestFrom(addr_, len);
  if (got != len) return false;
  for (size_t i = 0; i < len; ++i) buf[i] = static_cast<uint8_t>(wire_->read());
  return true;
}

bool Bmi160::begin(TwoWire& wire, uint8_t addr) {
  wire_ = &wire;
  addr_ = addr;

  uint8_t id = 0;
  if (!read_regs(reg::kChipId, &id, 1) || id != kChipIdValue) {
    Serial.printf("[bmi160] chip id 0x%02X (expected 0x%02X)\n", id, kChipIdValue);
    return false;
  }
  Serial.printf("[bmi160] chip id 0x%02X\n", id);

  // Soft reset restores all registers to defaults (sensors suspended, FIFO
  // in header mode).
  write_reg(reg::kCmd, cmd::kSoftReset);
  delay(100);

  // Power up: accel needs <= 3.8 ms, gyro <= 80 ms from suspend. Commands
  // must not be sent while the previous one is still executing.
  if (!write_reg(reg::kCmd, cmd::kAccNormal)) return false;
  delay(5);
  if (!write_reg(reg::kCmd, cmd::kGyrNormal)) return false;
  delay(80);

  uint8_t pmu = 0;
  if (!read_regs(reg::kPmuStatus, &pmu, 1) || ((pmu >> 4) & 0x3) != 1 || ((pmu >> 2) & 0x3) != 1) {
    Serial.printf("[bmi160] PMU_STATUS 0x%02X: accel/gyro not in normal mode\n", pmu);
    return false;
  }

  // INT1: output enabled, active high, push-pull, level (non-latched).
  // The line stays high while FIFO fill >= watermark, so the rising-edge ISR
  // only fires again once a read has taken the fill below the watermark;
  // the main loop's periodic poll covers any missed edge.
  bool ok = write_reg(reg::kIntOutCtrl, 0x0A);
  ok = ok && write_reg(reg::kIntMap1, 0x40);  // FIFO watermark -> INT1
  ok = ok && write_reg(reg::kIntEn1, 0x40);   // enable FIFO watermark IRQ
  return ok;
}

bool Bmi160::configure(uint16_t odr_hz, uint8_t acc_g, uint16_t gyro_dps) {
  uint8_t odr, acc, gyr;
  if (!odr_code(odr_hz, &odr) || !acc_range_code(acc_g, &acc) || !gyr_range_code(gyro_dps, &gyr)) {
    return false;
  }
  // ACC_CONF: acc_us = 0, acc_bwp = 0b010 (normal filter), acc_odr.
  bool ok = write_reg(reg::kAccConf, static_cast<uint8_t>((0b010 << 4) | odr));
  ok = ok && write_reg(reg::kAccRange, acc);
  // GYR_CONF: gyr_bwp = 0b10 (normal filter), gyr_odr.
  ok = ok && write_reg(reg::kGyrConf, static_cast<uint8_t>((0b10 << 4) | odr));
  ok = ok && write_reg(reg::kGyrRange, gyr);
  // Watermark in 4-byte units: 20 frames x 12 bytes / 4 = 60.
  ok = ok && write_reg(reg::kFifoConfig0, kFifoWatermarkFrames * kFrameBytes / 4);
  // FIFO_CONFIG_1: fifo_gyr_en | fifo_acc_en, header disabled. In headerless
  // mode each frame is gyro x,y,z then accel x,y,z.
  ok = ok && write_reg(reg::kFifoConfig1, 0xC0);
  if (!ok) return false;

  // err_code (e.g. 0b0110: headerless FIFO with mismatched sensor ODRs, or an
  // invalid ACC_CONF/GYR_CONF combination) makes the config unusable.
  uint8_t err = 0;
  if (!read_regs(reg::kErr, &err, 1)) return false;
  if ((err >> 1) & 0x0F) {
    Serial.printf("[bmi160] ERR_REG 0x%02X after configure\n", err);
    return false;
  }
  flush_fifo();  // drop frames produced under the previous configuration
  return true;
}

void Bmi160::flush_fifo() { write_reg(reg::kCmd, cmd::kFifoFlush); }

size_t Bmi160::read_frames(pk::Sample* out, size_t max) {
  uint8_t lenbuf[2];
  last_read_lost_data_ = false;
  if (!read_regs(reg::kFifoLength, lenbuf, 2)) {
    last_read_lost_data_ = true;
    return 0;
  }
  last_read_time_us_ = static_cast<uint64_t>(esp_timer_get_time());
  uint16_t fifo_bytes = static_cast<uint16_t>(lenbuf[0] | ((lenbuf[1] & 0x07) << 8));
  if (fifo_bytes >= kFifoOverflowBytes) last_read_lost_data_ = true;

  size_t frames = fifo_bytes / kFrameBytes;
  if (frames > max) frames = max;

  uint8_t buf[kChunkFrames * kFrameBytes];
  size_t done = 0;
  while (done < frames) {
    size_t n = frames - done;
    if (n > kChunkFrames) n = kChunkFrames;
    if (!read_regs(reg::kFifoData, buf, n * kFrameBytes)) {
      // A failed burst may have consumed part of a frame and left the FIFO
      // misaligned; flush to resynchronise and report the loss.
      flush_fifo();
      last_read_lost_data_ = true;
      break;
    }
    for (size_t i = 0; i < n; ++i) {
      const uint8_t* f = buf + i * kFrameBytes;  // gx gy gz ax ay az
      pk::Sample& s = out[done + i];
      s.v[0] = le16(f + 6);
      s.v[1] = le16(f + 8);
      s.v[2] = le16(f + 10);
      s.v[3] = le16(f + 0);
      s.v[4] = le16(f + 2);
      s.v[5] = le16(f + 4);
    }
    done += n;
  }
  return done;
}

int16_t Bmi160::read_temp_raw() {
  uint8_t b[2];
  if (!read_regs(reg::kTemperature, b, 2)) return static_cast<int16_t>(0x8000);  // "invalid"
  return le16(b);
}
