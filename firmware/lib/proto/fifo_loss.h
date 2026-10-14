#pragma once
#include <cstddef>
#include <cstdint>

namespace pk {

// Estimates how many frames the sensor FIFO lost between two reads, for a
// read that reported possible loss (FIFO near full, or an I2C error that
// forced a flush). Expected frames = round(elapsed * odr); whatever was not
// read is counted as lost, and at least 1, because loss was reported.
// The caller advances sample_index by the result before indexing the frames
// just read, so the gap shows up in first_index.
inline uint32_t estimate_lost_frames(uint64_t t_prev_us, uint64_t t_now_us, uint16_t odr_hz,
                                     size_t frames_read) {
  uint64_t dt = t_now_us > t_prev_us ? t_now_us - t_prev_us : 0;
  uint64_t expected = (dt * odr_hz + 500000) / 1000000;
  if (expected <= frames_read + 1) return 1;
  return static_cast<uint32_t>(expected - frames_read);
}

}  // namespace pk
