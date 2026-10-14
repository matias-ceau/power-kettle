#pragma once
#include <cstddef>
#include <cstdint>

namespace pk {

struct Sample {
  int16_t v[6];  // ax, ay, az, gx, gy, gz (raw counts)
};

constexpr size_t kMaxSamples = 19;
constexpr size_t kImuHeader = 8;
constexpr size_t kSampleBytes = 12;
constexpr size_t kStatusLen = 28;

// Max samples per notification for a negotiated ATT MTU (3-byte ATT header).
inline size_t max_samples_for_mtu(uint16_t mtu) {
  if (mtu < 23) return 0;
  size_t n = (static_cast<size_t>(mtu) - 3 - kImuHeader) / kSampleBytes;
  return n < kMaxSamples ? n : kMaxSamples;
}

namespace detail {
inline void put_u16(uint8_t* p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}
inline void put_u32(uint8_t* p, uint32_t v) {
  for (int i = 0; i < 4; ++i) p[i] = static_cast<uint8_t>((v >> (8 * i)) & 0xFF);
}
inline void put_u64(uint8_t* p, uint64_t v) {
  for (int i = 0; i < 8; ++i) p[i] = static_cast<uint8_t>((v >> (8 * i)) & 0xFF);
}
inline uint16_t get_u16(const uint8_t* p) {
  return static_cast<uint16_t>(p[0] | (p[1] << 8));
}
}  // namespace detail

// IMU packet: u16 seq, u32 first_index, u8 n, u8 flags, n * i16[6].
// `out` must hold kImuHeader + n * kSampleBytes bytes; n must be <= kMaxSamples.
inline size_t encode_imu(uint8_t* out, uint16_t seq, uint32_t first_index,
                         uint8_t flags, const Sample* s, size_t n) {
  detail::put_u16(out, seq);
  detail::put_u32(out + 2, first_index);
  out[6] = static_cast<uint8_t>(n);
  out[7] = flags;
  size_t pos = kImuHeader;
  for (size_t i = 0; i < n; ++i) {
    for (int k = 0; k < 6; ++k) {
      detail::put_u16(out + pos, static_cast<uint16_t>(s[i].v[k]));
      pos += 2;
    }
  }
  return pos;
}

struct StatusFields {
  uint8_t flags;  // bit0 streaming, bit1 low_battery
  uint16_t battery_mv, odr_hz;
  uint8_t acc_range_g;
  uint16_t gyro_range_dps;
  int16_t temp_raw;
  uint32_t dropped, sample_index;
  uint64_t time_us;
};

// Status packet (28 bytes, version 1). `out` must hold kStatusLen bytes.
inline size_t encode_status(uint8_t* out, const StatusFields& f) {
  out[0] = 1;  // version
  out[1] = f.flags;
  detail::put_u16(out + 2, f.battery_mv);
  detail::put_u16(out + 4, f.odr_hz);
  out[6] = f.acc_range_g;
  out[7] = 0;  // reserved
  detail::put_u16(out + 8, f.gyro_range_dps);
  detail::put_u16(out + 10, static_cast<uint16_t>(f.temp_raw));
  detail::put_u32(out + 12, f.dropped);
  detail::put_u32(out + 16, f.sample_index);
  detail::put_u64(out + 20, f.time_us);
  return kStatusLen;
}

enum class Op : uint8_t { Invalid = 0, Start = 1, Stop = 2, SetConfig = 3, Ping = 4 };

struct Command {
  Op op;
  uint16_t odr_hz;
  uint8_t acc_range_g;
  uint16_t gyro_range_dps;
};

inline Command parse_command(const uint8_t* b, size_t len) {
  Command c{Op::Invalid, 0, 0, 0};
  if (b == nullptr || len == 0) return c;
  switch (b[0]) {
    case 1:
      if (len == 1) c.op = Op::Start;
      break;
    case 2:
      if (len == 1) c.op = Op::Stop;
      break;
    case 4:
      if (len == 1) c.op = Op::Ping;
      break;
    case 3: {
      if (len != 6) break;
      uint16_t odr = detail::get_u16(b + 1);
      uint8_t acc = b[3];
      uint16_t gyro = detail::get_u16(b + 4);
      bool odr_ok = odr == 100 || odr == 200 || odr == 400 || odr == 800;
      bool acc_ok = acc == 2 || acc == 4 || acc == 8 || acc == 16;
      bool gyro_ok = gyro == 125 || gyro == 250 || gyro == 500 || gyro == 1000 ||
                     gyro == 2000;
      if (odr_ok && acc_ok && gyro_ok) {
        c.op = Op::SetConfig;
        c.odr_hz = odr;
        c.acc_range_g = acc;
        c.gyro_range_dps = gyro;
      }
      break;
    }
    default:
      break;
  }
  return c;
}

}  // namespace pk
