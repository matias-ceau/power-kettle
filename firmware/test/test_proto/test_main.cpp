#include <unity.h>

#include <cstdint>
#include <cstring>

#include "proto.h"
#include "ring_buffer.h"
#include "segments.h"
#include "fifo_loss.h"

using namespace pk;

void setUp() {}
void tearDown() {}

static const uint8_t kGoldenImu[32] = {
    0x34, 0x12, 0xe8, 0x03, 0x00, 0x00, 0x02, 0x01, 0x01, 0x00, 0x02,
    0x00, 0x03, 0x00, 0x04, 0x00, 0x05, 0x00, 0x06, 0x00, 0xff, 0xff,
    0xfe, 0xff, 0xfd, 0xff, 0xfc, 0xff, 0xfb, 0xff, 0xfa, 0xff};

static const uint8_t kGoldenStatus[28] = {
    0x01, 0x01, 0x93, 0x0f, 0x90, 0x01, 0x10, 0x00, 0xd0, 0x07,
    0x00, 0xfe, 0x07, 0x00, 0x00, 0x00, 0x40, 0xe2, 0x01, 0x00,
    0xea, 0x16, 0xb0, 0x4c, 0x02, 0x00, 0x00, 0x00};

static Sample make_sample(int16_t base) {
  Sample s;
  for (int i = 0; i < 6; ++i) s.v[i] = static_cast<int16_t>(base + i);
  return s;
}

void test_encode_imu_golden() {
  Sample s[2] = {{{1, 2, 3, 4, 5, 6}}, {{-1, -2, -3, -4, -5, -6}}};
  uint8_t out[64];
  std::memset(out, 0xAA, sizeof(out));
  size_t n = encode_imu(out, 0x1234, 1000, 1, s, 2);
  TEST_ASSERT_EQUAL_UINT32(32, n);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(kGoldenImu, out, 32);
  TEST_ASSERT_EQUAL_UINT8(0xAA, out[32]);  // no overrun
}

void test_encode_imu_full_packet_length() {
  Sample s[kMaxSamples];
  for (size_t i = 0; i < kMaxSamples; ++i) s[i] = make_sample(static_cast<int16_t>(i));
  uint8_t out[256];
  size_t n = encode_imu(out, 0, 0, 0, s, kMaxSamples);
  TEST_ASSERT_EQUAL_UINT32(kImuHeader + 12 * kMaxSamples, n);
  TEST_ASSERT_EQUAL_UINT8(19, out[6]);
}

void test_encode_status_golden() {
  StatusFields f{};
  f.flags = 1;
  f.battery_mv = 3987;
  f.odr_hz = 400;
  f.acc_range_g = 16;
  f.gyro_range_dps = 2000;
  f.temp_raw = -512;
  f.dropped = 7;
  f.sample_index = 123456;
  f.time_us = 9876543210ULL;
  uint8_t out[40];
  std::memset(out, 0xAA, sizeof(out));
  size_t n = encode_status(out, f);
  TEST_ASSERT_EQUAL_UINT32(28, n);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(kGoldenStatus, out, 28);
  TEST_ASSERT_EQUAL_UINT8(0xAA, out[28]);
}

void test_parse_set_config() {
  const uint8_t b[] = {0x03, 0x90, 0x01, 0x10, 0xD0, 0x07};
  Command c = parse_command(b, sizeof(b));
  TEST_ASSERT_TRUE(c.op == Op::SetConfig);
  TEST_ASSERT_EQUAL_UINT16(400, c.odr_hz);
  TEST_ASSERT_EQUAL_UINT8(16, c.acc_range_g);
  TEST_ASSERT_EQUAL_UINT16(2000, c.gyro_range_dps);
}

void test_parse_set_config_invalid_values() {
  const uint8_t bad_odr[] = {0x03, 0x2C, 0x01, 0x10, 0xD0, 0x07};  // 300
  TEST_ASSERT_TRUE(parse_command(bad_odr, sizeof(bad_odr)).op == Op::Invalid);
  const uint8_t bad_acc[] = {0x03, 0x90, 0x01, 0x03, 0xD0, 0x07};
  TEST_ASSERT_TRUE(parse_command(bad_acc, sizeof(bad_acc)).op == Op::Invalid);
  const uint8_t bad_gyro[] = {0x03, 0x90, 0x01, 0x10, 0xD1, 0x07};
  TEST_ASSERT_TRUE(parse_command(bad_gyro, sizeof(bad_gyro)).op == Op::Invalid);
  const uint8_t short_cfg[] = {0x03, 0x90, 0x01, 0x10, 0xD0};
  TEST_ASSERT_TRUE(parse_command(short_cfg, sizeof(short_cfg)).op == Op::Invalid);
}

void test_parse_simple_ops() {
  const uint8_t start[] = {0x01};
  TEST_ASSERT_TRUE(parse_command(start, 1).op == Op::Start);
  const uint8_t stop[] = {0x02};
  TEST_ASSERT_TRUE(parse_command(stop, 1).op == Op::Stop);
  const uint8_t ping[] = {0x04};
  TEST_ASSERT_TRUE(parse_command(ping, 1).op == Op::Ping);
}

void test_parse_invalid() {
  const uint8_t two[] = {0x01, 0x00};
  TEST_ASSERT_TRUE(parse_command(two, 2).op == Op::Invalid);
  const uint8_t unknown[] = {0x09};
  TEST_ASSERT_TRUE(parse_command(unknown, 1).op == Op::Invalid);
  TEST_ASSERT_TRUE(parse_command(nullptr, 0).op == Op::Invalid);
  const uint8_t dummy[] = {0x01};
  TEST_ASSERT_TRUE(parse_command(dummy, 0).op == Op::Invalid);
}

void test_max_samples_for_mtu() {
  TEST_ASSERT_EQUAL_UINT32(19, max_samples_for_mtu(247));
  TEST_ASSERT_EQUAL_UINT32(1, max_samples_for_mtu(23));
  TEST_ASSERT_EQUAL_UINT32(7, max_samples_for_mtu(100));
  TEST_ASSERT_EQUAL_UINT32(0, max_samples_for_mtu(20));
  TEST_ASSERT_EQUAL_UINT32(19, max_samples_for_mtu(517));
}

void test_ring_buffer_fifo_and_full() {
  RingBuffer<4> rb;
  TEST_ASSERT_EQUAL_UINT32(0, rb.size());
  for (int i = 0; i < 4; ++i) TEST_ASSERT_TRUE(rb.push(make_sample(static_cast<int16_t>(i * 10))));
  TEST_ASSERT_FALSE(rb.push(make_sample(99)));
  TEST_ASSERT_EQUAL_UINT32(4, rb.size());

  Sample out[4];
  TEST_ASSERT_EQUAL_UINT32(3, rb.pop(out, 3));
  TEST_ASSERT_EQUAL_INT16(0, out[0].v[0]);
  TEST_ASSERT_EQUAL_INT16(10, out[1].v[0]);
  TEST_ASSERT_EQUAL_INT16(20, out[2].v[0]);
  TEST_ASSERT_EQUAL_UINT32(1, rb.size());
}

void test_ring_buffer_wraparound_and_clear() {
  RingBuffer<4> rb;
  for (int i = 0; i < 4; ++i) rb.push(make_sample(static_cast<int16_t>(i * 10)));
  Sample out[8];
  rb.pop(out, 3);                      // leaves sample 30
  TEST_ASSERT_TRUE(rb.push(make_sample(40)));
  TEST_ASSERT_TRUE(rb.push(make_sample(50)));
  TEST_ASSERT_TRUE(rb.push(make_sample(60)));
  TEST_ASSERT_FALSE(rb.push(make_sample(70)));
  TEST_ASSERT_EQUAL_UINT32(4, rb.size());
  TEST_ASSERT_EQUAL_UINT32(4, rb.pop(out, 8));
  TEST_ASSERT_EQUAL_INT16(30, out[0].v[0]);
  TEST_ASSERT_EQUAL_INT16(40, out[1].v[0]);
  TEST_ASSERT_EQUAL_INT16(50, out[2].v[0]);
  TEST_ASSERT_EQUAL_INT16(60, out[3].v[0]);
  TEST_ASSERT_EQUAL_UINT32(0, rb.pop(out, 8));

  rb.push(make_sample(1));
  rb.clear();
  TEST_ASSERT_EQUAL_UINT32(0, rb.size());
  TEST_ASSERT_TRUE(rb.push(make_sample(2)));
  TEST_ASSERT_EQUAL_UINT32(1, rb.pop(out, 8));
  TEST_ASSERT_EQUAL_INT16(2, out[0].v[0]);
}

void test_ring_buffer_peek_then_drop() {
  RingBuffer<4> rb;
  for (int i = 0; i < 4; ++i) rb.push(make_sample(static_cast<int16_t>(i * 10)));
  Sample out[8];
  rb.pop(out, 2);  // head now at index 2, so peeks below wrap around
  rb.push(make_sample(40));
  rb.push(make_sample(50));

  // peek copies without consuming
  TEST_ASSERT_EQUAL_UINT32(3, rb.peek(out, 3));
  TEST_ASSERT_EQUAL_INT16(20, out[0].v[0]);
  TEST_ASSERT_EQUAL_INT16(30, out[1].v[0]);
  TEST_ASSERT_EQUAL_INT16(40, out[2].v[0]);
  TEST_ASSERT_EQUAL_UINT32(4, rb.size());
  TEST_ASSERT_EQUAL_UINT32(4, rb.peek(out, 8));
  TEST_ASSERT_EQUAL_INT16(50, out[3].v[0]);

  // drop discards from the front; clamps to size
  TEST_ASSERT_EQUAL_UINT32(3, rb.drop(3));
  TEST_ASSERT_EQUAL_UINT32(1, rb.size());
  TEST_ASSERT_EQUAL_UINT32(1, rb.peek(out, 8));
  TEST_ASSERT_EQUAL_INT16(50, out[0].v[0]);
  TEST_ASSERT_EQUAL_UINT32(1, rb.drop(5));
  TEST_ASSERT_EQUAL_UINT32(0, rb.size());
  TEST_ASSERT_EQUAL_UINT32(0, rb.peek(out, 8));
}

void test_segments_contiguous_and_gaps() {
  Segments<3> sg;
  TEST_ASSERT_TRUE(sg.empty());
  TEST_ASSERT_EQUAL_UINT32(0, sg.front_count());
  for (uint32_t i = 10; i < 15; ++i) {
    TEST_ASSERT_TRUE(sg.can_append(i));
    sg.append(i);
  }
  TEST_ASSERT_EQUAL_UINT32(10, sg.front_first());
  TEST_ASSERT_EQUAL_UINT32(5, sg.front_count());

  // 15..16 dropped; 17 starts a second segment, 30 a third
  sg.append(17);
  sg.append(18);
  sg.append(30);
  TEST_ASSERT_TRUE(sg.can_append(31));    // extends the back segment
  TEST_ASSERT_FALSE(sg.can_append(40));   // would need a 4th segment
  sg.append(31);

  sg.consume(3);
  TEST_ASSERT_EQUAL_UINT32(13, sg.front_first());
  TEST_ASSERT_EQUAL_UINT32(2, sg.front_count());
  sg.consume(2);  // first segment exhausted
  TEST_ASSERT_EQUAL_UINT32(17, sg.front_first());
  TEST_ASSERT_EQUAL_UINT32(2, sg.front_count());
  TEST_ASSERT_TRUE(sg.can_append(40));    // a slot is free again
  sg.append(40);
  sg.consume(2);
  TEST_ASSERT_EQUAL_UINT32(30, sg.front_first());
  TEST_ASSERT_EQUAL_UINT32(2, sg.front_count());
  sg.consume(2);
  TEST_ASSERT_EQUAL_UINT32(40, sg.front_first());
  TEST_ASSERT_EQUAL_UINT32(1, sg.front_count());
  sg.consume(1);
  TEST_ASSERT_TRUE(sg.empty());

  sg.append(5);
  sg.clear();
  TEST_ASSERT_TRUE(sg.empty());
  sg.append(0);  // after clear, index 0 starts fresh
  TEST_ASSERT_EQUAL_UINT32(0, sg.front_first());
  TEST_ASSERT_EQUAL_UINT32(1, sg.front_count());
}

void test_estimate_lost_frames() {
  // 400 Hz for 250 ms = 100 frames produced; 85 read -> 15 lost
  TEST_ASSERT_EQUAL_UINT32(15, estimate_lost_frames(1000000, 1250000, 400, 85));
  // rounding to the nearest frame: 251.2 ms -> 100.48 -> 100
  TEST_ASSERT_EQUAL_UINT32(15, estimate_lost_frames(0, 251200, 400, 85));
  // 251.3 ms -> 100.52 -> 101
  TEST_ASSERT_EQUAL_UINT32(16, estimate_lost_frames(0, 251300, 400, 85));
  // loss was reported, so at least one frame is assumed missing
  TEST_ASSERT_EQUAL_UINT32(1, estimate_lost_frames(0, 212500, 400, 85));
  TEST_ASSERT_EQUAL_UINT32(1, estimate_lost_frames(0, 100000, 400, 85));  // more read than expected
  TEST_ASSERT_EQUAL_UINT32(1, estimate_lost_frames(500, 400, 400, 0));    // time went backwards
  // nothing read at all (e.g. I2C error + flush): every expected frame is lost
  TEST_ASSERT_EQUAL_UINT32(80, estimate_lost_frames(0, 100000, 800, 0));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_encode_imu_golden);
  RUN_TEST(test_encode_imu_full_packet_length);
  RUN_TEST(test_encode_status_golden);
  RUN_TEST(test_parse_set_config);
  RUN_TEST(test_parse_set_config_invalid_values);
  RUN_TEST(test_parse_simple_ops);
  RUN_TEST(test_parse_invalid);
  RUN_TEST(test_max_samples_for_mtu);
  RUN_TEST(test_ring_buffer_fifo_and_full);
  RUN_TEST(test_ring_buffer_wraparound_and_clear);
  RUN_TEST(test_ring_buffer_peek_then_drop);
  RUN_TEST(test_segments_contiguous_and_gaps);
  RUN_TEST(test_estimate_lost_frames);
  return UNITY_END();
}
