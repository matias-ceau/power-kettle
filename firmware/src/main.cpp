// Power Kettle firmware: BMI160 FIFO -> ring buffer -> BLE notifications.
#include <Arduino.h>
#include <Wire.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "ble_link.h"
#include "bmi160.h"
#include "board.h"
#include "config.h"
#include "fifo_loss.h"
#include "proto.h"
#include "ring_buffer.h"
#include "segments.h"

namespace {

Bmi160 imu;
BleLink ble;
pk::RingBuffer<kRingSamples> ring;
// Index runs held in `ring` (gaps appear when the ring is full); 8 runs is
// plenty: once all are used, further samples are dropped until one drains.
pk::Segments<8> segments;
QueueHandle_t cmd_queue;

volatile bool imu_irq = false;
bool imu_ok = false;

// Sensor configuration currently applied.
uint16_t odr_hz = kDefaultOdrHz;
uint8_t acc_range_g = kDefaultAccRangeG;
uint16_t gyro_range_dps = kDefaultGyroRangeDps;

// Stream state (reset by `start`).
bool streaming = false;
uint16_t seq = 0;
uint32_t sample_index = 0;  // index of the next sample to be produced
uint32_t dropped = 0;
uint64_t time_us = 0;  // esp_timer time of the read that produced sample_index-1
bool overflow_pending = false;
uint64_t prev_read_us = 0;  // FIFO snapshot time of the previous read (or start flush)

uint32_t last_poll_ms = 0;
uint32_t last_send_ms = 0;
uint32_t last_status_ms = 0;
uint32_t last_imu_retry_ms = 0;
bool status_due = false;
uint16_t battery_mv = 0;

void IRAM_ATTR on_imu_int1() { imu_irq = true; }

// Runs on the NimBLE host task: only hand the command to the loop.
void on_command(const pk::Command& c) { xQueueSend(cmd_queue, &c, 0); }

bool init_imu() {
  if (!imu.begin(Wire, kBmi160Addr)) return false;
  if (!imu.configure(odr_hz, acc_range_g, gyro_range_dps)) return false;
  Serial.printf("[imu] config %u Hz, +-%u g, +-%u dps, watermark %u frames\n", odr_hz,
                acc_range_g, gyro_range_dps, kFifoWatermarkFrames);
  return true;
}

void start_stream() {
  if (!imu_ok) {
    Serial.println("[cmd] start ignored: IMU not available");
    return;
  }
  imu.flush_fifo();
  prev_read_us = static_cast<uint64_t>(esp_timer_get_time());
  ring.clear();
  segments.clear();
  seq = 0;
  sample_index = 0;
  dropped = 0;
  time_us = 0;
  overflow_pending = false;
  imu_irq = false;
  streaming = true;
  last_poll_ms = last_send_ms = millis();
  Serial.println("[cmd] start");
}

void apply_config(const pk::Command& c) {
  // A run must not mix configurations: stop streaming; the client sends
  // `start` again, which begins a new run from index 0.
  streaming = false;
  if (imu.configure(c.odr_hz, c.acc_range_g, c.gyro_range_dps)) {
    odr_hz = c.odr_hz;
    acc_range_g = c.acc_range_g;
    gyro_range_dps = c.gyro_range_dps;
    Serial.printf("[cmd] config %u Hz, +-%u g, +-%u dps\n", odr_hz, acc_range_g, gyro_range_dps);
  } else {
    Serial.println("[cmd] set_config failed; restoring previous config");
    imu_ok = imu.configure(odr_hz, acc_range_g, gyro_range_dps);
  }
}

void process_commands() {
  pk::Command c;
  while (xQueueReceive(cmd_queue, &c, 0) == pdTRUE) {
    switch (c.op) {
      case pk::Op::Start:
        start_stream();
        break;
      case pk::Op::Stop:
        streaming = false;
        Serial.println("[cmd] stop");
        break;
      case pk::Op::SetConfig:
        apply_config(c);
        break;
      case pk::Op::Ping:
        break;
      case pk::Op::Invalid:
      default:
        Serial.println("[cmd] invalid command rejected");
        break;
    }
    status_due = true;  // status follows every command
  }
}

void read_imu() {
  static pk::Sample frames[Bmi160::kMaxFifoFrames + 1];
  size_t n = imu.read_frames(frames, Bmi160::kMaxFifoFrames + 1);
  uint64_t t_read = imu.last_read_time_us();
  uint32_t lost = 0;
  if (imu.last_read_lost_data()) {
    // Frames lost inside the BMI160 (overflow discards the oldest; an I2C
    // error flushes everything) precede the frames just read. Skip their
    // indices so the gap is explicit in first_index and index <-> time_us
    // stays aligned for the rest of the run.
    lost = pk::estimate_lost_frames(prev_read_us, t_read, odr_hz, n);
    sample_index += lost;
    dropped += lost;
    overflow_pending = true;
    Serial.printf("[imu] FIFO loss: ~%u frames skipped\n", static_cast<unsigned>(lost));
  }
  prev_read_us = t_read;
  for (size_t i = 0; i < n; ++i) {
    uint32_t idx = sample_index + static_cast<uint32_t>(i);
    if (segments.can_append(idx) && ring.push(frames[i])) {
      segments.append(idx);
    } else {
      ++dropped;
      overflow_pending = true;
    }
  }
  sample_index += static_cast<uint32_t>(n);
  if (n > 0 || lost > 0) time_us = t_read;
}

void send_packets(uint32_t now) {
  size_t max_n = ble.max_samples();
  if (max_n == 0) return;
  pk::Sample batch[pk::kMaxSamples];
  uint8_t pkt[pk::kImuHeader + pk::kMaxSamples * pk::kSampleBytes];
  for (int k = 0; k < kMaxPacketsPerLoop; ++k) {
    size_t avail = ring.size();
    if (avail == 0) break;
    if (avail < max_n && now - last_send_ms < kSendMaxAgeMs) break;
    // A packet covers consecutive indices only, so it stops at a gap.
    size_t n = avail < max_n ? avail : max_n;
    if (n > segments.front_count()) n = segments.front_count();
    ring.peek(batch, n);
    size_t len = pk::encode_imu(pkt, seq, segments.front_first(), overflow_pending ? 1 : 0,
                                batch, n);
    if (!ble.send_imu(pkt, len)) break;  // congested: keep samples, retry later
    ring.drop(n);
    segments.consume(n);
    ++seq;
    overflow_pending = false;
    last_send_ms = now;
  }
}

void send_status() {
  battery_mv = read_battery_mv();
  pk::StatusFields f{};
  f.flags = static_cast<uint8_t>((streaming ? 1 : 0) | (battery_mv < kLowBatteryMv ? 2 : 0));
  f.battery_mv = battery_mv;
  f.odr_hz = odr_hz;
  f.acc_range_g = acc_range_g;
  f.gyro_range_dps = gyro_range_dps;
  f.temp_raw = imu_ok ? imu.read_temp_raw() : static_cast<int16_t>(0x8000);
  f.dropped = dropped;
  f.sample_index = sample_index;
  f.time_us = time_us;
  uint8_t buf[pk::kStatusLen];
  pk::encode_status(buf, f);
  ble.send_status(buf, sizeof(buf));
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(500);  // give USB CDC a moment so the boot log is visible
  Serial.println("\n[boot] Power Kettle");

  board_begin();
  cmd_queue = xQueueCreate(8, sizeof(pk::Command));

  Wire.begin(kPinSda, kPinScl, kI2cClockHz);
  imu_ok = init_imu();
  if (!imu_ok) Serial.println("[imu] init failed; retrying every second");

  pinMode(kPinImuInt1, INPUT);
  attachInterrupt(digitalPinToInterrupt(kPinImuInt1), on_imu_int1, RISING);

  ble.begin(on_command);
  battery_mv = read_battery_mv();
  Serial.printf("[boot] battery %u mV\n", battery_mv);
}

void loop() {
  uint32_t now = millis();

  process_commands();

  if (streaming && !ble.connected()) {
    // Advertising is restarted by BleLink's disconnect callback.
    streaming = false;
    Serial.println("[ble] link lost: streaming stopped");
  }

  if (!imu_ok && now - last_imu_retry_ms >= 1000) {
    last_imu_retry_ms = now;
    imu_ok = init_imu();
  }

  if (streaming) {
    if (imu_irq || now - last_poll_ms >= kFifoPollMs) {
      imu_irq = false;
      last_poll_ms = now;
      read_imu();
    }
    send_packets(now);
  } else {
    imu_irq = false;
  }

  if (status_due || now - last_status_ms >= kStatusPeriodMs) {
    status_due = false;
    last_status_ms = now;
    send_status();
  }

  if (battery_mv < kLowBatteryMv) {
    led_set(LedMode::LowBattery);
  } else {
    led_set(ble.connected() ? LedMode::Connected : LedMode::Advertising);
  }

  delay(1);  // yield to the idle task (keeps the task watchdog fed)
}
