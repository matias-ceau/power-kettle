#include "ble_link.h"

#include <Arduino.h>
#include <NimBLEDevice.h>

#include <atomic>

#include "config.h"

namespace {

NimBLECharacteristic* g_imu = nullptr;
NimBLECharacteristic* g_status = nullptr;
CommandHandler g_on_command = nullptr;

// Written from the NimBLE host task, read from the loop task.
std::atomic<uint16_t> g_conn{BLE_HS_CONN_HANDLE_NONE};
std::atomic<uint16_t> g_mtu{0};

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer*, NimBLEConnInfo& info) override {
    g_mtu = info.getMTU();  // 23 until the central runs the MTU exchange
    g_conn = info.getConnHandle();
    Serial.printf("[ble] connected %s, mtu %u\n", info.getAddress().toString().c_str(),
                  info.getMTU());
  }

  void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int reason) override {
    g_conn = BLE_HS_CONN_HANDLE_NONE;
    g_mtu = 0;
    Serial.printf("[ble] disconnected, reason 0x%X; advertising\n", reason);
    NimBLEDevice::startAdvertising();
  }

  void onMTUChange(uint16_t mtu, NimBLEConnInfo& info) override {
    if (info.getConnHandle() == g_conn) g_mtu = mtu;
    Serial.printf("[ble] mtu %u -> %u samples/packet\n", mtu,
                  static_cast<unsigned>(pk::max_samples_for_mtu(mtu)));
  }
};

class ControlCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* chr, NimBLEConnInfo&) override {
    NimBLEAttValue v = chr->getValue();
    pk::Command c = pk::parse_command(v.data(), v.length());
    if (g_on_command) g_on_command(c);
  }
};

ServerCallbacks g_server_cb;
ControlCallbacks g_control_cb;

}  // namespace

void BleLink::begin(CommandHandler on_command) {
  g_on_command = on_command;

  NimBLEDevice::init(kDeviceName);
  NimBLEDevice::setMTU(kPreferredMtu);  // preferred ATT MTU offered in the exchange

  NimBLEServer* server = NimBLEDevice::createServer();
  server->setCallbacks(&g_server_cb);

  NimBLEService* svc = server->createService(kUuidService);
  g_imu = svc->createCharacteristic(kUuidImuStream, NIMBLE_PROPERTY::NOTIFY);
  NimBLECharacteristic* control =
      svc->createCharacteristic(kUuidControl, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  control->setCallbacks(&g_control_cb);
  g_status = svc->createCharacteristic(kUuidStatus, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  uint8_t zero[pk::kStatusLen] = {};
  g_status->setValue(zero, sizeof(zero));

  // Flags + 128-bit service UUID fill most of the 31-byte advertising
  // payload, so the name goes into the scan response.
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->enableScanResponse(true);
  adv->addServiceUUID(kUuidService);
  adv->setName(kDeviceName);
  NimBLEDevice::startAdvertising();
  Serial.println("[ble] advertising as " + String(kDeviceName));
}

bool BleLink::connected() const { return g_conn != BLE_HS_CONN_HANDLE_NONE; }

size_t BleLink::max_samples() const {
  if (!connected()) return 0;
  return pk::max_samples_for_mtu(g_mtu);
}

bool BleLink::send_imu(const uint8_t* data, size_t len) {
  uint16_t conn = g_conn;
  if (conn == BLE_HS_CONN_HANDLE_NONE) return false;
  // notify() returns false when NimBLE could not allocate/queue the packet
  // (mbuf pool exhausted while the link is congested).
  return g_imu->notify(data, len, conn);
}

void BleLink::send_status(const uint8_t* data, size_t len) {
  g_status->setValue(data, len);  // what a GATT read returns
  uint16_t conn = g_conn;
  if (conn != BLE_HS_CONN_HANDLE_NONE) g_status->notify(data, len, conn);
}
