#include <Arduino.h>

#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#include <cstring>
#include <string>

#ifndef BRIDGE_RX_PIN
#define BRIDGE_RX_PIN 20
#endif

#ifndef BRIDGE_TX_PIN
#define BRIDGE_TX_PIN 21
#endif

#ifndef BRIDGE_UART_BAUD
#define BRIDGE_UART_BAUD 115200
#endif

class Esp32BridgeApp {
public:
  Esp32BridgeApp()
#if defined(CONFIG_IDF_TARGET_ESP32C3)
      : bridge_uart_(1), server_callbacks_(this), downlink_callbacks_(this) {}
#else
      : bridge_uart_(2), server_callbacks_(this), downlink_callbacks_(this) {}
#endif

  void Setup() {
    Serial.begin(kUsbSerialBaud);
    const uint32_t wait_start_ms = millis();
    while (!Serial && (millis() - wait_start_ms < 3000U)) {
      delay(10);
    }

    Serial.println();
    Serial.println("Starting ESP32 BLE/UART guidance bridge...");

    BeginBridgeUart();
    SetupBle();
    SetStatus("bridge ready", false);

    Serial.println("BLE bridge ready");
  }

  void Loop() {
    ProcessPendingDownlink();
    ProcessBridgeUart();
    MaintainAdvertising();
    PublishPeriodicStatus();
    delay(2);
  }

private:
  static constexpr char kDeviceName[] = "Dart_Guidance_Bridge";
  static constexpr char kServiceUuid[] = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
  static constexpr char kDownlinkCharacteristicUuid[] = "beb5483e-36e1-4688-b7f5-ea07361b26a8";
  static constexpr char kUplinkCharacteristicUuid[] = "9f6c1db5-0b3b-4d1d-8a4d-11dd5c3a4f21";
  static constexpr char kStatusCharacteristicUuid[] = "3d7f7b30-5602-4f2d-9d45-1f1be1b4c001";
  static constexpr uint32_t kUsbSerialBaud = 115200U;
  static constexpr uint32_t kStatusUpdateIntervalMs = 1000U;
  static constexpr uint32_t kDisconnectRestartDelayMs = 300U;
  static constexpr size_t kMaxBlePacketSize = 32U;
  static constexpr size_t kMaxUplinkFrameSize = 32U;
  static constexpr uint8_t kFrameHeader0 = 0xA5U;
  static constexpr uint8_t kFrameHeader1 = 0x5AU;

  class ServerCallbacks : public BLEServerCallbacks {
  public:
    explicit ServerCallbacks(Esp32BridgeApp* app) : app_(app) {}

    void onConnect(BLEServer* server) override {
      (void)server;
      app_->HandleBleConnected();
    }

    void onDisconnect(BLEServer* server) override {
      (void)server;
      app_->HandleBleDisconnected();
    }

  private:
    Esp32BridgeApp* app_;
  };

  class DownlinkCallbacks : public BLECharacteristicCallbacks {
  public:
    explicit DownlinkCallbacks(Esp32BridgeApp* app) : app_(app) {}

    void onWrite(BLECharacteristic* characteristic) override {
      app_->HandleBleWrite(characteristic);
    }

  private:
    Esp32BridgeApp* app_;
  };

  struct BridgeStats {
    uint32_t downlink_packets = 0U;
    uint32_t downlink_bytes = 0U;
    uint32_t uplink_frames = 0U;
    uint32_t uplink_bytes = 0U;
    uint32_t checksum_errors = 0U;
    uint32_t dropped_ble_packets = 0U;
    uint32_t dropped_uart_frames = 0U;
    uint32_t last_downlink_ms = 0U;
    uint32_t last_uplink_ms = 0U;
  };

  HardwareSerial bridge_uart_;
  BLEServer* ble_server_ = nullptr;
  BLECharacteristic* downlink_characteristic_ = nullptr;
  BLECharacteristic* uplink_characteristic_ = nullptr;
  BLECharacteristic* status_characteristic_ = nullptr;
  bool ble_connected_ = false;
  bool was_ble_connected_ = false;
  uint32_t last_status_update_ms_ = 0U;
  uint32_t disconnect_timestamp_ms_ = 0U;

  uint8_t pending_downlink_[kMaxBlePacketSize] = {0U};
  size_t pending_downlink_size_ = 0U;
  bool pending_downlink_ready_ = false;

  uint8_t uplink_frame_[kMaxUplinkFrameSize] = {0U};
  size_t uplink_frame_index_ = 0U;
  size_t uplink_expected_size_ = 0U;

  BridgeStats stats_;
  ServerCallbacks server_callbacks_;
  DownlinkCallbacks downlink_callbacks_;

  void BeginBridgeUart() {
    bridge_uart_.begin(static_cast<uint32_t>(BRIDGE_UART_BAUD),
                       SERIAL_8N1,
                       BRIDGE_RX_PIN,
                       BRIDGE_TX_PIN);
    Serial.printf("Bridge UART started: RX=%d TX=%d BAUD=%d\n",
                  BRIDGE_RX_PIN,
                  BRIDGE_TX_PIN,
                  BRIDGE_UART_BAUD);
  }

  void SetupBle() {
    BLEDevice::init(kDeviceName);
    BLEDevice::setMTU(64);

    ble_server_ = BLEDevice::createServer();
    ble_server_->setCallbacks(&server_callbacks_);

    BLEService* service = ble_server_->createService(kServiceUuid);

    downlink_characteristic_ = service->createCharacteristic(
        kDownlinkCharacteristicUuid,
        BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_WRITE |
            BLECharacteristic::PROPERTY_WRITE_NR);
    downlink_characteristic_->addDescriptor(new BLE2902());
    downlink_characteristic_->setCallbacks(&downlink_callbacks_);
    downlink_characteristic_->setValue("downlink ready");

    uplink_characteristic_ = service->createCharacteristic(
        kUplinkCharacteristicUuid,
        BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_NOTIFY);
    uplink_characteristic_->addDescriptor(new BLE2902());
    uplink_characteristic_->setValue(std::string());

    status_characteristic_ = service->createCharacteristic(
        kStatusCharacteristicUuid,
        BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_NOTIFY);
    status_characteristic_->addDescriptor(new BLE2902());
    status_characteristic_->setValue("status init");

    service->start();

    BLEAdvertising* advertising = BLEDevice::getAdvertising();
    advertising->addServiceUUID(kServiceUuid);
    advertising->setScanResponse(false);
    advertising->setMinPreferred(0x0);
    BLEDevice::startAdvertising();
  }

  void HandleBleConnected() {
    ble_connected_ = true;
    was_ble_connected_ = true;
    Serial.println("BLE client connected");
    SetStatus("ble connected", true);
  }

  void HandleBleDisconnected() {
    ble_connected_ = false;
    disconnect_timestamp_ms_ = millis();
    Serial.println("BLE client disconnected");
    SetStatus("ble disconnected", false);
  }

  void HandleBleWrite(BLECharacteristic* characteristic) {
    const std::string value = characteristic->getValue();
    if (value.empty()) {
      return;
    }

    if (value.size() > kMaxBlePacketSize) {
      stats_.dropped_ble_packets += 1U;
      Serial.printf("Dropped BLE packet: %u bytes exceeds %u\n",
                    static_cast<unsigned>(value.size()),
                    static_cast<unsigned>(kMaxBlePacketSize));
      SetStatus("ble packet too large", true);
      return;
    }

    if (pending_downlink_ready_) {
      stats_.dropped_ble_packets += 1U;
      Serial.println("Dropped BLE packet: pending buffer busy");
      SetStatus("ble packet dropped: busy", true);
      return;
    }

    memcpy(pending_downlink_, value.data(), value.size());
    pending_downlink_size_ = value.size();
    pending_downlink_ready_ = true;
  }

  void ProcessPendingDownlink() {
    if (!pending_downlink_ready_) {
      return;
    }

    const size_t written = bridge_uart_.write(pending_downlink_, pending_downlink_size_);
    bridge_uart_.flush();

    if (written == pending_downlink_size_) {
      stats_.downlink_packets += 1U;
      stats_.downlink_bytes += static_cast<uint32_t>(written);
      stats_.last_downlink_ms = millis();
      Serial.printf("Forwarded BLE->UART packet: %u bytes\n",
                    static_cast<unsigned>(written));
    } else {
      stats_.dropped_ble_packets += 1U;
      Serial.printf("Partial BLE->UART write: %u/%u bytes\n",
                    static_cast<unsigned>(written),
                    static_cast<unsigned>(pending_downlink_size_));
      SetStatus("uart write partial", true);
    }

    pending_downlink_size_ = 0U;
    pending_downlink_ready_ = false;
  }

  void ProcessBridgeUart() {
    while (bridge_uart_.available() > 0) {
      const uint8_t byte_value = static_cast<uint8_t>(bridge_uart_.read());
      ConsumeUartByte(byte_value);
    }
  }

  void ConsumeUartByte(uint8_t byte_value) {
    if (uplink_frame_index_ == 0U) {
      if (byte_value == kFrameHeader0) {
        uplink_frame_[uplink_frame_index_++] = byte_value;
      }
      return;
    }

    if (uplink_frame_index_ == 1U) {
      if (byte_value == kFrameHeader1) {
        uplink_frame_[uplink_frame_index_++] = byte_value;
        return;
      }

      uplink_frame_index_ = (byte_value == kFrameHeader0) ? 1U : 0U;
      uplink_frame_[0] = kFrameHeader0;
      return;
    }

    if (uplink_frame_index_ >= kMaxUplinkFrameSize) {
      ResetUplinkAssembler();
      stats_.dropped_uart_frames += 1U;
      SetStatus("uart frame overflow", true);
      return;
    }

    uplink_frame_[uplink_frame_index_++] = byte_value;
    if (uplink_frame_index_ == 4U) {
      if (uplink_frame_[3] > (kMaxUplinkFrameSize - 5U)) {
        ResetUplinkAssembler();
        stats_.dropped_uart_frames += 1U;
        SetStatus("uart payload too large", true);
        return;
      }
      uplink_expected_size_ = static_cast<size_t>(uplink_frame_[3]) + 5U;
    }

    if ((uplink_expected_size_ > 0U) && (uplink_frame_index_ >= uplink_expected_size_)) {
      HandleCompletedUartFrame();
      ResetUplinkAssembler();
    }
  }

  void HandleCompletedUartFrame() {
    if (!FrameChecksumIsValid(uplink_frame_, uplink_expected_size_)) {
      stats_.checksum_errors += 1U;
      Serial.println("UART frame checksum error");
      SetStatus("uart checksum error", true);
      return;
    }

    stats_.uplink_frames += 1U;
    stats_.uplink_bytes += static_cast<uint32_t>(uplink_expected_size_);
    stats_.last_uplink_ms = millis();

    Serial.printf("Forwarded UART->BLE frame: type=0x%02X size=%u\n",
                  static_cast<unsigned>(uplink_frame_[2]),
                  static_cast<unsigned>(uplink_expected_size_));

    if (uplink_characteristic_ != nullptr) {
      uplink_characteristic_->setValue(uplink_frame_, uplink_expected_size_);
      if (ble_connected_) {
        uplink_characteristic_->notify();
      }
    }
  }

  static bool FrameChecksumIsValid(const uint8_t* frame, size_t frame_size)  {
    uint8_t checksum = 0U;
    if ((frame == nullptr) || (frame_size < 5U)) {
      return false;
    }

    for (size_t index = 0U; index + 1U < frame_size; ++index) {
      checksum = static_cast<uint8_t>(checksum + frame[index]);
    }
    return checksum == frame[frame_size - 1U];
  }

  void ResetUplinkAssembler() {
    uplink_frame_index_ = 0U;
    uplink_expected_size_ = 0U;
  }

  void MaintainAdvertising() {
    if (!ble_connected_ && was_ble_connected_) {
      if (millis() - disconnect_timestamp_ms_ >= kDisconnectRestartDelayMs) {
        ble_server_->startAdvertising();
        was_ble_connected_ = false;
        Serial.println("BLE advertising restarted");
      }
    }
  }

  void PublishPeriodicStatus() {
    if (millis() - last_status_update_ms_ < kStatusUpdateIntervalMs) {
      return;
    }

    last_status_update_ms_ = millis();
    SetStatus(BuildStatusSnapshot(), ble_connected_);
  }

  String BuildStatusSnapshot() const {
    String status;
    status.reserve(160);
    status += "ble=";
    status += ble_connected_ ? "1" : "0";
    status += ",rx=";
    status += String(BRIDGE_RX_PIN);
    status += ",tx=";
    status += String(BRIDGE_TX_PIN);
    status += ",baud=";
    status += String(BRIDGE_UART_BAUD);
    status += ",dl_pkt=";
    status += String(stats_.downlink_packets);
    status += ",dl_b=";
    status += String(stats_.downlink_bytes);
    status += ",ul_fr=";
    status += String(stats_.uplink_frames);
    status += ",ul_b=";
    status += String(stats_.uplink_bytes);
    status += ",crc=";
    status += String(stats_.checksum_errors);
    status += ",drop_ble=";
    status += String(stats_.dropped_ble_packets);
    status += ",drop_uart=";
    status += String(stats_.dropped_uart_frames);
    return status;
  }

  void SetStatus(const String& message, bool notify_client) {
    if (status_characteristic_ == nullptr) {
      return;
    }

    status_characteristic_->setValue(message.c_str());
    if (notify_client && ble_connected_) {
      status_characteristic_->notify();
    }
  }
};

constexpr char Esp32BridgeApp::kDeviceName[];
constexpr char Esp32BridgeApp::kServiceUuid[];
constexpr char Esp32BridgeApp::kDownlinkCharacteristicUuid[];
constexpr char Esp32BridgeApp::kUplinkCharacteristicUuid[];
constexpr char Esp32BridgeApp::kStatusCharacteristicUuid[];

Esp32BridgeApp g_esp32_bridge_app;

void setup() {
  g_esp32_bridge_app.Setup();
}

void loop() {
  g_esp32_bridge_app.Loop();
}
