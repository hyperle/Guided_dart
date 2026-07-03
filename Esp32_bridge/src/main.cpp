#include <Arduino.h>
#include <NimBLEDevice.h>
#include <esp_log.h>

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
    esp_log_level_set("*", ESP_LOG_NONE);

    BeginBridgeUart();
    SetupBle();
    SetStatus("bridge ready", false);
  }

  void Loop() {
    ProcessBridgeUart();
    MaintainAdvertising();
    PublishPeriodicStatus();
    delay(2);
  }

private:
  static constexpr char kDeviceName[] = "Dart_1";
  static constexpr char kServiceUuid[] = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
  static constexpr char kDownlinkCharacteristicUuid[] = "beb5483e-36e1-4688-b7f5-ea07361b26a8";
  static constexpr char kUplinkCharacteristicUuid[] = "9f6c1db5-0b3b-4d1d-8a4d-11dd5c3a4f21";
  static constexpr char kAckCharacteristicUuid[] = "de24d570-5f81-4d6b-8e4d-2f1309367d91";
  static constexpr char kStatusCharacteristicUuid[] = "3d7f7b30-5602-4f2d-9d45-1f1be1b4c001";
  static constexpr uint32_t kUsbSerialBaud = 115200U;
  static constexpr uint32_t kStatusUpdateIntervalMs = 1000U;
  static constexpr uint32_t kDisconnectRestartDelayMs = 300U;
  static constexpr size_t kMaxBlePacketSize = 256U;
  static constexpr size_t kMaxUplinkFrameSize = 256U;
  static constexpr uint8_t kFrameHeader0 = 0xA5U;
  static constexpr uint8_t kFrameHeader1 = 0x5AU;
  static constexpr uint8_t kMessageTypeParamAck = 0x03U;

  class ServerCallbacks : public NimBLEServerCallbacks {
  public:
    explicit ServerCallbacks(Esp32BridgeApp* app) : app_(app) {}

    void onConnect(NimBLEServer* server, ble_gap_conn_desc* desc) override {
      (void)server; (void)desc;
      app_->HandleBleConnected();
    }

    void onDisconnect(NimBLEServer* server, ble_gap_conn_desc* desc) override {
      (void)server; (void)desc;
      app_->HandleBleDisconnected();
    }

  private:
    Esp32BridgeApp* app_;
  };

  class DownlinkCallbacks : public NimBLECharacteristicCallbacks {
  public:
    explicit DownlinkCallbacks(Esp32BridgeApp* app) : app_(app) {}

    void onWrite(NimBLECharacteristic* characteristic, ble_gap_conn_desc* desc) override {
      (void)desc;
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
  NimBLEServer* ble_server_ = nullptr;
  NimBLECharacteristic* downlink_characteristic_ = nullptr;
  NimBLECharacteristic* uplink_characteristic_ = nullptr;
  NimBLECharacteristic* ack_characteristic_ = nullptr;
  NimBLECharacteristic* status_characteristic_ = nullptr;
  bool ble_connected_ = false;
  bool was_ble_connected_ = false;
  uint32_t last_status_update_ms_ = 0U;
  uint32_t disconnect_timestamp_ms_ = 0U;

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
    if (Serial) {
      Serial.printf("Bridge UART started: RX=%d TX=%d BAUD=%d\n",
                    BRIDGE_RX_PIN,
                    BRIDGE_TX_PIN,
                    BRIDGE_UART_BAUD);
    }
  }

  void SetupBle() {
    NimBLEDevice::init(kDeviceName);
    NimBLEDevice::setMTU(247);

    ble_server_ = NimBLEDevice::createServer();
    ble_server_->setCallbacks(&server_callbacks_);

    NimBLEService* service = ble_server_->createService(kServiceUuid);

    downlink_characteristic_ = service->createCharacteristic(
        kDownlinkCharacteristicUuid,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR | NIMBLE_PROPERTY::NOTIFY);
    downlink_characteristic_->setCallbacks(&downlink_callbacks_);
    downlink_characteristic_->setValue((const uint8_t*)"downlink ready", 13);

    uplink_characteristic_ = service->createCharacteristic(
        kUplinkCharacteristicUuid,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    uplink_characteristic_->setValue(std::string());

    ack_characteristic_ = service->createCharacteristic(
        kAckCharacteristicUuid,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    ack_characteristic_->setValue(std::string());

    status_characteristic_ = service->createCharacteristic(
        kStatusCharacteristicUuid,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    status_characteristic_->setValue((const uint8_t*)"status init", 11);

    service->start();

    NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();
    advertising->addServiceUUID(kServiceUuid);
    advertising->setScanResponse(false);
    advertising->setMinPreferred(0x20);
    advertising->setMaxPreferred(0x40);
    advertising->start();
  }

  void HandleBleConnected() {
    ble_connected_ = true;
    was_ble_connected_ = true;
    if (Serial) {
      Serial.println("BLE client connected");
    }
    SetStatus("ble connected", true);
  }

  void HandleBleDisconnected() {
    ble_connected_ = false;
    disconnect_timestamp_ms_ = millis();
    if (Serial) {
      Serial.println("BLE client disconnected");
    }
    SetStatus("ble disconnected", false);
  }

  void HandleBleWrite(NimBLECharacteristic* characteristic) {
    const std::string value = characteristic->getValue();
    if (value.empty()) {
      return;
    }

    if (value.size() > kMaxBlePacketSize) {
      stats_.dropped_ble_packets += 1U;
      if (Serial) {
        Serial.printf("Dropped BLE packet: %u bytes exceeds %u\n",
                      static_cast<unsigned>(value.size()),
                      static_cast<unsigned>(kMaxBlePacketSize));
      }
      SetStatus("ble packet too large", true);
      return;
    }

    ForwardBlePacketToUart(reinterpret_cast<const uint8_t*>(value.data()), value.size());
  }

  void ForwardBlePacketToUart(const uint8_t* packet, size_t packet_size) {
    if ((packet == nullptr) || (packet_size == 0U)) {
      return;
    }

    const size_t written = bridge_uart_.write(packet, packet_size);
    bridge_uart_.flush();

    if (written == packet_size) {
      stats_.downlink_packets += 1U;
      stats_.downlink_bytes += static_cast<uint32_t>(written);
      stats_.last_downlink_ms = millis();
      if (Serial) {
        Serial.printf("Forwarded BLE->UART packet: %u bytes\n",
                      static_cast<unsigned>(written));
      }
    } else {
      stats_.dropped_ble_packets += 1U;
      if (Serial) {
        Serial.printf("Partial BLE->UART write: %u/%u bytes\n",
                      static_cast<unsigned>(written),
                      static_cast<unsigned>(packet_size));
      }
      SetStatus("uart write partial", true);
    }
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
      if (Serial) {
        Serial.println("UART frame checksum error");
      }
      SetStatus("uart checksum error", true);
      return;
    }

    stats_.uplink_frames += 1U;
    stats_.uplink_bytes += static_cast<uint32_t>(uplink_expected_size_);
    stats_.last_uplink_ms = millis();

    if (Serial) {
      Serial.printf("Forwarded UART->BLE frame: type=0x%02X size=%u\n",
                    static_cast<unsigned>(uplink_frame_[2]),
                    static_cast<unsigned>(uplink_expected_size_));
    }

    NotifyFrame(uplink_characteristic_, uplink_frame_, uplink_expected_size_);
    if (uplink_frame_[2] == kMessageTypeParamAck) {
      NotifyFrame(ack_characteristic_, uplink_frame_, uplink_expected_size_);
    }
  }

  void NotifyFrame(NimBLECharacteristic* characteristic, const uint8_t* frame, size_t frame_size) {
    if ((characteristic == nullptr) || (frame == nullptr) || (frame_size == 0U)) {
      return;
    }

    characteristic->setValue(frame, frame_size);
    if (ble_connected_) {
      characteristic->notify();
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
        if (Serial) {
          Serial.println("BLE advertising restarted");
        }
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

    status_characteristic_->setValue(message);
    if (notify_client && ble_connected_) {
      status_characteristic_->notify();
    }
  }
};

constexpr char Esp32BridgeApp::kDeviceName[];
constexpr char Esp32BridgeApp::kServiceUuid[];
constexpr char Esp32BridgeApp::kDownlinkCharacteristicUuid[];
constexpr char Esp32BridgeApp::kUplinkCharacteristicUuid[];
constexpr char Esp32BridgeApp::kAckCharacteristicUuid[];
constexpr char Esp32BridgeApp::kStatusCharacteristicUuid[];

Esp32BridgeApp g_esp32_bridge_app;

void setup() {
  g_esp32_bridge_app.Setup();
}

void loop() {
  g_esp32_bridge_app.Loop();
}
