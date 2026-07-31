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

#ifndef BRIDGE_TELEMETRY_NOTIFY_HZ
#define BRIDGE_TELEMETRY_NOTIFY_HZ 25
#endif

class Esp32BridgeApp {
public:
  Esp32BridgeApp()
#if defined(CONFIG_IDF_TARGET_ESP32C3)
      : bridge_uart_(1), server_callbacks_(this) {}
#else
      : bridge_uart_(2), server_callbacks_(this) {}
#endif

  void Setup() {
    esp_log_level_set("*", ESP_LOG_NONE);

    BeginBridgeUart();
    SetupBle();
    SetStatus("bridge ready", false);
  }

  void Loop() {
    ProcessBridgeUart();
    PublishTelemetrySnapshot();
    MaintainAdvertising();
    PublishPeriodicStatus();
    delay(2);
  }

private:
  static constexpr char kDeviceName[] = "Dart_1";
  static constexpr char kServiceUuid[] = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
  static constexpr char kTelemetryCharacteristicUuid[] = "7b3f5a10-2d36-4c8f-9a7f-3a2d542fd1b2";
  static constexpr char kStatusCharacteristicUuid[] = "3d7f7b30-5602-4f2d-9d45-1f1be1b4c001";

  static constexpr uint32_t kStatusUpdateIntervalMs = 1000U;
  static constexpr uint32_t kDisconnectRestartDelayMs = 300U;
  static constexpr uint32_t kSourceStaleMs = 500U;
#if BRIDGE_TELEMETRY_NOTIFY_HZ <= 0
  static constexpr uint32_t kTelemetryNotifyIntervalMs = 40U;
#else
  static constexpr uint32_t kTelemetryNotifyIntervalMs = 1000U / BRIDGE_TELEMETRY_NOTIFY_HZ;
#endif

  static constexpr size_t kMaxUplinkFrameSize = 256U;
  static constexpr uint8_t kFrameHeader0 = 0xA5U;
  static constexpr uint8_t kFrameHeader1 = 0x5AU;
  static constexpr uint8_t kFrameTypeGuidanceTelemetry = 0x01U;
  static constexpr uint8_t kFrameTypeImuAccel = 0x04U;
  static constexpr uint8_t kFrameTypeImuMotion = 0x05U;
  static constexpr uint8_t kFrameTypeDartLaunchSample = 0x06U;
  static constexpr uint8_t kGuidanceFlagTargetDetected = 1U << 0U;
  static constexpr uint16_t kNoTargetCoordinate = 0xFFFFU;

  static constexpr uint16_t kTelemetryMagic = 0xDA7AU;
  static constexpr uint8_t kTelemetryVersion = 2U;
  static constexpr uint8_t kTelemetryTypeSnapshot = 1U;
  static constexpr uint8_t kImuMotionPayloadLegacySize = 24U;
  static constexpr uint8_t kImuMotionPayloadExtendedSize = 40U;
  static constexpr uint8_t kDartLaunchSamplePayloadSize = 14U;
  static constexpr uint16_t kTelemetryFlagTargetValid = 1U << 0U;
  static constexpr uint16_t kTelemetryFlagGuidanceSeen = 1U << 1U;
  static constexpr uint16_t kTelemetryFlagMotionSeen = 1U << 2U;
  static constexpr uint16_t kTelemetryFlagAccelSeen = 1U << 3U;
  static constexpr uint16_t kTelemetryFlagTargetLost = 1U << 4U;
  static constexpr uint16_t kTelemetryFlagGuidanceStale = 1U << 5U;
  static constexpr uint16_t kTelemetryFlagMotionStale = 1U << 6U;

  struct __attribute__((packed)) TelemetryPacket {
    uint16_t magic;
    uint8_t version;
    uint8_t type;
    uint16_t sequence;
    uint16_t flags;
    uint32_t esp_time_ms;
    uint32_t source_time_ms;
    float target_x;
    float target_y;
    float velocity_x;
    float velocity_y;
    float velocity_z;
    float accel_x;
    float accel_y;
    float accel_z;
    uint16_t dart_launch_counter_ticks;
    float dart_launch_velocity_x;
    float dart_launch_velocity_y;
    float dart_launch_velocity_z;
    uint16_t crc16;
  };

  class ServerCallbacks : public NimBLEServerCallbacks {
  public:
    explicit ServerCallbacks(Esp32BridgeApp* app) : app_(app) {}

    void onConnect(NimBLEServer* server, ble_gap_conn_desc* desc) override {
      (void)server;
      app_->HandleBleConnected(desc);
    }

    void onDisconnect(NimBLEServer* server, ble_gap_conn_desc* desc) override {
      (void)server;
      (void)desc;
      app_->HandleBleDisconnected();
    }

  private:
    Esp32BridgeApp* app_;
  };

  struct BridgeStats {
    uint32_t uart_rx_bytes = 0U;
    uint32_t uart_header0_bytes = 0U;
    uint32_t uart_header_pairs = 0U;
    uint32_t uart_frames = 0U;
    uint32_t guidance_frames = 0U;
    uint32_t motion_frames = 0U;
    uint32_t accel_frames = 0U;
    uint32_t launch_sample_frames = 0U;
    uint32_t checksum_errors = 0U;
    uint32_t bad_payload_frames = 0U;
    uint32_t unknown_uart_frames = 0U;
    uint32_t dropped_uart_frames = 0U;
    uint32_t telemetry_notifications = 0U;
    uint8_t last_frame_type = 0U;
    uint8_t last_payload_size = 0U;
  };

  struct TelemetryState {
    bool guidance_seen = false;
    bool motion_seen = false;
    bool accel_seen = false;
    bool target_valid = false;
    uint32_t last_guidance_ms = 0U;
    uint32_t last_motion_ms = 0U;
    uint32_t last_accel_ms = 0U;
    float target_x = -1.0f;
    float target_y = -1.0f;
    float velocity_x = 0.0f;
    float velocity_y = 0.0f;
    float velocity_z = 0.0f;
    float accel_x = 0.0f;
    float accel_y = 0.0f;
    float accel_z = 0.0f;
    uint16_t dart_launch_counter_ticks = 0U;
    float dart_launch_velocity_x = 0.0f;
    float dart_launch_velocity_y = 0.0f;
    float dart_launch_velocity_z = 0.0f;
  };

  HardwareSerial bridge_uart_;
  NimBLEServer* ble_server_ = nullptr;
  NimBLECharacteristic* telemetry_characteristic_ = nullptr;
  NimBLECharacteristic* status_characteristic_ = nullptr;
  bool ble_connected_ = false;
  uint32_t last_status_update_ms_ = 0U;
  uint32_t disconnect_timestamp_ms_ = 0U;
  uint32_t last_telemetry_notify_ms_ = 0U;
  uint16_t telemetry_sequence_ = 0U;

  uint8_t uplink_frame_[kMaxUplinkFrameSize] = {0U};
  size_t uplink_frame_index_ = 0U;
  size_t uplink_expected_size_ = 0U;

  BridgeStats stats_;
  TelemetryState telemetry_;
  ServerCallbacks server_callbacks_;

  void BeginBridgeUart() {
    bridge_uart_.begin(static_cast<uint32_t>(BRIDGE_UART_BAUD),
                       SERIAL_8N1,
                       BRIDGE_RX_PIN,
                       BRIDGE_TX_PIN);
  }

  void SetupBle() {
    NimBLEDevice::init(kDeviceName);
    NimBLEDevice::setMTU(185);
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);

    ble_server_ = NimBLEDevice::createServer();
    ble_server_->setCallbacks(&server_callbacks_);

    NimBLEService* service = ble_server_->createService(kServiceUuid);

    telemetry_characteristic_ = service->createCharacteristic(
        kTelemetryCharacteristicUuid,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    telemetry_characteristic_->setValue(std::string());

    status_characteristic_ = service->createCharacteristic(
        kStatusCharacteristicUuid,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    status_characteristic_->setValue((const uint8_t*)"status init", 11);

    service->start();

    NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();
    advertising->addServiceUUID(kServiceUuid);
    advertising->setScanResponse(true);
    advertising->setMinPreferred(0x12);
    advertising->setMaxPreferred(0x24);
    advertising->start();
  }

  void HandleBleConnected(const ble_gap_conn_desc* desc) {
    ble_connected_ = true;
    last_telemetry_notify_ms_ = 0U;
    if ((ble_server_ != nullptr) && (desc != nullptr)) {
      ble_server_->updateConnParams(desc->conn_handle, 12, 24, 0, 200);
    }
    SetStatus("ble connected", true);
  }

  void HandleBleDisconnected() {
    ble_connected_ = false;
    disconnect_timestamp_ms_ = millis();
    SetStatus("ble disconnected", false);
    StartAdvertising();
  }

  void ProcessBridgeUart() {
    while (bridge_uart_.available() > 0) {
      const uint8_t byte_value = static_cast<uint8_t>(bridge_uart_.read());
      stats_.uart_rx_bytes += 1U;
      ConsumeUartByte(byte_value);
    }
  }

  void ConsumeUartByte(uint8_t byte_value) {
    if (uplink_frame_index_ == 0U) {
      if (byte_value == kFrameHeader0) {
        stats_.uart_header0_bytes += 1U;
        uplink_frame_[uplink_frame_index_++] = byte_value;
      }
      return;
    }

    if (uplink_frame_index_ == 1U) {
      if (byte_value == kFrameHeader1) {
        stats_.uart_header_pairs += 1U;
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
      stats_.last_frame_type = uplink_frame_[2];
      stats_.last_payload_size = uplink_frame_[3];
      if (uplink_frame_[3] > (kMaxUplinkFrameSize - 5U)) {
        ResetUplinkAssembler();
        stats_.bad_payload_frames += 1U;
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
      SetStatus("uart checksum error", true);
      return;
    }

    stats_.uart_frames += 1U;
    UpdateTelemetryFromFrame(uplink_frame_, uplink_expected_size_);
  }

  void UpdateTelemetryFromFrame(const uint8_t* frame, size_t frame_size) {
    if ((frame == nullptr) || (frame_size < 5U)) {
      return;
    }

    const uint8_t frame_type = frame[2];
    const uint8_t payload_size = frame[3];
    const uint8_t* payload = frame + 4U;
    const uint32_t now_ms = millis();

    stats_.last_frame_type = frame_type;
    stats_.last_payload_size = payload_size;

    if (frame_type == kFrameTypeGuidanceTelemetry) {
      if (payload_size != 28U) {
        stats_.bad_payload_frames += 1U;
        return;
      }
      ApplyGuidanceTelemetry(payload, payload_size, now_ms);
    } else if (frame_type == kFrameTypeImuAccel) {
      if (payload_size != 12U) {
        stats_.bad_payload_frames += 1U;
        return;
      }
      ApplyImuAccel(payload, payload_size, now_ms);
    } else if (frame_type == kFrameTypeImuMotion) {
      if ((payload_size != kImuMotionPayloadLegacySize) &&
          (payload_size != kImuMotionPayloadExtendedSize)) {
        stats_.bad_payload_frames += 1U;
        return;
      }
      ApplyImuMotion(payload, payload_size, now_ms);
    } else if (frame_type == kFrameTypeDartLaunchSample) {
      if (payload_size != kDartLaunchSamplePayloadSize) {
        stats_.bad_payload_frames += 1U;
        return;
      }
      ApplyDartLaunchSample(payload, payload_size, now_ms);
    } else {
      stats_.unknown_uart_frames += 1U;
    }
  }

  void ApplyGuidanceTelemetry(const uint8_t* payload, uint8_t payload_size, uint32_t now_ms) {
    if ((payload == nullptr) || (payload_size < 14U)) {
      return;
    }

    const uint8_t flags = payload[2];
    const uint16_t measurement_x = ReadU16Le(payload + 4U);
    const uint16_t measurement_y = ReadU16Le(payload + 6U);
    const int16_t delta_x = ReadI16Le(payload + 10U);
    const int16_t delta_y = ReadI16Le(payload + 12U);
    const bool target_valid = ((flags & kGuidanceFlagTargetDetected) != 0U) &&
                              (measurement_x != kNoTargetCoordinate) &&
                              (measurement_y != kNoTargetCoordinate);

    telemetry_.guidance_seen = true;
    telemetry_.last_guidance_ms = now_ms;
    telemetry_.target_valid = target_valid;
    telemetry_.target_x = target_valid ? static_cast<float>(delta_x) : -1.0f;
    telemetry_.target_y = target_valid ? static_cast<float>(delta_y) : -1.0f;
    stats_.guidance_frames += 1U;
  }

  void ApplyImuAccel(const uint8_t* payload, uint8_t payload_size, uint32_t now_ms) {
    if ((payload == nullptr) || (payload_size != 12U)) {
      return;
    }

    telemetry_.accel_x = ReadFloatLe(payload + 0U);
    telemetry_.accel_y = ReadFloatLe(payload + 4U);
    telemetry_.accel_z = ReadFloatLe(payload + 8U);
    telemetry_.accel_seen = true;
    telemetry_.last_accel_ms = now_ms;
    stats_.accel_frames += 1U;
  }

  void ApplyImuMotion(const uint8_t* payload, uint8_t payload_size, uint32_t now_ms) {
    if ((payload == nullptr) ||
        ((payload_size != kImuMotionPayloadLegacySize) &&
         (payload_size != kImuMotionPayloadExtendedSize))) {
      return;
    }

    telemetry_.velocity_x = ReadFloatLe(payload + 0U);
    telemetry_.velocity_y = ReadFloatLe(payload + 4U);
    telemetry_.velocity_z = ReadFloatLe(payload + 8U);
    telemetry_.accel_x = ReadFloatLe(payload + 12U);
    telemetry_.accel_y = ReadFloatLe(payload + 16U);
    telemetry_.accel_z = ReadFloatLe(payload + 20U);

    if (payload_size == kImuMotionPayloadExtendedSize) {
      const float counter_ticks = ReadFloatLe(payload + 24U);
      telemetry_.dart_launch_counter_ticks =
          (counter_ticks <= 0.0f) ? 0U : static_cast<uint16_t>(counter_ticks + 0.5f);
      telemetry_.dart_launch_velocity_x = ReadFloatLe(payload + 28U);
      telemetry_.dart_launch_velocity_y = ReadFloatLe(payload + 32U);
      telemetry_.dart_launch_velocity_z = ReadFloatLe(payload + 36U);
    }

    telemetry_.motion_seen = true;
    telemetry_.accel_seen = true;
    telemetry_.last_motion_ms = now_ms;
    telemetry_.last_accel_ms = now_ms;
    stats_.motion_frames += 1U;
  }

  void ApplyDartLaunchSample(const uint8_t* payload, uint8_t payload_size, uint32_t now_ms) {
    if ((payload == nullptr) || (payload_size != kDartLaunchSamplePayloadSize)) {
      return;
    }

    telemetry_.dart_launch_counter_ticks = ReadU16Le(payload + 0U);
    telemetry_.dart_launch_velocity_x = ReadFloatLe(payload + 2U);
    telemetry_.dart_launch_velocity_y = ReadFloatLe(payload + 6U);
    telemetry_.dart_launch_velocity_z = ReadFloatLe(payload + 10U);
    telemetry_.last_motion_ms = now_ms;
    stats_.launch_sample_frames += 1U;
  }

  void PublishTelemetrySnapshot() {
    const uint32_t now_ms = millis();
    if ((telemetry_characteristic_ == nullptr) || !ble_connected_) {
      return;
    }
    if (now_ms - last_telemetry_notify_ms_ < kTelemetryNotifyIntervalMs) {
      return;
    }
    last_telemetry_notify_ms_ = now_ms;

    TelemetryPacket packet = {};
    packet.magic = kTelemetryMagic;
    packet.version = kTelemetryVersion;
    packet.type = kTelemetryTypeSnapshot;
    packet.sequence = telemetry_sequence_++;
    packet.flags = BuildTelemetryFlags(now_ms);
    packet.esp_time_ms = now_ms;
    packet.source_time_ms = LatestSourceTimeMs();

    if ((packet.flags & kTelemetryFlagTargetValid) != 0U) {
      packet.target_x = telemetry_.target_x;
      packet.target_y = telemetry_.target_y;
    } else {
      packet.target_x = -1.0f;
      packet.target_y = -1.0f;
    }

    packet.velocity_x = telemetry_.velocity_x;
    packet.velocity_y = telemetry_.velocity_y;
    packet.velocity_z = telemetry_.velocity_z;
    packet.accel_x = telemetry_.accel_x;
    packet.accel_y = telemetry_.accel_y;
    packet.accel_z = telemetry_.accel_z;
    packet.dart_launch_counter_ticks = telemetry_.dart_launch_counter_ticks;
    packet.dart_launch_velocity_x = telemetry_.dart_launch_velocity_x;
    packet.dart_launch_velocity_y = telemetry_.dart_launch_velocity_y;
    packet.dart_launch_velocity_z = telemetry_.dart_launch_velocity_z;
    packet.crc16 = Crc16Ccitt(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet) - sizeof(packet.crc16));

    NotifyFrame(telemetry_characteristic_, reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
    stats_.telemetry_notifications += 1U;
  }

  uint16_t BuildTelemetryFlags(uint32_t now_ms) const {
    uint16_t flags = 0U;
    const bool guidance_current = telemetry_.guidance_seen &&
                                  ((now_ms - telemetry_.last_guidance_ms) <= kSourceStaleMs);
    const bool motion_current = telemetry_.motion_seen &&
                                ((now_ms - telemetry_.last_motion_ms) <= kSourceStaleMs);
    const bool accel_current = telemetry_.accel_seen &&
                               ((now_ms - telemetry_.last_accel_ms) <= kSourceStaleMs);

    if (telemetry_.guidance_seen) {
      flags |= kTelemetryFlagGuidanceSeen;
      if (!guidance_current) {
        flags |= kTelemetryFlagGuidanceStale;
      }
    }
    if (telemetry_.motion_seen) {
      flags |= kTelemetryFlagMotionSeen;
      if (!motion_current) {
        flags |= kTelemetryFlagMotionStale;
      }
    }
    if (telemetry_.accel_seen) {
      flags |= kTelemetryFlagAccelSeen;
      if (!accel_current) {
        flags |= kTelemetryFlagMotionStale;
      }
    }
    if (guidance_current && telemetry_.target_valid) {
      flags |= kTelemetryFlagTargetValid;
    } else if (telemetry_.guidance_seen) {
      flags |= kTelemetryFlagTargetLost;
    }

    return flags;
  }

  uint32_t LatestSourceTimeMs() const {
    uint32_t latest_ms = telemetry_.last_guidance_ms;
    if (telemetry_.last_motion_ms > latest_ms) {
      latest_ms = telemetry_.last_motion_ms;
    }
    if (telemetry_.last_accel_ms > latest_ms) {
      latest_ms = telemetry_.last_accel_ms;
    }
    return latest_ms;
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

  static uint16_t ReadU16Le(const uint8_t* data) {
    return static_cast<uint16_t>(data[0]) |
           static_cast<uint16_t>(static_cast<uint16_t>(data[1]) << 8U);
  }

  static int16_t ReadI16Le(const uint8_t* data) {
    return static_cast<int16_t>(ReadU16Le(data));
  }

  static float ReadFloatLe(const uint8_t* data) {
    float value = 0.0f;
    std::memcpy(&value, data, sizeof(value));
    return value;
  }

  static uint16_t Crc16Ccitt(const uint8_t* data, size_t data_size) {
    uint16_t crc = 0xFFFFU;
    for (size_t index = 0U; index < data_size; ++index) {
      crc ^= static_cast<uint16_t>(data[index]) << 8U;
      for (uint8_t bit = 0U; bit < 8U; ++bit) {
        if ((crc & 0x8000U) != 0U) {
          crc = static_cast<uint16_t>((crc << 1U) ^ 0x1021U);
        } else {
          crc = static_cast<uint16_t>(crc << 1U);
        }
      }
    }
    return crc;
  }

  void ResetUplinkAssembler() {
    uplink_frame_index_ = 0U;
    uplink_expected_size_ = 0U;
  }

  void MaintainAdvertising() {
    if (ble_connected_) {
      return;
    }
    if (millis() - disconnect_timestamp_ms_ >= kDisconnectRestartDelayMs) {
      StartAdvertising();
    }
  }

  void StartAdvertising() {
    NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();
    if ((advertising != nullptr) && !advertising->isAdvertising()) {
      advertising->start();
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
    const uint32_t now_ms = millis();
    String status;
    status.reserve(220);
    status += "ble=";
    status += ble_connected_ ? "1" : "0";
    status += ",rx=";
    status += String(BRIDGE_RX_PIN);
    status += ",tx=";
    status += String(BRIDGE_TX_PIN);
    status += ",baud=";
    status += String(BRIDGE_UART_BAUD);
    status += ",rx_b=";
    status += String(stats_.uart_rx_bytes);
    status += ",h0=";
    status += String(stats_.uart_header0_bytes);
    status += ",hdr=";
    status += String(stats_.uart_header_pairs);
    status += ",last_t=0x";
    if (stats_.last_frame_type < 16U) {
      status += "0";
    }
    status += String(static_cast<unsigned int>(stats_.last_frame_type), HEX);
    status += ",last_len=";
    status += String(stats_.last_payload_size);
    status += ",uart=";
    status += String(stats_.uart_frames);
    status += ",guid=";
    status += String(stats_.guidance_frames);
    status += ",motion=";
    status += String(stats_.motion_frames);
    status += ",accel=";
    status += String(stats_.accel_frames);
    status += ",launch=";
    status += String(stats_.launch_sample_frames);
    status += ",snap=";
    status += String(stats_.telemetry_notifications);
    status += ",crc=";
    status += String(stats_.checksum_errors);
    status += ",bad_len=";
    status += String(stats_.bad_payload_frames);
    status += ",bad_type=";
    status += String(stats_.unknown_uart_frames);
    status += ",drop_uart=";
    status += String(stats_.dropped_uart_frames);
    status += ",age_g=";
    status += String(telemetry_.guidance_seen ? now_ms - telemetry_.last_guidance_ms : 0U);
    status += ",age_m=";
    status += String(telemetry_.motion_seen ? now_ms - telemetry_.last_motion_ms : 0U);
    status += ",age_a=";
    status += String(telemetry_.accel_seen ? now_ms - telemetry_.last_accel_ms : 0U);
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
constexpr char Esp32BridgeApp::kTelemetryCharacteristicUuid[];
constexpr char Esp32BridgeApp::kStatusCharacteristicUuid[];

Esp32BridgeApp g_esp32_bridge_app;

void setup() {
  g_esp32_bridge_app.Setup();
}

void loop() {
  g_esp32_bridge_app.Loop();
}
