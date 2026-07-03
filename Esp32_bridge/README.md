# ESP32 Bridge

`Esp32_bridge/` is a clean PlatformIO project distilled from the useful BLE/PlatformIO parts of `/home/hyperlee/esp_transmit`.

This project is no longer IMU-specific. Its role is to bridge:

- host <-> ESP32 over BLE
- ESP32 <-> STM32 guidance board over UART

The STM32 side keeps using the existing binary frame format:

- frame header: `0xA5 0x5A`
- type: `0x01` telemetry, `0x02` parameter command, `0x03` parameter ack
- frame body: `type + len + payload + checksum`

## Files

- `platformio.ini`: ESP32-C3 PlatformIO configuration
- `src/main.cpp`: BLE <-> UART bridge implementation

## Wiring

Default pins come from `platformio.ini`:

- `BRIDGE_RX_PIN=20`: ESP32 RX, connect to STM32 TX
- `BRIDGE_TX_PIN=21`: ESP32 TX, connect to STM32 RX
- `BRIDGE_UART_BAUD=115200`

Also connect:

- `ESP32 GND` <-> `STM32 GND`

If your board uses different pins, change only `platformio.ini`.

## BLE Interface

Service UUID:

- `4fafc201-1fb5-459e-8fcc-c5c9c331914b`

Characteristics:

- downlink write characteristic:
  `beb5483e-36e1-4688-b7f5-ea07361b26a8`
  Host writes complete STM32 parameter frames here. ESP32 forwards them to UART unchanged.

- uplink notify characteristic:
  `9f6c1db5-0b3b-4d1d-8a4d-11dd5c3a4f21`
  ESP32 continuously notifies complete STM32 uplink frames here. This keeps telemetry on a dedicated always-on stream.

- ack notify characteristic:
  `de24d570-5f81-4d6b-8e4d-2f1309367d91`
  ESP32 duplicates `0x03` parameter ack frames here so host-side `param-push` can wait for deterministic confirmations without competing with telemetry.

- status read/notify characteristic:
  `3d7f7b30-5602-4f2d-9d45-1f1be1b4c001`
  Human-readable bridge state and counters for debugging.

## BLE Protocol

### Roles

- Host is the BLE client
- `Esp32_bridge` is the BLE server and UART bridge
- STM32 only sees the existing UART binary frames and does not know BLE exists

### Characteristic Semantics

- Downlink write characteristic:
  Host writes one complete STM32 parameter frame per BLE write.
  ESP32 forwards the frame to UART immediately from the BLE write callback. There is no periodic downlink polling loop.

- Uplink notify characteristic:
  ESP32 assembles one complete STM32 UART frame and notifies it to the host unchanged.

- Ack notify characteristic:
  ESP32 mirrors only parameter ack frames (`0x03`) to a dedicated notify channel.
  Telemetry remains on the general uplink channel.

- Status characteristic:
  Debug-only human-readable bridge state.
  It is not part of the parameter/telemetry protocol.

### Binary Frame Format

Each transported frame is:

```text
0xA5 0x5A | type | len | payload[len] | checksum
```

Checksum rule:

- `checksum = sum(all previous frame bytes) & 0xFF`

Supported frame types:

- `0x01`: guidance telemetry
- `0x02`: parameter command
- `0x03`: parameter ack

### Payload Layout

Parameter command payload:

```text
<H B I
sequence : uint16 little-endian
key      : uint8
value    : uint32 little-endian
```

Parameter ack payload:

```text
<H B B I
sequence : uint16 little-endian
key      : uint8
status   : uint8
value    : uint32 little-endian
```

`float` parameters are encoded by packing IEEE754 `float32` bits into the `uint32` value field.

### Transport Constraints

- One BLE write carries exactly one complete STM32 frame
- One BLE notify carries exactly one complete STM32 frame
- No fragmentation is implemented in the current bridge
- Host should use a strict request/ack flow: send one parameter frame, then wait for the matching `0x03` ack frame with the same sequence and key
- Current guidance telemetry and parameter frames are all within the bridge limits and do not require BLE fragmentation

### Host Tool Mapping

The host sync tool in [guidance_param_sync.py](/home/hyperlee/guided_dart/Host_tools/guidance/guidance_param_sync.py:1) supports:

- `--transport serial`
- `--transport ble`

In BLE mode it writes binary parameter frames to:

- `beb5483e-36e1-4688-b7f5-ea07361b26a8`

and waits for `0x03` ack frames from:

- `de24d570-5f81-4d6b-8e4d-2f1309367d91`

Viewer-style tools should keep consuming telemetry from:

- `9f6c1db5-0b3b-4d1d-8a4d-11dd5c3a4f21`

## Build

```bash
cd Esp32_bridge
pio run
```

Upload:

```bash
cd Esp32_bridge
pio run -t upload
```

Serial monitor:

```bash
cd Esp32_bridge
pio device monitor -b 115200
```

## Migration Notes

The following external-directory artifacts were intentionally not migrated:

- `.pio/`
- `.venv/`
- `logs/`
- `__pycache__/`
- `.vscode/`
- `esp32_ble_server.ino`
- IMU-only host-side scripts and notes

The old project was focused on IMU BLE forwarding. This clean bridge project keeps the reusable BLE server and PlatformIO structure, but retargets the application to the current guidance parameter/telemetry tunnel.
