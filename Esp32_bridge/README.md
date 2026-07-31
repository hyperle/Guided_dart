# ESP32 Bridge

`Esp32_bridge/` is the BLE telemetry bridge for the dart guidance stack. The runtime path is intentionally one-way:

- STM32 USART2 -> ESP32 bridge UART
- ESP32 -> host over BLE notify

USART1 remains reserved for the OpenMV/camera measurement input on the STM32 side. Do not mix debug text into USART2; the STM32 firmware disables printf on that UART by default.

## Files

- `platformio.ini`: ESP32-C3 PlatformIO configuration
- `src/main.cpp`: UART frame parser, telemetry aggregator, and BLE GATT server

## Wiring

Default ESP32 pins come from `platformio.ini`:

- `BRIDGE_RX_PIN=20`: ESP32 RX, connect to STM32 USART2 TX (`PA2`)
- `BRIDGE_TX_PIN=21`: ESP32 TX, connect to STM32 USART2 RX (`PA3`) if a return path is later needed; current telemetry path does not use it
- `BRIDGE_UART_BAUD=115200`

Also connect:

- `ESP32 GND` <-> `STM32 GND`

If your board uses different ESP32 pins, change only `platformio.ini`.

## BLE Interface

Service UUID:

- `4fafc201-1fb5-459e-8fcc-c5c9c331914b`

Characteristics:

- telemetry snapshot notify/read characteristic:
  `7b3f5a10-2d36-4c8f-9a7f-3a2d542fd1b2`
  The host visualizer subscribes here. One notification is one fixed 50-byte snapshot.

- status read/notify characteristic:
  `3d7f7b30-5602-4f2d-9d45-1f1be1b4c001`
  Human-readable bridge state and counters for debugging.

## STM32 UART2 Uplink

Each UART2 frame is:

```text
0xA5 0x5A | type | len | payload[len] | checksum
```

Checksum rule:

- `checksum = sum(all previous frame bytes) & 0xFF`

Supported frame types consumed by the bridge:

- `0x01`: guidance telemetry, including target delta; target lost is exported as `<-1, -1>`
- `0x05`: IMU motion, six `float32` values: `gx, gy, gz` in `deg/s`, then `ax, ay, az` in `g`
- `0x06`: dart launch sample, `uint16 counter_ticks` plus three `float32` tick10 speeds in `deg/s`

The bridge aggregates these UART frames into a stable BLE snapshot stream instead of forwarding every UART frame immediately.

## BLE Snapshot Packet

Snapshot characteristic payload (`little-endian`, 64 bytes):

```text
uint16 magic          0xDA7A
uint8  version        2
uint8  type           1
uint16 sequence
uint16 flags
uint32 esp_time_ms
uint32 source_time_ms
float  target_x
float  target_y
float  velocity_x
float  velocity_y
float  velocity_z
float  accel_x
float  accel_y
float  accel_z
uint16 dart_launch_counter_ticks
float  dart_launch_velocity_x
float  dart_launch_velocity_y
float  dart_launch_velocity_z
uint16 crc16_ccitt
```

Flag bits:

- bit 0: target valid
- bit 1: guidance frame has been seen
- bit 2: motion frame has been seen
- bit 3: acceleration data has been seen
- bit 4: target lost
- bit 5: guidance data is stale
- bit 6: motion data is stale

The default notify rate is `25 Hz` (`BRIDGE_TELEMETRY_NOTIFY_HZ=25`). Connection parameters are requested at about 15-30 ms interval with a 2 s supervision timeout.

## Host Visualization

Run the host client from the repo root:

```bash
./.script/ble-connect
```

Useful options:

```bash
./.script/ble-connect --scan
./.script/ble-connect --address AA:BB:CC:DD:EE:FF
./.script/ble-connect --plot
./.script/ble-connect --no-log
```

The default terminal view shows connection state, packet rate, drop count, target `<x, y>`, three-axis velocity, and three-axis acceleration. `--plot` adds a matplotlib live plot if matplotlib is installed.

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
