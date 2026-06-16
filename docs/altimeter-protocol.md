# Altimeter protocol (family B)

Applies to NanoRadar radar **altimeters**: NRA12, NRA15, **NRA24**, NRA24Pro, UAM231, **UAM285**. Single target (ground-relative height), output over CAN and/or UART (TTL/RS232/RS422/RS485 depending on the hardware variant).

## Models & ranges (matters for the distance field width)

| Model | Band | Max range | Distance field |
|---|---|---|---|
| NRA15 | 24 GHz | short | 16-bit |
| NRA24 | 24 GHz (K-band) | 50 m / 200 m | 16-bit (`data[2..3]`) |
| NRA24Pro | 24 GHz | ~300–500 m | 16-bit (≤655 m fits) |
| UAM231 | 24 GHz | **1000 m** | **20-bit** (needs high nibble) |
| UAM285 | mmWave | **3000 m** (max ~5400) | **20-bit** |

The 16-bit `(data[2]*256+data[3])*0.01` decode saturates at **655.35 m**. So NRA24/NRA24Pro fit in 16 bits, but **UAM231 (1000 m) and UAM285 (3000 m) require the 20-bit form below**. UAM231's 1000 m range is independent corroboration of the 20-bit field recovered from the vendor tool.

Sources:
- ArduPilot [`AP_RangeFinder_NRA24_CAN`](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_RangeFinder/AP_RangeFinder_NRA24_CAN.cpp) (CAN decode)
- NanoRadar NRA24 user manual ("with check sum") — serial frame + checksum
- Decompilation of NanoRadar's own configurator **NSM Tools v2.0.7** (supports NRA24/NRA15/UAM231) — the long-range distance field

> ⚠️ The 20-bit long-range distance decode below is reverse-engineered from the vendor tool and is **not yet hardware-verified**. The short-range form is confirmed by the ArduPilot driver and the NRA24 manual.

## CAN

| ID | Name | Purpose |
|---|---|---|
| `0x60A` | Radar status / heartbeat | liveness; not parsed for data |
| `0x70C` | Target Info | distance + SNR |

Radar-id addressing is the same `id + sensorID*0x10` scheme; ArduPilot extracts the id as `(id & 0xF0) >> 4` and matches the message by the low nibble (`0xA` = status, `0xC` = target). Default bitrate 500 kbit/s (250k / 1M selectable in the tool).

### `0x70C` Target Info decode

Short-range altimeters (NRA24 50 m / 200 m — value never exceeds 655 m):
```
distance = (data[2]*256 + data[3]) * 0.01            # m
snr      = data[7] - 128                              # ArduPilot NRA24 CAN driver
```

Long-range altimeters (**UAM231 ≥1000 m, UAM285 ≥3000 m**) — the upper distance bits are stored in the **high nibble of the target-id byte**:
```
id       = data[0] & 0x0F
distance = ( ((data[0]>>4) & 0x0F) << 16 | data[2]<<8 | data[3] ) * 0.01   # 20-bit → up to ~10485 m
```
In the vendor tool's UAM parser, SNR is taken from `data[1] * 0.5` rather than `data[7]-128`, i.e. the long-range frame layout differs from NRA24 by more than just the high nibble. Treat the exact SNR/extra-field placement as unconfirmed until checked against the official UAM285 protocol manual or a capture.

## UART / serial

The serial form wraps the same message IDs in a framed, checksummed 14-byte packet (NRA24 manual):
```
0xAA 0xAA | ID_lo ID_hi | b0 b1 b2 b3 b4 b5 b6 | checksum | 0x55 0x55
ID       = ID_lo + ID_hi*0x100        # 0x0A 0x06 → 0x60A status ; 0x0C 0x07 → 0x70C target
checksum = 0xFF & (b0 + b1 + b2 + b3 + b4 + b5 + b6)     # low byte of payload sum
```

### `0x70C` target payload (serial)
Worked example from the manual — payload `01 C8 07 D0 00 02 EE`, checksum `90`:

| Byte | Field | Formula | Example |
|---|---|---|---|
| b0 | Index / target id | `b0 & 0x0F` (high nibble = distance bits 16-19 on long-range) | 1 |
| b1 | RCS | `b1 * 0.5 - 50` | 0xC8 → 50 |
| b2,b3 | Range | `(b2*256 + b3) * 0.01` m | 0x07D0 → 20.00 |
| b4 | reserved | — | 0 |
| b5 | roll count | `(b5 & 0xE0) >> 5` | |
| b6 | (speed / snr — model-dependent) | | |
| +1 | checksum | low byte of payload sum | 0x90 |

A complete record is typically `0x60A` status followed by `0x70C` target(s).

## "Open-source" output mode = USD1 / uLanding compatible

The altimeters expose two protocol modes (selectable in the NSM tool / per the UAM285 manual):
- **general** — the native NanoRadar frames above (`0x60A` / `0x70C`).
- **open-source** — emits an **Aerotenna/Ainstein US-D1 (uLanding) compatible serial stream**, so it works with ArduPilot/PX4 **without any code change** — you just select the existing USD1 rangefinder driver. NanoRadar explicitly documents this path (and recommends it for UART use).

USD1 serial frame (from ArduPilot's [`AP_RangeFinder_USD1_Serial`](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_RangeFinder/AP_RangeFinder_USD1_Serial.cpp), 115200 baud):
```
V1:  0xFE | version | distL | distH | b4 | checksum
     distance = (distH*256 + distL) * 0.01   # m
     checksum = (distL + distH + b4) ... low byte (driver: (buf[1]+buf[2]+buf[3]+buf[4]) & 0xFF == buf[5])
V0 (beta, header 0x48):  0x48 | b1 | b2     # distance = ((b2&0x7F)*128 + (b1&0x7F)) * 0.01
```
So in open-source mode the radar pretends to be a US-D1; in ArduPilot set `RNGFNDx_TYPE = 11 (USD1_Serial)`, `SERIALx_PROTOCOL = 9`, baud 115200. See [ardupilot-px4.md](ardupilot-px4.md).

## Behaviour notes (from UAM285 manual)

- Output is single-target ("object" mode).
- When no target / no motion, the radar outputs the configured **blind-spot** value; out of range it holds the last value with **confidence = 0**.
- If the platform attitude angle exceeds ~20°, no distance is output.
- The interface (CAN vs TTL vs RS232/422/485) is fixed in hardware per SKU — not switchable by configuration.

## Reading on Linux

```sh
# CAN
sudo ip link set can0 up type can bitrate 500000
candump -L can0 | tee altimeter.log
candump can0,70C:7FF                 # target frames only (sensor 0)

# serial
tio /dev/ttyUSB0 -b 115200 | tee altimeter_uart.log
```
