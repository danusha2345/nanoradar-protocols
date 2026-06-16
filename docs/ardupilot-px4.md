# ArduPilot / PX4 integration

NanoRadar is an official partner of both ArduPilot and PX4, but stock support is limited to two models.

## ArduPilot — supported today

Verified across all release branches (checked June 2026). Both drivers were added in the **4.5** cycle (absent in 4.4 and earlier):

| Model | Subsystem | Type | Bus | Added |
|---|---|---|---|---|
| **MR72** | Proximity | `PRX_TYPE = 17` (MR72_CAN) | CAN | 4.5 ([PR #25801](https://github.com/ArduPilot/ardupilot/pull/25801)) |
| **NRA24** | RangeFinder | `RNGFNDx_TYPE` NRA24_CAN (39) | CAN | 4.5 ([PR #24097](https://github.com/ArduPilot/ardupilot/pull/24097)) |

No stock support for MR76, **MR82**, NRA15, NRA24Pro, UAM231, **UAM285** (0 references in the codebase). MR82 is likely picked up by the MR72 proximity type since it shares family-A protocol (unconfirmed on hardware).

MR72 and NRA24 can share one CAN bus. Note `MR72_MAX_RANGE_M = 50` in the proximity driver clamps range to 50 m even though MR82 reaches 80 m.

### NRA24 parameters (from the ArduPilot wiki)

**CAN** (native NanoRadar protocol):
```
CAN_P2_DRIVER   = 2
CAN_P2_BITRATE  = 500000        # or whatever you set in the NSM tool
CAN_D2_PROTOCOL = 14            # RadarCAN
RNGFND1_TYPE    = 39            # NRA-24 (reboot after setting)
RNGFND1_RECV_ID = <sensor id>   # 0 = accept all CAN ids
RNGFND1_MAX     = 190 ; RNGFND1_MIN = 0.5
```

**UART** — NanoRadar's recommended path is the **"open-source" / USD1 mode** (the radar emits a US-D1-compatible stream; no ArduPilot code change):
```
SERIAL1_PROTOCOL = 9            # Rangefinder
SERIAL1_BAUD     = 115          # 115200
RNGFND1_TYPE     = 11           # USD1_Serial (reboot after setting)
RNGFND1_MAX      = 190 ; RNGFND1_MIN = 0.5
```
⚠️ ArduPilot warns it hasn't verified all edge cases of the USD1 driver with NanoRadar hardware. PR #30973 adds a *native* NRA24 serial driver as an alternative (because USD1 doesn't fully match the NRA24 docs / some firmwares).

## ArduPilot — in progress

- **[PR #30973](https://github.com/ArduPilot/ardupilot/pull/30973)** (open) — adds a NanoRadar **NRA24 serial/UART** rangefinder driver (hardware-verified by the author). Decodes distance as 16-bit (`(DIST_H<<8)|DIST_L`) capped at 50 m, so it does not cover the long-range altimeters.
- **[PR #23860](https://github.com/ArduPilot/ardupilot/pull/23860)** (open) — Continental **ARS408** CAN proximity, the same family-A protocol as MR72/MR82.

## The long-range gap

Both NanoRadar drivers (the merged `NRA24_CAN` and the open serial PR) decode altimeter distance as a **16-bit** field, which saturates at ~655 m. NanoRadar's long-range altimeters (NRA24Pro / UAM231 / UAM285, up to 3000 m) reuse the same `0x70C` frame but extend distance to **20 bits** using the high nibble of the target-id byte (see [altimeter-protocol.md](altimeter-protocol.md)). Supporting those models needs:
```
dist = ( ((data[0]>>4) & 0x0F) << 16 | data[2]<<8 | data[3] ) * 0.01
```
and a higher (or removed) range clamp. This was flagged upstream on PR #30973.

A **no-recompile** option for the full range is a Lua scripting driver (reads the CAN
frames, does the 20-bit decode, feeds a scripting rangefinder backend `RNGFNDx_TYPE=36`).
A ready-to-use one is in [`../ardupilot/nanoradar_altimeter_can_rangefinder.lua`](../ardupilot/nanoradar_altimeter_can_rangefinder.lua).

## PX4

PX4 has a `nanoradar_can` driver under `src/drivers/distance_sensor/` (UAVCAN-based), discussed on the [PX4 forum](https://discuss.px4.io/t/px4-support-millimeter-wave-radar-as-rangfinder-and-obstacle-distance-sensor/46876). Configure via the standard `EKF2_RNG_*` rangefinder parameters once the driver is built in.
