# Sources

Everything here was reconstructed from public, freely available material. No vendor datasheets, firmware, or tool binaries are redistributed in this repo.

## Open-source drivers (object / family A)
- [olezhkameleshko-ux/mr72-radar-library](https://github.com/olezhkameleshko-ux/mr72-radar-library) — C++ MR72 CAN library (SocketCAN + Waveshare USB-CAN-A), firmware V4.0.0. Primary source for the `0x60B` decode and config bit layout.
- [ArduPilot `AP_Proximity_MR72_CAN`](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Proximity/AP_Proximity_MR72_CAN.cpp)
- [harrylal/radar_mr76](https://github.com/harrylal/radar_mr76) — ROS package for MR76 (CAN).
- [Ashraff93/NanoRadar_MR72](https://github.com/Ashraff93/NanoRadar_MR72), [Ashraff93/NanoRadar_NRA15](https://github.com/Ashraff93/NanoRadar_NRA15)

## Open-source drivers (altimeter / family B)
- [ArduPilot `AP_RangeFinder_NRA24_CAN`](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_RangeFinder/AP_RangeFinder_NRA24_CAN.cpp) — `0x70C` target / `0x60A` heartbeat, 16-bit distance.
- [ArduPilot PR #30973](https://github.com/ArduPilot/ardupilot/pull/30973) — NRA24 serial driver (documents the 14-byte serial frame).
- PX4 `nanoradar_can` driver (`src/drivers/distance_sensor/`).

## NanoRadar official (for reference — go here for the authoritative docs/tools)
- Product pages: <https://www.nanoradar.com/> · <https://en.nanoradar.cn/>
- Tool downloads (altimeter / obstacle-avoidance): <https://www.nanoradar.com/Tool_Download/8.html>
  - `NSM77 Tools v1.3.7` — host software for MR82 and the MR/SR object series.
  - `NSM Tools v2.0.7` — host software for the altimeter series (NRA24/NRA15/UAM231). Used (decompiled) to recover the 20-bit long-range distance field.
- MR72 communication protocol (official): <http://en.nanoradar.cn/File/view/id/491.html>
- NRA24 user manual ("with check sum") — serial frame + checksum (search ManualsLib / fcc.report FCC-ID `2A6WU-NRA24`).
- The **UAM285 protocol manual** does not appear to be publicly posted; request it from `sales@nanoradar.cn`.

## ArduPilot docs
- [Nanoradar MR72 (Copter)](https://ardupilot.org/copter/docs/common-rangefinder-mr72.html)
- [Nanoradar NRA24 (Copter)](https://ardupilot.org/copter/docs/common-rangefinder-nra24.html)

## How the reverse-engineering was done
- Object protocol: read directly from the C++/ArduPilot driver source (constants + bit math), cross-checked between independent implementations.
- Altimeter long-range distance: the official `NSM Tools v2.0.7` (.NET / Mono assembly) was disassembled with `monodis`; the `RadarPktParse` / `RadarPktParseCan` methods showed `distance = (((B0>>4)&0x0F)<<16 | B2<<8 | B3) * 0.01` for the UAM path.

Found something wrong, or have a hardware capture or the official UAM285 manual? Please open an issue or PR.
