# Object / Cluster CAN protocol (family A)

Applies to NanoRadar **MR72, MR76, MR82** (and the SR/UCM object-output series). Derived from the Continental **ARS408** protocol: the radar outputs two parallel lists — *Objects* (tracked targets) and *Clusters* (raw reflections).

Cross-verified from several independent implementations, including **NanoRadar's own PX4 driver**:
- [NanoRadar official PX4 driver](https://github.com/OuYangLei92/PX4-Autopilot/tree/NanoradarCAN-v1.15.4/src/drivers/distance_sensor/nanoradar_can) (PX4 PR #25006) — same `0x60B` math: `x = (((data[2]&0x07)<<8)|data[3])*0.2-204.6`, `y = ((data[1]<<5)|(data[2]>>3))*0.2-500.0`
- [`olezhkameleshko-ux/mr72-radar-library`](https://github.com/olezhkameleshko-ux/mr72-radar-library) (C++, firmware V4.0.0)
- ArduPilot [`AP_Proximity_MR72_CAN`](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Proximity/AP_Proximity_MR72_CAN.cpp)
- [`harrylal/radar_mr76`](https://github.com/harrylal/radar_mr76) (ROS)

## Bus

- Default bitrate **500 kbit/s**.
- Standard 11-bit IDs, DLC = 8.
- Bit numbering below is little-endian within the 8 data bytes (LSB0), matching the reference library's `extractBits`/`insertBits`.

## Addressing by radar id

```
message_id = base_id + sensorID * 0x10        # sensorID = 0..7
```
e.g. radar id 1: `0x200→0x210`, `0x60A→0x61A`, `0x60B→0x61B`, `0x700→0x710`.

## Message map (base ids, sensorID = 0)

| ID | Name | Dir | Purpose |
|---|---|---|---|
| `0x200` | RadarConfig | host→radar | set max distance, sensor id, output type, power, sort, RCS threshold, NVM |
| `0x201` | RadarState | radar→host | heartbeat: current config + nvmReadOK/nvmWriteOK |
| `0x401` | RegionConfig | host→radar | collision/detection region rectangle |
| `0x402` | RegionState | radar→host | region readback |
| `0x60A` | ObjectListStatus | radar→host | start of a cycle: number of objects |
| `0x60B` | ObjectGeneral | radar→host | one frame per object |
| `0x700` | FirmwareVersion | radar→host | major/minor/patch |

Flow: `0x60A` (`nofObjects = N`) followed by `N` × `0x60B`.

## `0x60A` ObjectListStatus

```
nofObjects       = data[0]                       # objects in this cycle
measCount        = bits 8..23  (16-bit)          # rolling measurement counter
interfaceVersion = bits 28..31 (4-bit)
```

## `0x60B` ObjectGeneral

```
id       = data[0]
distLong = (data[1]*32 + (data[2]>>3))        * 0.2 - 500.0    # m, longitudinal
distLat  = ((data[2]&0x07)*256 + data[3])     * 0.2 - 204.6    # m, lateral
vrelLong = (data[4]*4 + (data[5]>>6))         * 0.25 - 128.0   # m/s
vrelLat  = ((data[5]&0x3F)*8 + (data[6]>>5))  * 0.25 - 64.0    # m/s
dynProp  = data[6] & 0x07
sector   = (data[6] >> 3) & 0x03
rcs      = data[7] * 0.5 - 64.0                                 # dBsm
```
Derived range/azimuth: `range = hypot(distLong, distLat)`, `azimuth = atan2(distLat, distLong)`
(ArduPilot uses `atan2(x, y)` with `x = distLat`, `y = distLong`; check signs against your mounting).

## `0x200` RadarConfig (host → radar)

DLC = 8. Byte 0 carries one *valid* flag per parameter (only flagged fields are applied):
`maxDistance, sensorID, radarPower, outputType, sendQuality, sendExtInfo, sortIndex, storeInNVM` (+ `rcsThreshold_valid` in byte 6).

| Field | Start bit | Width | Encoding |
|---|---|---|---|
| maxDistance | 22 | 10 | `raw = maxDistance / 2` (m) |
| sensorID | 32 | 3 | 0..7 |
| outputType | 35 | 2 | enum OutputType |
| radarPower | 37 | 3 | enum RadarPower |
| sortIndex | 44 | 3 | enum SortIndex |
| storeInNVM | 47 | 1 | write to NVM |
| rcsThreshold | 49 | 3 | enum RCSThreshold |

`0x201` RadarState mirrors these for readback (`maxDistanceCfg = raw*2`, plus `nvmReadOK`=bit6, `nvmWriteOK`=bit7).

## Region (collision zone)

`0x401` RegionConfig (encode): `maxOutputNumber` (bits 0-5), `activation` (6), `coordinatesValid` (7), `regionID` (8-10); rectangle corners `point1Long`(27,13)/`point1Lat`(32,11)/`point2Long`(51,13)/`point2Lat`(56,11) with `long_raw=(v+500)*5`, `lat_raw=(v+204.6)*5` (LSB 0.2 m).
`0x402` RegionState decodes the same with `maxOutputNumber`(0-5), `regionID`(6-7).

## Enums

```
OutputType    : None=0, Objects=1, Clusters=2
RadarPower    : Standard=0, -3dB=1, -6dB=2, -9dB=3
SortIndex     : NoSort=0, ByRange=1, ByRCS=2
RCSThreshold  : Standard=0, HighSensitivity=1
DynProp       : Moving=0, Stationary=1, Oncoming=2, StationaryCandidate=3,
                Unknown=4, CrossingStationary=5, CrossingMoving=6, Stopped=7
```
RadarConfig defaults: `maxDistance=80, sensorID=0, outputType=Objects, radarPower=Standard, sortIndex=ByRange, rcsThreshold=Standard`.

## Reading on Linux

```sh
sudo ip link set can0 up type can bitrate 500000
candump -L can0 | tee mr72.log
candump can0,60A:7FF          # status frames only (sensorID 0)
```

## MR82 note

MR82-UAV (80 GHz obstacle-avoidance) is not separately documented by ArduPilot, but its datasheet states the same CAN-up-to-500k interface and the same object output, so this family-A map is expected to apply. Practically this means `PRX_TYPE = 17 (MR72_CAN)` should pick it up; the only known difference is range (MR82 reaches 80 m vs the ArduPilot driver's 50 m clamp). Confirm on hardware.
