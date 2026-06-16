[![Boosty](https://img.shields.io/badge/Boosty-Buy_me_a_coffee-FF7143?logo=boosty&logoColor=white&style=for-the-badge)](https://boosty.to/danusha/donate)

# NanoRadar Protocols (unofficial)

Community **reverse-engineered protocol reference** for [NanoRadar](https://www.nanoradar.com/) (Hunan/Nanjing NanoRadar) mmWave radars — CAN and UART/serial message maps, byte layouts and decode formulas, plus ArduPilot / PX4 integration notes.

These are sensors widely used on drones (obstacle avoidance + radar altimeters), but their protocol documentation is hard to find and not consistently published. This repo collects what could be recovered from **open-source drivers** and from **NanoRadar's own freely downloadable host tools**, so others don't have to redo the work.

> ⚠️ **Unofficial & partly reverse-engineered.** This is not affiliated with NanoRadar. The object/CAN protocol is cross-verified against three independent open-source drivers; the altimeter long-range decode is recovered by decompiling NanoRadar's own configurator tool and is **not yet verified on hardware**. Always confirm against a real device or the official protocol manual before relying on it. Corrections via PR/issue welcome.
>
> No vendor materials are redistributed here — only original documentation and links to the official sources.

## Two protocol families

NanoRadar splits its products into two protocol families. They share some message IDs (`0x60A` appears in both) but mean different things — do not mix them up.

| | **A — Object / Cluster** | **B — Altimeter** |
|---|---|---|
| Purpose | obstacle detection / avoidance (multi-target) | ground-relative height (single target) |
| Models | MR72, MR76, **MR82**, SR60/61/71/73/75, UCM211 | NRA12/15, NRA24, NRA24Pro, UAM231, **UAM285** |
| Lineage | derived from Continental **ARS408** | NanoRadar altimeter framing |
| CAN IDs | `0x200` cfg, `0x201` state, `0x60A` obj-list-status, `0x60B` object, `0x401/0x402` region, `0x700` fw | `0x60A` heartbeat, `0x70C` target |
| Target fields | range, azimuth, vrel, RCS, dyn-property, class | distance, SNR (single target) |
| Serial frame | — (CAN-centric) | `0xAA 0xAA \| id \| payload \| checksum \| 0x55 0x55` |
| Default CAN baud | 500 kbit/s | 500 kbit/s (250k / 1M configurable) |
| Host tool | `NSM77 Tools` (v1.3.x) | `NSM Tools` (v2.0.x) |
| ArduPilot type | `PRX_TYPE = 17` (MR72_CAN) | `RNGFNDx_TYPE` NRA24/CAN |

→ **[docs/object-can-protocol.md](docs/object-can-protocol.md)** — family A, full message map + decode formulas
→ **[docs/altimeter-protocol.md](docs/altimeter-protocol.md)** — family B, CAN + serial, incl. the 20-bit long-range distance field
→ **[docs/ardupilot-px4.md](docs/ardupilot-px4.md)** — what is supported today, open PRs, the long-range caveat
→ **[docs/sources.md](docs/sources.md)** — every source, with links
→ **[python/](python/)** — ready-to-use Python parsers for both families (`python3 -m nanoradar.selftest`, 9/9 vectors)
→ **[ardupilot/](ardupilot/)** — Lua CAN driver to run the UAM285 altimeter (full 3000 m / 20-bit) on ArduPilot **without recompiling**

## Quick reference

### Object CAN (MR72 / MR76 / MR82) — message IDs (radar id `n`: add `n*0x10`)
```
0x200  RadarConfig    (host → radar)   maxDistance, sensorID, outputType, radarPower, sortIndex, rcsThreshold, NVM
0x201  RadarState     (radar → host)   current config + nvmReadOK/nvmWriteOK
0x60A  ObjListStatus  (radar → host)   data[0] = number of objects in this cycle
0x60B  ObjectGeneral  (radar → host)   one frame per object (see formulas below)
0x401/0x402 Region cfg/state           collision region rectangle
0x700  FirmwareVersion                 major/minor/patch
```
`0x60B` object decode:
```
id       = data[0]
distLong = (data[1]*32 + (data[2]>>3))        * 0.2 - 500.0   # m (longitudinal)
distLat  = ((data[2]&0x07)*256 + data[3])     * 0.2 - 204.6   # m (lateral)
vrelLong = (data[4]*4 + (data[5]>>6))         * 0.25 - 128.0  # m/s
vrelLat  = ((data[5]&0x3F)*8 + (data[6]>>5))  * 0.25 - 64.0   # m/s
dynProp  = data[6] & 0x07
rcs      = data[7] * 0.5 - 64.0                                # dBsm
```

### Altimeter (NRA24 / UAM231 / UAM285) — CAN
```
0x60A  Radar status / heartbeat
0x70C  Target Info — two firmware variants (auto-detect via checksum in data[7]):
       A. no-checksum / 16-bit:  distance = (data[2]*256 + data[3]) * 0.01            # max 655 m
       B. checksum / 20-bit:     distance = (((data[0]&0xF0)<<12) | data[2]<<8 | data[3]) * 0.01  # up to ~10485 m
                                 checksum = data[7] == (sum(data[0..6]) & 0xFF)
       id = data[0] & 0x0F
```

### Altimeter — UART/serial (native 14-byte frame)
```
0xAA 0xAA | ID_lo ID_hi | b0 b1 b2 b3 b4 b5 b6 | checksum | 0x55 0x55
ID = ID_lo + ID_hi*0x100        # 0x0A 0x06 → 0x60A status ; 0x0C 0x07 → 0x70C target
checksum = 0xFF & (b0+b1+...+b6)
Target 0x70C: b0=Index, b1=RCS(*0.5-50), b2:b3=Range(*0.01 m)  (+ high nibble of b0 for long-range)
```

### Altimeter — "open-source" mode = USD1 / uLanding compatible
The altimeters can switch to an **Aerotenna/Ainstein US-D1 compatible** serial stream so they work with ArduPilot/PX4 **without code changes** — just select the existing USD1 driver (`RNGFNDx_TYPE = 11`, `SERIALx_PROTOCOL = 9`, 115200 baud):
```
0xFE | version | distL distH | b4 | checksum     # distance = (distH*256+distL)*0.01 m
```

### Altimeter models / ranges
`NRA24` 50–200 m · `NRA24Pro` ~300–500 m → **16-bit distance**;  `UAM231` 1000 m · `UAM285` 3000 m → **20-bit distance** (needs the high nibble of `data[0]`).

## Status & contributions

| Item | Confidence |
|---|---|
| Object CAN map (MR72/MR76/MR82) | ✅ cross-verified from 3 open-source drivers |
| Altimeter CAN/serial framing (`0x60A`/`0x70C`) | ✅ from ArduPilot + the official NanoRadar PX4 driver + manual |
| Altimeter 20-bit long-range distance + checksum | ✅ **confirmed by NanoRadar's own PX4 driver** (PR #25006) |
| MR82-specific fields | ⚠️ assumed same as MR72 family (datasheet confirms protocol family) |
| UAM285-specific scaling | ✅ same family as UAM231/UAM221, confirmed in the PX4 driver |

PRs/issues with hardware captures, corrections, or official protocol docs are very welcome. If a finding here helps you, see the Boosty button above ☕.

## License

Documentation in this repository is released under **CC-BY-4.0**. It contains only original notes and interoperability facts (message IDs, byte layouts, formulas); no NanoRadar datasheets, firmware, or tool binaries are included. NanoRadar and product names are trademarks of their respective owners.
