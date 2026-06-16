# ArduPilot Lua drivers

No-compile way to run NanoRadar radars on ArduPilot: a Lua script reads the CAN
frames, decodes them, and feeds ArduPilot through a scripting backend. Drop the
`.lua` on the SD card (`APM/scripts/`), set a few parameters, reboot.

## `nanoradar_altimeter_can_rangefinder.lua`

Altimeter (NRA24 / NRA24Pro / UAM231 / **UAM285**) → ArduPilot rangefinder, over CAN.

Why a script instead of the built-in `NRA24_CAN` driver: the stock driver decodes
distance as **16-bit** (caps at ~655 m). This script uses the full **20-bit** decode
(`(((data[0]>>4)&0x0F)<<16 | data[2]<<8 | data[3]) * 0.01`), so it covers the
long-range models — UAM231 (1000 m) and UAM285 (3000 m).

### Parameters
```
SCR_ENABLE      = 1          # enable Lua scripting (reboot)
CAN_P1_DRIVER   = 1          # enable CAN port 1
CAN_D1_PROTOCOL = 10         # Scripting (Lua owns the bus)
CAN_D1_BITRATE  = 500000     # match the radar (NSM tool default)
RNGFND1_TYPE    = 36         # Lua_Scripting (reboot)
RNGFND1_ORIENT  = 25         # down
RNGFND1_MIN     = 0.20
RNGFND1_MAX     = 3000       # set to your model's range
```
Copy the script to `APM/scripts/` and reboot. You should see
`NanoRadar altimeter Lua driver started` in the messages, and a live value in
Mission Planner → Flight Data → Status → `rangefinder1`.

### Status
- Lua syntax checked (`luac5.4 -p`) and the decode is runtime-tested against the
  20-bit (3000 m) and 16-bit (20 m) vectors (see the harness in the repo history).
- ⚠️ The 20-bit decode is reverse-engineered from the vendor tool and **not yet
  verified on real UAM285 hardware**. Bench-test before flight. `b5`/`b6`
  (roll/speed) are ignored; only id + distance are used.
- Needs a board with scripting enabled and enough flash/RAM (F7/H7).

## MR82 (obstacle avoidance) — usually no script needed

MR82 speaks the family-A object protocol, which the built-in proximity driver
already handles: set `PRX_TYPE = 17 (MR72_CAN)`, `CAN_Dx_PROTOCOL = 14`, bitrate
500000. The only limit is the driver's 50 m range clamp (MR82 reaches 80 m); a
proximity Lua driver could lift that, but for avoidance the built-in path is fine.
