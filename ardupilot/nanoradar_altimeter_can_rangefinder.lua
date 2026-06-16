--[[
  NanoRadar altimeter -> ArduPilot RangeFinder (Lua CAN driver)

  Reads the NanoRadar altimeter CAN "Target Info" frame (0x70C) and feeds the
  measured height to ArduPilot as a scripting rangefinder backend. No firmware
  recompile is needed.

  Works for the altimeter family: NRA24 / NRA24Pro / UAM231 / UAM285.
  It uses the full 20-bit distance decode, so it covers the long-range models
  (UAM231 1000 m, UAM285 3000 m) that the built-in 16-bit drivers cannot.

  Distance (per docs/altimeter-protocol.md, verified against the vendor tool):
      id        = data[0] & 0x0F
      distance  = ( ((data[0]>>4)&0x0F)<<16 | data[2]<<8 | data[3] ) * 0.01  metres
  For <=655 m sensors the high nibble is 0, so this also matches the 16-bit form.

  -------------------------------------------------------------------------
  SETUP (parameters), example for the first scripting CAN port + RNGFND1:
      SCR_ENABLE       = 1            -- enable Lua scripting (reboot)
      CAN_P1_DRIVER    = 1            -- enable CAN port 1
      CAN_D1_PROTOCOL  = 10           -- "Scripting" (so Lua owns the bus)
      CAN_D1_BITRATE   = 500000       -- match the radar (NSM tool default)
      RNGFND1_TYPE     = 36           -- Lua_Scripting (reboot)
      RNGFND1_MIN      = 0.20         -- m
      RNGFND1_MAX      = 3000         -- m (set to your model's range)
      RNGFND1_ORIENT   = 25           -- down (for an altimeter)
  Put this file in  APM/scripts/  on the SD card and reboot.
  Confirm "NanoRadar altimeter Lua driver started" appears in the messages.

  The 20-bit decode + data[7] checksum match NanoRadar's own PX4 driver
  (OuYangLei92/PX4-Autopilot, PR #25006), so the math is vendor-confirmed.
  Still bench-test your wiring/IDs before flight. b5/b6 (roll/speed) are ignored.
--]]

---@diagnostic disable: undefined-global
---@diagnostic disable: need-check-nil

-- ---- user-settable ----
local UPDATE_MS       = 20      -- driver loop period (ms); 20 ms = 50 Hz
local CAN_BUFFER_LEN  = 8       -- received-frame buffer depth
local DEBUG           = false   -- true to print each decoded reading
local DIST_MIN_M      = 0.10    -- ignore readings below this (m)
local DIST_MAX_M      = 5400.0  -- ignore readings above this (m)
local REQUIRE_CHECKSUM = false  -- true: only accept checksum-valid (20-bit) frames;
                                -- false: also accept old no-checksum (16-bit) firmware

-- ---- constants ----
local RFND_LUA_TYPE   = 36      -- RNGFNDx_TYPE for Lua scripting backend
local MSG_TARGET_LOW  = 0x0C    -- low nibble of the 0x70C target id
local SEVERITY_INFO   = 6
local SEVERITY_ERR    = 3

-- ---- state ----
local can_driver = CAN:get_device(CAN_BUFFER_LEN)
local backend                       -- the Lua rangefinder backend
local backend_found = false

gcs:send_text(SEVERITY_INFO, "NanoRadar altimeter Lua driver started")

-- Bind to the RNGFNDx instance whose TYPE = 36 (Lua_Scripting)
local function find_backend()
  for i = 0, rangefinder:num_sensors() - 1 do
    local dev = rangefinder:get_backend(i)
    if dev and dev:type() == RFND_LUA_TYPE then
      backend = dev
      backend_found = true
      return
    end
  end
  gcs:send_text(SEVERITY_ERR, "NanoRadar: no RNGFND with TYPE=36 (Lua). Set it and reboot.")
end

-- Is this a NanoRadar altimeter Target Info frame (0x70C, any radar id 0..7)?
local function is_target_frame(id)
  return (id & 0x0F) == MSG_TARGET_LOW and id >= 0x70C and id <= 0x77C
end

-- checksum = low byte of sum(data[0..6]); the long-range firmware puts it in data[7]
local function frame_checksum(frame)
  local s = 0
  for i = 0, 6 do
    s = s + frame:data(i)
  end
  return s & 0xFF
end

-- Decode distance (metres). The official NanoRadar driver auto-detects two
-- firmware variants by the data[7] checksum:
--   * checksum valid -> 20-bit decode (high nibble of data[0] = bits 16..19)
--   * otherwise      -> 16-bit decode (data[2..3] only), <=655 m models
-- Returns nil if the checksum is required but fails.
local function decode_distance_m(frame)
  local b0, b2, b3 = frame:data(0), frame:data(2), frame:data(3)
  if frame_checksum(frame) == frame:data(7) then
    local raw = (((b0 >> 4) & 0x0F) << 16) | (b2 << 8) | b3   -- 20-bit
    return raw * 0.01
  elseif not REQUIRE_CHECKSUM then
    return ((b2 << 8) | b3) * 0.01                            -- 16-bit fallback
  end
  return nil
end

local function update()
  if not backend_found then
    find_backend()
    if not backend_found then
      return
    end
  end

  -- drain everything queued this tick so we don't lag behind the radar
  local frame = can_driver:read_frame()
  while frame do
    if is_target_frame(frame:id()) then
      local dist_m = decode_distance_m(frame)
      if dist_m and dist_m >= DIST_MIN_M and dist_m <= DIST_MAX_M then
        if DEBUG then
          gcs:send_text(SEVERITY_INFO, string.format("NanoRadar alt: %.2f m", dist_m))
        end
        if not backend:handle_script_msg(dist_m) then
          gcs:send_text(SEVERITY_ERR, "NanoRadar: handle_script_msg failed")
        end
      end
    end
    frame = can_driver:read_frame()
  end
end

-- fault-tolerant wrapper: a runtime error slows the loop instead of killing it
local function protected_wrapper()
  local ok, err = pcall(update)
  if not ok then
    gcs:send_text(SEVERITY_ERR, "NanoRadar driver error: " .. tostring(err))
    return protected_wrapper, 1000
  end
  return protected_wrapper, UPDATE_MS
end

return protected_wrapper()
