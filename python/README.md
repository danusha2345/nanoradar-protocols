# Python reference parsers

Pure-Python, dependency-free decoders for both NanoRadar protocol families. They
implement exactly the message maps in [`../docs`](../docs) and are verified
against known vectors (including the NRA24 manual's worked example) — no hardware
needed.

## Layout
```
nanoradar/
  object_can.py   # family A: MR72/MR76/MR82 — 0x60A/0x60B/0x200/0x201/0x700
  altimeter.py    # family B: NRA24/UAM231/UAM285 — CAN 0x70C, serial 0xAA frame, USD1
  bits.py         # LSB0 bit-field helpers
  selftest.py     # known-vector tests
examples/
  decode_candump.py   # decode a `candump -L` log / live stream
```

## Run the self-test
```sh
cd python
python3 -m nanoradar.selftest      # -> 9/9 passed
```

## Quick usage

Object frame (MR72/MR76/MR82):
```python
from nanoradar import object_can as oc

obj = oc.decode(0x60B, bytes([0x03, 0x55, 0xF2, 0x0B, 0x80, 0x20, 0x00, 0x80]))
print(obj.dist_long, obj.dist_lat, obj.range_m, obj.azimuth_deg)   # 50.0 -100.0 ...

can_id, payload = oc.encode_radar_config(oc.RadarConfig(max_distance_m=120, sensor_id=0))
```

Altimeter (NRA24 / UAM231 / UAM285):
```python
from nanoradar import altimeter as alt

# CAN target (20-bit decode works for both short- and long-range models)
t = alt.decode_target_can(bytes([0x42, 0x00, 0x93, 0xE0, 0, 0, 0, 128]))
print(t.distance_m)            # 3000.0

# native UART stream (0xAA .. 0x55 framed)
for msg_id, target in alt.iter_serial_frames(raw_bytes):
    if target:
        print(target.distance_m, target.rcs)

# "open-source" USD1 / uLanding stream
for r in alt.iter_usd1_frames(raw_bytes):
    print(r.distance_m)
```

## Decode a candump log
```sh
candump -L can0 | python3 examples/decode_candump.py --family auto
# or:  python3 examples/decode_candump.py --family object capture.log
```

## Caveats
- The altimeter `distance` uses the 20-bit form (high nibble of `data[0]` → bits 16-19); for ≤655 m sensors that nibble is 0 so it matches the 16-bit decode. This is reverse-engineered from the vendor tool and **not yet hardware-verified** for UAM285.
- In the serial target frame, `b5`/`b6` (roll count / speed) are tentative; `id`, `RCS` and `range` are confirmed.
- `0x60A` exists in both families with different meaning — pass `--family` to disambiguate rather than relying on `auto`.
