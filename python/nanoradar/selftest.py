"""Self-test against known vectors. No hardware or dependencies required.

Run from the ``python/`` directory:

    python3 -m nanoradar.selftest
"""

from __future__ import annotations

import math

from . import altimeter as alt
from . import object_can as oc


def _close(a: float, b: float, tol: float = 1e-6) -> bool:
    return math.isclose(a, b, abs_tol=tol)


def test_object_general() -> None:
    # Hand-built so that: distLong=50.0, distLat=-100.0, vrel*=0, dynProp=MOVING, rcs=0
    data = bytes([0x03, 0x55, 0xF2, 0x0B, 0x80, 0x20, 0x00, 0x80])
    o = oc.decode(0x60B, data)
    assert isinstance(o, oc.RadarObject)
    assert o.id == 3
    assert _close(o.dist_long, 50.0)
    assert _close(o.dist_lat, -100.0)
    assert _close(o.vrel_long, 0.0)
    assert _close(o.vrel_lat, 0.0)
    assert o.dyn_prop == oc.DynProp.MOVING
    assert _close(o.rcs, 0.0)
    assert _close(o.range_m, math.hypot(50.0, 100.0))


def test_object_list_status() -> None:
    data = bytes([0x05, 0x34, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00])
    s = oc.decode(0x60A, data)
    assert isinstance(s, oc.ObjectListStatus)
    assert s.n_objects == 5
    assert s.meas_count == 0x1234


def test_id_addressing() -> None:
    assert oc.split_id(0x61B) == (0x60B, 1)
    assert oc.split_id(0x60A) == (0x60A, 0)
    assert oc.split_id(0x123) == (None, 0)


def test_config_roundtrip() -> None:
    can_id, data = oc.encode_radar_config(oc.RadarConfig(max_distance_m=80), sensor_id=1)
    assert can_id == 0x210
    st = oc.decode_radar_state(data)        # config/state share the bit layout
    assert st.max_distance_m == 80


def test_altimeter_can_short() -> None:
    data = bytes([0x01, 0x00, 0x07, 0xD0, 0x00, 0x00, 0x00, 138])
    t = alt.decode_target_can(data)
    assert t.id == 1
    assert _close(t.distance_m, 20.0)
    assert t.snr == 10


def test_altimeter_can_long() -> None:
    # 3000 m needs the 20-bit field: raw = 300000 = 0x493E0 -> nibble 4, 0x93, 0xE0
    data = bytes([0x42, 0x00, 0x93, 0xE0, 0x00, 0x00, 0x00, 128])
    t = alt.decode_target_can(data)
    assert t.id == 2
    assert _close(t.distance_m, 3000.0)


def test_altimeter_serial_nra24_manual() -> None:
    # Worked example straight from the NRA24 manual (checksum 0x90)
    frame = bytes([0xAA, 0xAA, 0x0C, 0x07,
                   0x01, 0xC8, 0x07, 0xD0, 0x00, 0x02, 0xEE,
                   0x90, 0x55, 0x55])
    frames = list(alt.iter_serial_frames(frame))
    assert len(frames) == 1
    msg_id, t = frames[0]
    assert msg_id == alt.MSG_ALT_TARGET
    assert t.id == 1
    assert _close(t.distance_m, 20.0)
    assert _close(t.rcs, 50.0)


def test_altimeter_serial_stream_noise() -> None:
    # leading/trailing junk must not break framing
    frame = bytes([0xAA, 0xAA, 0x0C, 0x07,
                   0x01, 0xC8, 0x07, 0xD0, 0x00, 0x02, 0xEE, 0x90, 0x55, 0x55])
    buf = bytes([0x00, 0xAA, 0x12]) + frame + bytes([0x55, 0x99])
    assert len(list(alt.iter_serial_frames(buf))) == 1


def test_usd1() -> None:
    # distance 12.34 m -> raw 1234 = 0x04D2 ; checksum over b1..b4
    frame = bytes([0xFE, 0x00, 0xD2, 0x04, 0x00, 0xD6])
    out = list(alt.iter_usd1_frames(frame))
    assert len(out) == 1
    assert _close(out[0].distance_m, 12.34)


def main() -> int:
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for t in tests:
        try:
            t()
            print(f"  PASS  {t.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"  FAIL  {t.__name__}: {e}")
    print(f"\n{len(tests) - failed}/{len(tests)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
