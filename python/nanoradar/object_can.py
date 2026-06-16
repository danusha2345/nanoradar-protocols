"""Family A - Object/Cluster CAN protocol (MR72 / MR76 / MR82).

Derived from the Continental ARS408 protocol. Verified against the
``mr72-radar-library`` C++ source and ArduPilot's ``AP_Proximity_MR72_CAN``.
See ``docs/object-can-protocol.md``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import IntEnum

from .bits import extract_bits, insert_bits

# Base message IDs (sensor id 0). Actual id = base + sensor_id * SENSOR_MSG_OFFSET.
MSG_RADAR_CFG = 0x200
MSG_RADAR_STATE = 0x201
MSG_REGION_CFG = 0x401
MSG_REGION_STATE = 0x402
MSG_OBJ_STATUS = 0x60A
MSG_OBJ_GENERAL = 0x60B
MSG_VERSION = 0x700
SENSOR_MSG_OFFSET = 0x10

_BASE_IDS = (
    MSG_RADAR_CFG, MSG_RADAR_STATE, MSG_REGION_CFG, MSG_REGION_STATE,
    MSG_OBJ_STATUS, MSG_OBJ_GENERAL, MSG_VERSION,
)


class OutputType(IntEnum):
    NONE = 0
    OBJECTS = 1
    CLUSTERS = 2


class RadarPower(IntEnum):
    STANDARD = 0
    MINUS_3DB = 1
    MINUS_6DB = 2
    MINUS_9DB = 3


class SortIndex(IntEnum):
    NO_SORT = 0
    BY_RANGE = 1
    BY_RCS = 2


class RCSThreshold(IntEnum):
    STANDARD = 0
    HIGH_SENSITIVITY = 1


class DynProp(IntEnum):
    MOVING = 0
    STATIONARY = 1
    ONCOMING = 2
    STATIONARY_CANDIDATE = 3
    UNKNOWN = 4
    CROSSING_STATIONARY = 5
    CROSSING_MOVING = 6
    STOPPED = 7


def split_id(can_id: int) -> tuple[int | None, int]:
    """Map a CAN id to ``(base_id, sensor_id)``; ``(None, 0)`` if not a known frame.

    Bases differ by 1 while sensors are spaced 0x10 apart, so this is unambiguous.
    """
    for base in _BASE_IDS:
        delta = can_id - base
        if 0 <= delta <= 7 * SENSOR_MSG_OFFSET and delta % SENSOR_MSG_OFFSET == 0:
            return base, delta // SENSOR_MSG_OFFSET
    return None, 0


@dataclass
class ObjectListStatus:
    sensor_id: int
    n_objects: int
    meas_count: int
    interface_version: int


@dataclass
class RadarObject:
    sensor_id: int
    id: int
    dist_long: float   # m, longitudinal
    dist_lat: float    # m, lateral
    vrel_long: float   # m/s
    vrel_lat: float    # m/s
    dyn_prop: DynProp
    sector: int
    rcs: float         # dBsm

    @property
    def range_m(self) -> float:
        return math.hypot(self.dist_long, self.dist_lat)

    @property
    def azimuth_deg(self) -> float:
        return math.degrees(math.atan2(self.dist_lat, self.dist_long))


@dataclass
class RadarState:
    sensor_id: int
    max_distance_m: int
    output_type: OutputType
    radar_power: RadarPower
    sort_index: SortIndex
    rcs_threshold: RCSThreshold
    nvm_read_ok: bool
    nvm_write_ok: bool


@dataclass
class FirmwareVersion:
    sensor_id: int
    major: int
    minor: int
    patch: int


def decode_object_list_status(data: bytes, sensor_id: int = 0) -> ObjectListStatus:
    return ObjectListStatus(
        sensor_id=sensor_id,
        n_objects=data[0],
        meas_count=extract_bits(data, 8, 16),
        interface_version=extract_bits(data, 28, 4),
    )


def decode_object(data: bytes, sensor_id: int = 0) -> RadarObject:
    return RadarObject(
        sensor_id=sensor_id,
        id=data[0],
        dist_long=(data[1] * 32 + (data[2] >> 3)) * 0.2 - 500.0,
        dist_lat=((data[2] & 0x07) * 256 + data[3]) * 0.2 - 204.6,
        vrel_long=(data[4] * 4 + (data[5] >> 6)) * 0.25 - 128.0,
        vrel_lat=((data[5] & 0x3F) * 8 + (data[6] >> 5)) * 0.25 - 64.0,
        dyn_prop=DynProp(data[6] & 0x07),
        sector=(data[6] >> 3) & 0x03,
        rcs=data[7] * 0.5 - 64.0,
    )


def decode_radar_state(data: bytes, sensor_id: int = 0) -> RadarState:
    return RadarState(
        sensor_id=sensor_id,
        max_distance_m=extract_bits(data, 22, 10) * 2,
        output_type=OutputType(extract_bits(data, 42, 2)),
        radar_power=RadarPower(extract_bits(data, 39, 3)),
        sort_index=SortIndex(extract_bits(data, 36, 3)),
        rcs_threshold=RCSThreshold(extract_bits(data, 58, 3)),
        nvm_read_ok=bool(extract_bits(data, 6, 1)),
        nvm_write_ok=bool(extract_bits(data, 7, 1)),
    )


def decode_firmware_version(data: bytes, sensor_id: int = 0) -> FirmwareVersion:
    return FirmwareVersion(sensor_id=sensor_id, major=data[0], minor=data[1], patch=data[2])


def decode(can_id: int, data: bytes):
    """Dispatch a raw object-protocol frame to the right decoder.

    Returns a typed dataclass, or ``None`` if the id is not a known frame.
    """
    base, sid = split_id(can_id)
    if base == MSG_OBJ_STATUS:
        return decode_object_list_status(data, sid)
    if base == MSG_OBJ_GENERAL:
        return decode_object(data, sid)
    if base == MSG_RADAR_STATE:
        return decode_radar_state(data, sid)
    if base == MSG_VERSION:
        return decode_firmware_version(data, sid)
    return None


@dataclass
class RadarConfig:
    max_distance_m: int = 80
    sensor_id: int = 0
    output_type: OutputType = OutputType.OBJECTS
    radar_power: RadarPower = RadarPower.STANDARD
    sort_index: SortIndex = SortIndex.BY_RANGE
    rcs_threshold: RCSThreshold = RCSThreshold.STANDARD
    store_in_nvm: bool = False


def encode_radar_config(cfg: RadarConfig, sensor_id: int = 0) -> tuple[int, bytes]:
    """Encode a ``0x200`` RadarConfig frame. Returns ``(can_id, 8 bytes)``.

    All "valid" flags are set so every field is applied.
    """
    data = bytearray(8)
    # byte 0: one "valid" flag per field
    for bit in range(8):
        insert_bits(data, bit, 1, 1)
    insert_bits(data, 22, 10, cfg.max_distance_m // 2)
    insert_bits(data, 32, 3, cfg.sensor_id & 0x7)
    insert_bits(data, 35, 2, int(cfg.output_type) & 0x3)
    insert_bits(data, 37, 3, int(cfg.radar_power) & 0x7)
    insert_bits(data, 44, 3, int(cfg.sort_index) & 0x7)
    insert_bits(data, 47, 1, 1 if cfg.store_in_nvm else 0)
    insert_bits(data, 48, 1, 1)  # rcsThreshold_valid
    insert_bits(data, 49, 3, int(cfg.rcs_threshold) & 0x7)
    can_id = MSG_RADAR_CFG + sensor_id * SENSOR_MSG_OFFSET
    return can_id, bytes(data)
