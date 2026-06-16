"""Family B - Altimeter protocol (NRA24 / NRA24Pro / UAM231 / UAM285).

Single-target ground-relative height. Three wire formats are covered:

* native CAN  - ``0x60A`` heartbeat + ``0x70C`` target
* native UART - 14-byte ``0xAA..0x55`` framed packets carrying the same ids
* "open-source" UART - Aerotenna/Ainstein US-D1 (uLanding) compatible stream

The distance field is up to 20 bits wide: the top 4 bits live in the high
nibble of the target-id byte. The 20-bit decode is a superset of the 16-bit
one (for <=655 m sensors the high nibble is 0). See ``docs/altimeter-protocol.md``.
"""

from __future__ import annotations

from dataclasses import dataclass

MSG_ALT_STATUS = 0x60A   # heartbeat
MSG_ALT_TARGET = 0x70C   # target info
SENSOR_MSG_OFFSET = 0x10

# native serial framing
SER_HEADER = 0xAA
SER_END = 0x55
SER_FRAME_LEN = 14

# USD1 / uLanding ("open-source" mode)
USD1_HDR_V1 = 0xFE
USD1_HDR_V0 = 0x48


@dataclass
class AltimeterTarget:
    id: int
    distance_m: float
    rcs: float | None = None     # dBsm (serial frame); None on CAN
    snr: int | None = None       # CAN frame: data[7] - 128
    roll_count: int | None = None
    raw_distance: int = 0        # raw 20-bit count (x0.01 m)


def _decode_distance20(b0: int, b2: int, b3: int) -> int:
    """20-bit raw distance: high nibble of the id byte + the two range bytes."""
    return (((b0 >> 4) & 0x0F) << 16) | (b2 << 8) | b3


# --- native CAN -------------------------------------------------------------

def is_status_id(can_id: int) -> bool:
    delta = can_id - MSG_ALT_STATUS
    return 0 <= delta <= 7 * SENSOR_MSG_OFFSET and delta % SENSOR_MSG_OFFSET == 0


def is_target_id(can_id: int) -> bool:
    delta = can_id - MSG_ALT_TARGET
    return 0 <= delta <= 7 * SENSOR_MSG_OFFSET and delta % SENSOR_MSG_OFFSET == 0


def decode_target_can(data: bytes) -> AltimeterTarget:
    raw = _decode_distance20(data[0], data[2], data[3])
    return AltimeterTarget(
        id=data[0] & 0x0F,
        distance_m=raw * 0.01,
        snr=data[7] - 128,
        raw_distance=raw,
    )


# --- native UART (0xAA .. 0x55 framed) --------------------------------------

def decode_target_serial_payload(payload: bytes) -> AltimeterTarget:
    """Decode the 7-byte payload (b0..b6) of a 0x70C serial target frame."""
    raw = _decode_distance20(payload[0], payload[2], payload[3])
    return AltimeterTarget(
        id=payload[0] & 0x0F,
        distance_m=raw * 0.01,
        rcs=payload[1] * 0.5 - 50.0,
        roll_count=(payload[5] & 0xE0) >> 5,   # b5/b6 semantics not fully confirmed
        raw_distance=raw,
    )


def iter_serial_frames(buf: bytes):
    """Yield ``(msg_id, AltimeterTarget | None)`` for each valid 14-byte frame in ``buf``.

    Stream tolerant: skips bytes until a ``0xAA 0xAA`` header that is followed by a
    valid checksum and ``0x55 0x55`` terminator.
    """
    i, n = 0, len(buf)
    while i + SER_FRAME_LEN <= n:
        if buf[i] == SER_HEADER and buf[i + 1] == SER_HEADER:
            frame = buf[i:i + SER_FRAME_LEN]
            payload = frame[4:11]              # b0..b6
            checksum = frame[11]
            if (frame[12] == SER_END and frame[13] == SER_END
                    and (sum(payload) & 0xFF) == checksum):
                msg_id = frame[2] | (frame[3] << 8)
                target = decode_target_serial_payload(payload) if msg_id == MSG_ALT_TARGET else None
                yield msg_id, target
                i += SER_FRAME_LEN
                continue
        i += 1


# --- USD1 / uLanding "open-source" mode -------------------------------------

@dataclass
class USD1Reading:
    distance_m: float


def iter_usd1_frames(buf: bytes):
    """Yield ``USD1Reading`` for each valid 6-byte US-D1 v1 frame in ``buf``.

    Frame: ``0xFE | b1 | distL | distH | b4 | checksum``,
    ``distance = (distH*256 + distL) * 0.01``, ``checksum = (b1+distL+distH+b4) & 0xFF``.
    """
    i, n = 0, len(buf)
    while i + 6 <= n:
        if buf[i] == USD1_HDR_V1:
            f = buf[i:i + 6]
            if ((f[1] + f[2] + f[3] + f[4]) & 0xFF) == f[5]:
                yield USD1Reading(distance_m=(f[3] * 256 + f[2]) * 0.01)
                i += 6
                continue
        i += 1
