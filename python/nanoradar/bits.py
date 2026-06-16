"""Little-endian (LSB0) bit field helpers.

These match the bit numbering used by the NanoRadar object/CAN protocol
(and the ``mr72-radar-library`` reference C++ implementation): bit ``n`` is
bit ``n % 8`` of data byte ``n // 8``, counting from the LSB.
"""

from __future__ import annotations


def extract_bits(data: bytes | bytearray, start: int, length: int) -> int:
    """Read ``length`` bits starting at absolute bit ``start`` (LSB0)."""
    result = 0
    for i in range(length):
        bitpos = start + i
        if (data[bitpos >> 3] >> (bitpos & 7)) & 1:
            result |= 1 << i
    return result


def insert_bits(data: bytearray, start: int, length: int, value: int) -> None:
    """Write the low ``length`` bits of ``value`` starting at bit ``start`` (LSB0)."""
    for i in range(length):
        bitpos = start + i
        byte, bit = bitpos >> 3, bitpos & 7
        if (value >> i) & 1:
            data[byte] |= 1 << bit
        else:
            data[byte] &= ~(1 << bit)
