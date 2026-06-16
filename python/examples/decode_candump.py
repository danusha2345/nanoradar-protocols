#!/usr/bin/env python3
"""Decode a ``candump -L`` log (or live ``candump`` output) into radar targets.

Usage:
    candump -L can0 | python3 examples/decode_candump.py --family auto
    python3 examples/decode_candump.py --family object  capture.log

``candump -L`` lines look like:  ``(1700000000.000000) can0 60B#0102030405060708``
Plain ``candump`` lines look like: ``can0  60B   [8]  01 02 03 04 05 06 07 08``
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from nanoradar import altimeter as alt          # noqa: E402
from nanoradar import object_can as oc           # noqa: E402

_LOG = re.compile(r"\)\s+\S+\s+([0-9A-Fa-f]+)#([0-9A-Fa-f]*)")
_PLAIN = re.compile(r"^\s*\S+\s+([0-9A-Fa-f]+)\s+\[\d+\]\s+([0-9A-Fa-f ]+)")


def parse_line(line: str):
    m = _LOG.search(line)
    if m:
        return int(m.group(1), 16), bytes.fromhex(m.group(2))
    m = _PLAIN.match(line)
    if m:
        return int(m.group(1), 16), bytes.fromhex(m.group(2).replace(" ", ""))
    return None


def decode(can_id: int, data: bytes, family: str):
    if family in ("object", "auto"):
        obj = oc.decode(can_id, data)
        if obj is not None:
            return obj
    if family in ("altimeter", "auto"):
        if alt.is_target_id(can_id) and len(data) >= 8:
            return alt.decode_target_can(data)
        if alt.is_status_id(can_id):
            return f"altimeter heartbeat (id 0x{can_id:X})"
    return None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("file", nargs="?", help="candump log file (default: stdin)")
    ap.add_argument("--family", choices=["object", "altimeter", "auto"], default="auto")
    args = ap.parse_args()

    stream = open(args.file) if args.file else sys.stdin
    for line in stream:
        parsed = parse_line(line)
        if not parsed:
            continue
        can_id, data = parsed
        result = decode(can_id, data, args.family)
        if result is not None:
            print(f"0x{can_id:03X}  {result}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
