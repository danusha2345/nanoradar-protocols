"""Reference parsers for NanoRadar mmWave radar protocols.

Two families:
  * ``object_can`` - MR72 / MR76 / MR82 obstacle/object output (CAN)
  * ``altimeter``  - NRA24 / NRA24Pro / UAM231 / UAM285 height (CAN + UART + USD1)

Pure Python, no dependencies. See the repo docs for the protocol details.
"""

from __future__ import annotations

from . import altimeter, object_can

__all__ = ["object_can", "altimeter"]
__version__ = "0.1.0"
