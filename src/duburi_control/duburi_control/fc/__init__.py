"""Flight-controller HAL.

`duburi_ws` grew around one autopilot (Pixhawk/ArduSub) reached through one
object (`duburi_control.pixhawk.Pixhawk`), constructed directly and passed
duck-typed into every `motion_*.py` function. That worked while there was
exactly one backend. There are now two: the SROT board running Hengla
firmware speaks MAVLink 2 as well, but with different verbs -- no
`RC_CHANNELS_OVERRIDE`, `MANUAL_CONTROL` instead, and a custom
`MAV_CMD_SROT_MOVE` that owns motion primitives the Python side used to run.

This package is the seam. `FlightController` (base.py) is the contract,
`PixhawkFC` and `SrotFC` are the two implementations, and `make_fc()` picks
between them from a ROS parameter.

Nothing else in the workspace has to change to gain the seam: both backends
expose the same duck-typed surface the motion layer already calls, so
`self.pixhawk = make_fc(...)` is the entire integration.

See `DUBURI_WS_INTEGRATION.md` in the srot-control-board repo for the
migration plan this implements, and `JETSON_COMMS.md` for the SROT wire
contract.
"""

from .base import FlightController, MoveHandle, MoveResult, Telemetry
from .factory import BUILDERS, make_fc

__all__ = [
    'FlightController',
    'MoveHandle',
    'MoveResult',
    'Telemetry',
    'make_fc',
    'BUILDERS',
]
