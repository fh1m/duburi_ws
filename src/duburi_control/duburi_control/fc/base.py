"""Abstract FlightController -- every autopilot backend implements this.

Contract
--------
The surface below is deliberately in two halves, because the migration is
incremental and both halves have to work at once.

**Intent API** (`arm`, `disarm`, `set_mode`, `manual`, `move`, `telemetry`)
    What new code should call. Expresses what the vehicle should do, not
    which channel carries it. `move()` is the interesting one: on SROT it is
    a single on-board primitive, on Pixhawk it is unimplemented because
    ArduSub has no equivalent and the Python motion layer does that job.

**Legacy channel API** (`send_rc_override`, `send_rc_translation`,
`send_neutral`, `release_rc_override`, `set_target_depth`)
    What `motion_*.py`, `heading_lock.py` and `duburi.py` call today. Kept on
    the ABC rather than quarantined, because deleting it would mean rewriting
    the whole motion layer in one step -- exactly the flag-day this HAL
    exists to avoid. `SrotFC` implements these by translating to
    `MANUAL_CONTROL`, so existing motion code runs unmodified on either
    backend.

    The translation is not lossless and the difference matters:
    `NO_OVERRIDE` (65535) means "release this axis to the autopilot" under RC
    override, and SROT has no equivalent -- `MANUAL_CONTROL` always carries
    all four axes. `SrotFC` holds a released axis at neutral instead. Any
    code that depends on release semantics (the ALT_HOLD depth handoff in
    `motion_depth.py`, Ch4 release in `heading_lock.py`) must switch to a
    mode change or a `SROT_MOVE` before it is correct on SROT. Those are
    steps 5-6 of the migration, not step 1.

Every telemetry getter is a **non-blocking cache read**. Only the reader
thread in `auv_manager_node` calls `recv_match()`; nothing here blocks on the
wire. Getters return `None` when no sample has arrived rather than raising or
blocking -- callers hold the last good value for that tick.

`arm`, `disarm` and `set_mode` return `(success, reason)` where `reason` is a
short uppercase token suitable for a log line or an action result message.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple


# Terminal results a move can resolve to. These mirror MAV_RESULT names
# because SROT reports them literally; the Pixhawk backend never produces
# them (it has no native move).
#
# TEMP_REJECTED is deliberately NOT here. SROT returns it when it could not
# take its control mutex in time, which is a "try again", not an outcome. A
# client that treats it as terminal aborts a move that was never started.
MOVE_TERMINAL = ('ACCEPTED', 'CANCELLED', 'FAILED', 'DENIED')


@dataclass
class MoveResult:
    """How a move ended.

    `success` is True only for ACCEPTED. CANCELLED means preempted by a
    later goal -- map it to *abort*, not *error*: it is the documented way
    to interrupt a move, and treating it as a fault makes normal preemption
    look like a failure.
    """

    result: str = 'FAILED'          # one of MOVE_TERMINAL
    success: bool = False
    message: str = ''
    progress: int = 0               # last progress seen, 0..100

    @property
    def preempted(self) -> bool:
        return self.result == 'CANCELLED'


@dataclass
class Telemetry:
    """One snapshot of vehicle state. Fields are None when unknown.

    None is not a placeholder for zero. `depth_m = None` means no depth
    sample has arrived; `depth_m = 0.0` means the vehicle is at the surface.
    Collapsing the two is how a stale reading gets used as a live one.
    """

    armed: Optional[bool] = None
    mode: Optional[str] = None
    yaw_deg: Optional[float] = None
    roll_deg: Optional[float] = None
    pitch_deg: Optional[float] = None
    depth_m: Optional[float] = None
    battery_voltage: Optional[float] = None
    # Thruster-pack voltage (SROT BATTERY_STATUS id 1, from the 2nd board
    # over ESP-NOW). None whenever that link is not fresh -- it is absent,
    # never held at its last value, so None here means "no data" and not
    # "flat pack". Always None on Pixhawk.
    thruster_voltage: Optional[float] = None
    # Per-thruster RPM from bidirectional DShot. SROT only; empty on Pixhawk.
    rpm: List[int] = field(default_factory=list)
    # Age of the newest attitude sample, seconds. Use it to gate control.
    attitude_age: Optional[float] = None


class MoveHandle:
    """A move in flight.

    Yielded by `FlightController.move()`. Poll `update()` on your feedback
    tick; it returns the current progress 0..100 and sets `.result` once the
    move reaches a terminal state.

    The whole point of this type is that **every move reaches exactly one
    terminal result**. An action client that waits only for ACCEPTED hangs
    forever on a preempted move (CANCELLED) or an unstartable one (FAILED).
    `done` is true for all four.
    """

    def __init__(self, verb: str):
        self.verb = verb
        self.progress: int = 0
        self.result: Optional[MoveResult] = None

    @property
    def done(self) -> bool:
        return self.result is not None

    def update(self) -> int:
        """Poll once. Returns progress 0..100; sets `.result` when terminal."""
        raise NotImplementedError

    def cancel(self) -> None:
        """Ask the vehicle to stop. Resolution still arrives via update()."""
        raise NotImplementedError

    def __repr__(self) -> str:
        state = self.result.result if self.result else f'{self.progress}%'
        return f'<{self.__class__.__name__} {self.verb} {state}>'


class FlightController:
    """See the module docstring for the contract."""

    name: str = 'base'

    #: True when the backend implements RC_CHANNELS_OVERRIDE natively, with
    #: real per-channel release. False means the channel API is emulated and
    #: NO_OVERRIDE degrades to neutral -- check this before relying on a
    #: channel handoff.
    supports_rc_override: bool = False

    #: True when `move()` runs a primitive on the vehicle. False means the
    #: Python motion layer must do it.
    supports_native_move: bool = False

    # ---- lifecycle -----------------------------------------------------
    def close(self) -> None:
        """Release the link. Safe to call more than once."""

    # ---- arming --------------------------------------------------------
    def arm(self, timeout: float = 15.0) -> Tuple[bool, str]:
        raise NotImplementedError

    def disarm(self, timeout: float = 15.0) -> Tuple[bool, str]:
        raise NotImplementedError

    def is_armed(self) -> bool:
        return False

    # ---- modes ---------------------------------------------------------
    def set_mode(self, mode_name: str, timeout: float = 8.0) -> Tuple[bool, str]:
        raise NotImplementedError

    def get_mode(self) -> str:
        return 'UNKNOWN'

    # ---- intent actuation ----------------------------------------------
    def manual(self, fwd: float = 0.0, lat: float = 0.0,
               up: float = 0.0, yaw: float = 0.0) -> None:
        """Normalised pilot input, each axis -1..1. `up` positive = ascend."""
        raise NotImplementedError

    def move(self, verb: str, **kw: Any) -> MoveHandle:
        """Run a motion primitive on the vehicle.

        Raises NotImplementedError on backends without native moves; check
        `supports_native_move` first.
        """
        raise NotImplementedError(
            f'{self.name} has no native move -- use the Python motion layer')

    # ---- legacy channel actuation --------------------------------------
    def send_rc_override(self, pitch: int = 1500, roll: int = 1500,
                         throttle: int = 1500, yaw: int = 1500,
                         forward: int = 1500, lateral: int = 1500) -> None:
        raise NotImplementedError

    def send_rc_translation(self, throttle: int = 1500,
                            forward: int = 1500, lateral: int = 1500) -> None:
        raise NotImplementedError

    def send_neutral(self) -> None:
        raise NotImplementedError

    def release_rc_override(self) -> None:
        raise NotImplementedError

    def set_target_depth(self, depth_m: float) -> None:
        raise NotImplementedError

    # ---- misc actuation ------------------------------------------------
    def set_servo_pwm(self, aux_n: int, pwm: int) -> None:
        raise NotImplementedError

    def send_heartbeat(self) -> None:
        raise NotImplementedError

    def set_message_rate(self, message_id: int, hz: float) -> None:
        """Best-effort stream-rate pin. A backend may ignore it."""

    # ---- telemetry (non-blocking cache reads) --------------------------
    def telemetry(self) -> Telemetry:
        raise NotImplementedError

    def get_attitude(self) -> Optional[Dict[str, float]]:
        """{'yaw','roll','pitch','depth'} in degrees / metres, or None."""
        raise NotImplementedError

    def get_attitude_age(self) -> Optional[float]:
        raise NotImplementedError

    def get_battery(self) -> Optional[Dict[str, float]]:
        raise NotImplementedError

    def get_rc_channels(self) -> Optional[List[int]]:
        return None

    def get_statustext(self) -> Optional[str]:
        return None

    # ---- pure helpers, identical on every backend ----------------------
    # Kept here so callers can reach them off the HAL instead of importing
    # the concrete Pixhawk class. Nine modules import `Pixhawk` today purely
    # for these two static methods; that is what made the class look load
    # bearing when it is not.
    @staticmethod
    def percent_to_pwm(percent: float) -> int:
        return max(1100, min(1900, int(1500 + (percent / 100.0) * 400)))

    @staticmethod
    def heading_error(target: float, current: float) -> float:
        """Signed shortest-path error in degrees, -180..180."""
        return (target - current + 540) % 360 - 180

    def __repr__(self) -> str:
        return f'<{self.__class__.__name__} name={self.name}>'
