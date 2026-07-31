"""PixhawkFC -- the existing ArduSub backend, behind the HAL.

This is a delegation shim, not a rewrite. `duburi_control/pixhawk.py` stays
exactly where it is and keeps doing exactly what it did; `PixhawkFC` holds one
and forwards to it.

That is deliberate. `DUBURI_WS_INTEGRATION.md` §3 suggests *moving*
`pixhawk.py` under `fc/`, but eleven modules import it -- nine of them only
for the two static helpers -- and the whole point of migration step 1 is
"existing missions still run identically on Pixhawk". Moving the file buys
nothing and breaks eleven imports; wrapping it buys the seam and breaks none.

`__getattr__` forwards anything the ABC does not name, so any Pixhawk method
this shim has not thought about still reaches the real object. Without that,
adding a method to `Pixhawk` would silently stop working through the HAL.
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional, Tuple

from .base import FlightController, Telemetry


class PixhawkFC(FlightController):

    name = 'pixhawk'

    # ArduSub implements RC override properly, including per-channel release
    # via NO_OVERRIDE. This is the backend the channel-based motion layer was
    # written against.
    supports_rc_override = True

    # ArduSub has no equivalent of SROT_MOVE. Motion primitives live in the
    # Python motion layer for this backend, which is why it is there at all.
    supports_native_move = False

    def __init__(self, pixhawk, log=None):
        self._px = pixhawk
        self._log = log

    @property
    def raw(self):
        """The underlying `Pixhawk`. For tests and for code mid-migration."""
        return self._px

    # ---- arming --------------------------------------------------------
    def arm(self, timeout: float = 15.0) -> Tuple[bool, str]:
        return self._px.arm(timeout=timeout)

    def disarm(self, timeout: float = 15.0) -> Tuple[bool, str]:
        return self._px.disarm(timeout=timeout)

    def is_armed(self) -> bool:
        return self._px.is_armed()

    # ---- modes ---------------------------------------------------------
    def set_mode(self, mode_name: str, timeout: float = 8.0) -> Tuple[bool, str]:
        return self._px.set_mode(mode_name, timeout=timeout)

    def get_mode(self) -> str:
        return self._px.get_mode()

    # ---- intent actuation ----------------------------------------------
    def manual(self, fwd: float = 0.0, lat: float = 0.0,
               up: float = 0.0, yaw: float = 0.0) -> None:
        """Normalised axes -> RC override.

        -1..1 maps to the same 1100..1900 window `percent_to_pwm` produces,
        so this is the intent API expressed in the backend's native verb
        rather than a second, subtly different scaling.
        """
        self._px.send_rc_override(
            throttle=self.percent_to_pwm(up * 100.0),
            yaw=self.percent_to_pwm(yaw * 100.0),
            forward=self.percent_to_pwm(fwd * 100.0),
            lateral=self.percent_to_pwm(lat * 100.0),
        )

    # move() intentionally not implemented -- see supports_native_move.

    # ---- legacy channel actuation --------------------------------------
    def send_rc_override(self, pitch: int = 1500, roll: int = 1500,
                         throttle: int = 1500, yaw: int = 1500,
                         forward: int = 1500, lateral: int = 1500) -> None:
        self._px.send_rc_override(pitch=pitch, roll=roll, throttle=throttle,
                                  yaw=yaw, forward=forward, lateral=lateral)

    def send_rc_translation(self, throttle: int = 1500,
                            forward: int = 1500, lateral: int = 1500) -> None:
        self._px.send_rc_translation(
            throttle=throttle, forward=forward, lateral=lateral)

    def send_neutral(self) -> None:
        self._px.send_neutral()

    def release_rc_override(self) -> None:
        self._px.release_rc_override()

    def set_target_depth(self, depth_m: float) -> None:
        self._px.set_target_depth(depth_m)

    # ---- misc ----------------------------------------------------------
    def set_servo_pwm(self, aux_n: int, pwm: int) -> None:
        self._px.set_servo_pwm(aux_n, pwm)

    def send_heartbeat(self) -> None:
        self._px.send_heartbeat()

    def set_message_rate(self, message_id: int, hz: float) -> None:
        self._px.set_message_rate(message_id, hz)

    # ---- telemetry -----------------------------------------------------
    def get_attitude(self) -> Optional[Dict[str, float]]:
        return self._px.get_attitude()

    def get_attitude_age(self) -> Optional[float]:
        return self._px.get_attitude_age()

    def get_battery(self) -> Optional[Dict[str, float]]:
        return self._px.get_battery()

    def get_rc_channels(self) -> Optional[List[int]]:
        return self._px.get_rc_channels()

    def get_statustext(self) -> Optional[str]:
        return self._px.get_statustext()

    def telemetry(self) -> Telemetry:
        att = self.get_attitude()
        bat = self.get_battery()
        return Telemetry(
            armed=self.is_armed(),
            mode=self.get_mode(),
            yaw_deg=att['yaw'] if att else None,
            roll_deg=att['roll'] if att else None,
            pitch_deg=att['pitch'] if att else None,
            depth_m=att['depth'] if att else None,
            battery_voltage=bat['voltage'] if bat else None,
            # ArduSub here reports one pack. thruster_voltage stays None
            # rather than duplicating it -- a consumer must be able to tell
            # "no second pack on this vehicle" from "second pack reads the
            # same as the first".
            thruster_voltage=None,
            rpm=[],
            attitude_age=self.get_attitude_age(),
        )

    # ---- anything not named above --------------------------------------
    def __getattr__(self, item: str) -> Any:
        # Only called for attributes not found normally, so it never shadows
        # the explicit methods above.
        return getattr(self._px, item)
