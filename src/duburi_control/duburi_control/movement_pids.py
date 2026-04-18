#!/usr/bin/env python3
"""
PID controllers for depth and yaw axes.
No ROS2 dependencies — pure math, importable anywhere.
"""

from .mavlink_api import MavlinkAPI


class PIDController:
    """
    Discrete PID with:
    - Anti-windup (back-calculation / clamp)
    - Derivative on measurement (no kick on setpoint change)
    - Low-pass derivative filter
    - Output deadband (skip tiny corrections)
    - Minimum-kick (boost small non-zero outputs past actuator deadzones)
    """

    def __init__(self, kp: float, ki: float, kd: float,
                 out_min: float, out_max: float,
                 integral_max: float = 300.0,
                 alpha: float = 0.2,
                 deadband: float = 0.0,
                 min_kick: float = 0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self.integral_max = integral_max
        self.alpha = alpha          # derivative low-pass (0=no filter, 1=freeze)
        self.deadband = deadband
        self.min_kick = min_kick    # |output| < min_kick is boosted to ±min_kick

        self._integral = 0.0
        self._prev_meas = None
        self._prev_deriv = 0.0

    def reset(self):
        self._integral = 0.0
        self._prev_meas = None
        self._prev_deriv = 0.0

    def _error(self, setpoint: float, measurement: float) -> float:
        return setpoint - measurement

    def _meas_delta(self, current: float, previous: float) -> float:
        """Change in measurement. Override for wrapped-angle signals."""
        return current - previous

    def compute(self, setpoint: float, measurement: float, dt: float) -> float:
        if dt <= 0:
            return 0.0

        error = self._error(setpoint, measurement)

        if abs(error) <= self.deadband:
            return 0.0

        # Derivative on measurement (not error) — no kick on setpoint change
        if self._prev_meas is None:
            raw_deriv = 0.0
        else:
            raw_deriv = -self._meas_delta(measurement, self._prev_meas) / dt
        self._prev_meas = measurement

        # Low-pass filter on derivative
        deriv = self.alpha * self._prev_deriv + (1.0 - self.alpha) * raw_deriv
        self._prev_deriv = deriv

        # Proportional + integral + derivative
        p_term = self.kp * error
        d_term = self.kd * deriv

        # Integrate with anti-windup clamp
        new_integral = self._integral + self.ki * error * dt
        new_integral = max(-self.integral_max, min(self.integral_max, new_integral))

        output = p_term + new_integral + d_term
        output_clamped = max(self.out_min, min(self.out_max, output))

        # Back-calculation anti-windup: don't grow integral when saturated
        if output != output_clamped:
            # Saturation: keep integral only if it would reduce saturation
            if not (output > self.out_max and error > 0) and \
               not (output < self.out_min and error < 0):
                self._integral = new_integral
        else:
            self._integral = new_integral

        # Minimum-kick: boost small non-zero commands past actuator deadzones
        # (e.g. ArduSub RCn_DZ default = 30 µs). Sign of error is ground truth.
        if self.min_kick > 0.0 and 0.0 < abs(output_clamped) < self.min_kick:
            output_clamped = self.min_kick if error > 0 else -self.min_kick

        return output_clamped


class DepthPID(PIDController):
    """
    Depth axis. Output is PWM delta from 1500 for throttle channel (Ch3).
    Positive output → rise, negative → dive (ArduSub ALT_HOLD convention).

    Gains are expressed in PWM-µs per metre of depth error. With the sub in
    ALT_HOLD mode, Ch3 deflection from 1500 commands a climb/dive *rate* —
    so a strong proportional term is needed to actually move the vehicle.
    """

    def __init__(self):
        super().__init__(
            kp=180.0, ki=6.0, kd=40.0,
            out_min=-300, out_max=300,
            integral_max=80.0,
            alpha=0.2,
            deadband=0.08,     # 8 cm — don't correct tiny errors
            min_kick=40.0,     # escape ArduSub RC3_DZ (default 30 µs)
        )

    def compute_pwm(self, target_m: float, current_m: float, dt: float) -> int:
        """Returns absolute PWM value for Ch3 (throttle)."""
        delta = self.compute(target_m, current_m, dt)
        return int(1500 + delta)


class YawPID(PIDController):
    """
    LEGACY — retained for reference / fallback only.

    The live `MovementCommands._run_yaw` no longer uses this; yaw is driven
    by SET_ATTITUDE_TARGET which delegates to ArduSub's internal 400 Hz
    attitude stabiliser. See `movement_commands.py` docstring for rationale.

    A 20 Hz Python PID on RC override cannot match ArduSub's 400 Hz loop
    because:
      • AHRS yaw noise (~0.5°) amplifies through kd by 20× per second,
        producing the jerky ±150 µs PWM swings visible in earlier logs.
      • ArduSub's RCn_DZ (30 µs) silently swallows small P-term commands,
        stalling the last few degrees of every turn.
      • ArduSub's native controller has direct access to gyro rates, not
        differentiated compass heading, so its damping is clean.

    Kept here because it's still a useful reference for the depth tune
    and because future mission logic may want a PID-based heading-hold
    slot alongside the translation channels.
    """

    def __init__(self):
        super().__init__(
            kp=3.5, ki=0.0, kd=8.0,
            out_min=-400, out_max=400,
            integral_max=0.0,
            alpha=0.25,
            deadband=2.5,
            min_kick=40.0,
        )

    def _error(self, setpoint: float, measurement: float) -> float:
        return MavlinkAPI.heading_error(setpoint, measurement)

    def _meas_delta(self, current: float, previous: float) -> float:
        # Shortest signed angular change from previous → current.
        # Avoids the ±358° spike when heading wraps across 0/360°.
        return MavlinkAPI.heading_error(current, previous)

    def compute_pwm(self, target_deg: float, current_deg: float, dt: float) -> int:
        """Returns absolute PWM value for Ch4 (yaw)."""
        delta = self.compute(target_deg, current_deg, dt)
        return int(1500 + delta)
