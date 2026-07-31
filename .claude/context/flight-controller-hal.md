# Flight-controller HAL (`duburi_control/fc/`)

**Status:** migration steps 1–5 built and unit-tested. Nothing has flown on SROT.
`flight_controller` defaults to `pixhawk`; the ArduSub path is untouched.

## Why it exists

The stack grew around one autopilot reached through one object
(`duburi_control/pixhawk.py`), constructed directly and passed duck-typed into every
`motion_*.py` function. That was fine with one backend. There are now two:

| | Pixhawk / ArduSub | SROT board (Hengla firmware) |
|---|---|---|
| Actuation | `RC_CHANNELS_OVERRIDE`, 1100–1900 µs | `MANUAL_CONTROL`, x/y/r ±1000, z 0–1000 (500 neutral) |
| Motion primitives | Python (`motion_*.py`) | on-board, `MAV_CMD_SROT_MOVE` (31000) |
| Depth hold | ArduSub ALT_HOLD + streamed setpoint | on-board, `dive` primitive |
| Heading hold | Python `heading_lock.py` | on-board, per translate leg |
| Modes | ArduSub numbers | different numbers, 5 extra modes |
| Attitude | AHRS2 (mag fused) | ATTITUDE, BNO085 **game** vector — mag never fused |

Both speak MAVLink 2 at sysid 1 / compid 1. It is a verb swap, not a rewrite.

## Layout

```
duburi_control/fc/
  base.py      FlightController ABC + Telemetry, MoveHandle, MoveResult
  pixhawk.py   PixhawkFC   — delegation shim over the existing Pixhawk
  srot.py      SrotFC      — new backend
  factory.py   make_fc(name, master=..., log=...)
```

Selected by the `flight_controller` ROS param on `duburi_manager`:

```bash
ros2 run duburi_manager auv_manager_node --ros-args -p flight_controller:=srot
```

`self.pixhawk` in the manager keeps its name deliberately — it is passed duck-typed
into a dozen places and both backends expose the same surface. The seam is the
object, not the label.

## Two decisions worth knowing

**`pixhawk.py` was NOT moved under `fc/`.** The integration guide suggests moving it.
Eleven modules import it — nine purely for the two static helpers — and step 1's
checkpoint is "existing missions still run identically on Pixhawk". Moving the file
breaks eleven imports and buys nothing; `PixhawkFC` wraps it instead, with
`__getattr__` forwarding anything the ABC does not name.

**The ABC keeps the legacy channel API.** `send_rc_override`, `send_rc_translation`,
`send_neutral`, `release_rc_override`, `set_target_depth` are all still there, because
the alternative is rewriting the whole motion layer in one flag-day commit. `SrotFC`
implements them by translating to `MANUAL_CONTROL`, so existing motion code runs on
either backend unmodified.

## The trap in that translation

`NO_OVERRIDE` (65535) means "release this axis to the autopilot". **`MANUAL_CONTROL`
has no equivalent** — it always carries all four axes. `SrotFC` holds a released axis
at *neutral* instead. That is the safe reading, but it is not the same thing: under RC
override the autopilot took the axis over; here nobody does.

Two places depend on real release semantics and are therefore **not yet correct on
SROT**:

- `motion_depth.py` hands Ch3 to ALT_HOLD to run its depth ramp.
- `heading_lock.py` releases Ch4 on stop.

Both are migration step 6 (delete the redundant Python loops, since SROT does depth and
heading on-board). Until then, `supports_rc_override = False` is the flag to check, and
the manager logs a warning at startup on any backend where it is false.

## `move()` and the four terminal results

`SrotFC.move(verb, ...)` returns a `MoveHandle`; poll `update()` on your feedback tick.
It resolves to exactly one of **ACCEPTED / CANCELLED / FAILED / DENIED**.

An action client that waits only for ACCEPTED **hangs**: preemption resolves CANCELLED
and an unstartable move resolves FAILED. `MoveResult.preempted` distinguishes the
expected case — a new move displacing the running one is documented behaviour, so map
it to *abort*, not *error*.

`MAV_RESULT_TEMPORARILY_REJECTED` is a **fifth, non-terminal** reply (the board could
not take its control mutex). It sets `handle.temporarily_rejected` and does **not**
resolve the move. Retry it.

There is also a watchdog: if no terminal ACK arrives before the board's safety timeout
plus slack, the handle resolves FAILED rather than pending for ever. Link loss, a
reboot, or a safety abort that disarmed mid-stream all land here.

## Still to do (migration step 6+)

- Delete `heading_lock.py`, `motion_easing.py` and the ALT_HOLD dance **for the SROT
  path only** — they must keep working while `pixhawk` is the default.
- Route `/duburi/move` verbs to `fc.move()` when `supports_native_move`, instead of
  through the Python motion layer.
- `DuburiState.mode` is typed as an ArduSub mode-name string; SROT mode names differ.
  `SrotFC` translates ALT_HOLD/POSHOLD/GUIDED on the way *in*, but `get_mode()` returns
  SROT names on the way out.
- Vision verbs need only `manual()`, which is done — they should port last and cheaply.

## Running the tests off-ROS

`test/conftest.py` stubs `duburi_interfaces` when the real colcon-generated package is
absent, so the pure-Python suite runs on a bare machine:

```bash
cd src/duburi_control && python -m pytest test/ -q
```

It is a collection convenience only — anything touching real message serialisation,
rclpy, or the action machinery still needs a built, sourced workspace.
