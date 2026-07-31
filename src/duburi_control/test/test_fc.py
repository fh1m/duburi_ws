"""Tests for the flight-controller HAL.

Two things are being pinned here:

1. **PixhawkFC changes nothing.** Migration step 1's checkpoint is "existing
   missions still run identically on Pixhawk". A shim that quietly reorders
   or drops a call would pass a smoke test and fail in the pool, so the
   delegation is asserted call-for-call.

2. **SrotFC speaks the right wire.** The MANUAL_CONTROL scaling and the
   SROT_MOVE parameter layout are the two places where an off-by-a-factor
   sends the vehicle somewhere unintended, and neither is observable without
   hardware. They are checked against the numbers in JETSON_COMMS.md.
"""

import math

import pytest

from duburi_control.fc import make_fc
from duburi_control.fc.base import FlightController, MoveResult
from duburi_control.fc.pixhawk import PixhawkFC


# --------------------------------------------------------------------------
# Fakes
# --------------------------------------------------------------------------

class FakePixhawk:
    """Records calls. Narrow on purpose -- if PixhawkFC reaches for a method
    that is not here, the test fails loudly rather than silently passing."""

    def __init__(self):
        self.calls = []
        self.armed = False
        self.mode = 'MANUAL'
        self.attitude = {'yaw': 90.0, 'roll': 1.0, 'pitch': -2.0, 'depth': -1.5}
        self.battery = {'voltage': 15.4, 'current': 2.0}

    def _rec(self, name, **kw):
        self.calls.append((name, kw))

    def arm(self, timeout=15.0):
        self._rec('arm', timeout=timeout)
        return True, 'ACCEPTED'

    def disarm(self, timeout=15.0):
        self._rec('disarm', timeout=timeout)
        return True, 'ACCEPTED'

    def is_armed(self):
        return self.armed

    def set_mode(self, mode_name, timeout=8.0):
        self._rec('set_mode', mode_name=mode_name, timeout=timeout)
        self.mode = mode_name
        return True, 'ACCEPTED'

    def get_mode(self):
        return self.mode

    def send_rc_override(self, pitch=1500, roll=1500, throttle=1500,
                         yaw=1500, forward=1500, lateral=1500):
        self._rec('send_rc_override', pitch=pitch, roll=roll, throttle=throttle,
                  yaw=yaw, forward=forward, lateral=lateral)

    def send_rc_translation(self, throttle=1500, forward=1500, lateral=1500):
        self._rec('send_rc_translation', throttle=throttle,
                  forward=forward, lateral=lateral)

    def send_neutral(self):
        self._rec('send_neutral')

    def release_rc_override(self):
        self._rec('release_rc_override')

    def set_target_depth(self, depth_m):
        self._rec('set_target_depth', depth_m=depth_m)

    def set_servo_pwm(self, aux_n, pwm):
        self._rec('set_servo_pwm', aux_n=aux_n, pwm=pwm)

    def send_heartbeat(self):
        self._rec('send_heartbeat')

    def set_message_rate(self, message_id, hz):
        self._rec('set_message_rate', message_id=message_id, hz=hz)

    def get_attitude(self):
        return self.attitude

    def get_attitude_age(self):
        return 0.01

    def get_battery(self):
        return self.battery

    def get_rc_channels(self):
        return [1500] * 8

    def get_statustext(self):
        return 'all good'

    def some_method_the_hal_never_heard_of(self):
        self._rec('passthrough')
        return 'reached'


class FakeMav:
    """Captures the MAVLink calls SrotFC makes."""

    def __init__(self):
        self.sent = []

    def command_long_send(self, sysid, compid, cmd, conf, *params):
        self.sent.append(('command_long', cmd, list(params)))

    def manual_control_send(self, sysid, x, y, z, r, buttons):
        self.sent.append(('manual_control', x, y, z, r, buttons))

    def set_mode_send(self, sysid, base_mode, custom_mode):
        self.sent.append(('set_mode', custom_mode))

    def heartbeat_send(self, *a):
        self.sent.append(('heartbeat',))

    def param_set_send(self, sysid, compid, name, value, ptype):
        self.sent.append(('param_set', name, value))


class FakeMaster:
    def __init__(self):
        self.mav = FakeMav()
        self.messages = {}
        self.target_system = 1
        self.target_component = 1


class Msg:
    """Minimal stand-in for a pymavlink message object."""

    def __init__(self, **kw):
        self.__dict__.update(kw)


# --------------------------------------------------------------------------
# Factory
# --------------------------------------------------------------------------

def test_factory_rejects_unknown_backend():
    with pytest.raises(ValueError) as exc:
        make_fc('ardupilot', master=FakeMaster())
    # The message must list what IS valid -- a bare "unknown" sends you
    # reading source at the dockside.
    assert 'pixhawk' in str(exc.value)
    assert 'srot' in str(exc.value)


def test_factory_rejects_missing_master():
    with pytest.raises(ValueError):
        make_fc('srot', master=None)


def test_factory_normalises_case_and_whitespace():
    fc = make_fc('  SROT ', master=FakeMaster())
    assert fc.name == 'srot'


# --------------------------------------------------------------------------
# PixhawkFC -- must be a transparent shim
# --------------------------------------------------------------------------

def test_pixhawk_fc_delegates_every_call():
    px = FakePixhawk()
    fc = PixhawkFC(px)

    fc.arm(timeout=9.0)
    fc.disarm(timeout=8.0)
    fc.set_mode('ALT_HOLD', timeout=4.0)
    fc.send_rc_override(throttle=1600, yaw=1400)
    fc.send_rc_translation(forward=1700)
    fc.send_neutral()
    fc.release_rc_override()
    fc.set_target_depth(-2.0)
    fc.set_servo_pwm(3, 1800)
    fc.send_heartbeat()
    fc.set_message_rate(30, 50)

    names = [c[0] for c in px.calls]
    assert names == [
        'arm', 'disarm', 'set_mode', 'send_rc_override', 'send_rc_translation',
        'send_neutral', 'release_rc_override', 'set_target_depth',
        'set_servo_pwm', 'send_heartbeat', 'set_message_rate',
    ]
    # Arguments must survive intact, not just the call order.
    assert px.calls[0][1] == {'timeout': 9.0}
    assert px.calls[2][1] == {'mode_name': 'ALT_HOLD', 'timeout': 4.0}
    assert px.calls[3][1]['throttle'] == 1600
    assert px.calls[3][1]['yaw'] == 1400
    assert px.calls[7][1] == {'depth_m': -2.0}


def test_pixhawk_fc_passes_through_unknown_methods():
    # Without __getattr__, adding a method to Pixhawk would silently stop
    # working through the HAL.
    fc = PixhawkFC(FakePixhawk())
    assert fc.some_method_the_hal_never_heard_of() == 'reached'


def test_pixhawk_fc_telemetry_snapshot():
    px = FakePixhawk()
    px.armed = True
    tel = PixhawkFC(px).telemetry()
    assert tel.armed is True
    assert tel.yaw_deg == 90.0
    assert tel.depth_m == -1.5
    assert tel.battery_voltage == 15.4
    # ArduSub has one pack here. None means "no second pack", which must be
    # distinguishable from "second pack reads the same as the first".
    assert tel.thruster_voltage is None
    assert tel.rpm == []


def test_pixhawk_fc_capability_flags():
    fc = PixhawkFC(FakePixhawk())
    assert fc.supports_rc_override is True
    assert fc.supports_native_move is False
    with pytest.raises(NotImplementedError):
        fc.move('forward')


# --------------------------------------------------------------------------
# SrotFC -- actuation scaling
# --------------------------------------------------------------------------

def _srot():
    master = FakeMaster()
    return make_fc('srot', master=master), master


def test_manual_control_scaling():
    fc, m = _srot()
    fc.manual(fwd=1.0, lat=-1.0, up=1.0, yaw=0.5)
    kind, x, y, z, r, _ = m.mav.sent[-1]
    assert kind == 'manual_control'
    assert (x, y, r) == (1000, -1000, 500)   # +/-1000
    assert z == 1000                          # 0..1000, full ascend


def test_manual_control_neutral_is_500_on_z_only():
    fc, m = _srot()
    fc.manual()
    _, x, y, z, r, _ = m.mav.sent[-1]
    assert (x, y, r) == (0, 0, 0)
    # z is the odd axis out: 0..1000 with 500 neutral, not +/-1000. Sending 0
    # here would be full descent, not "stop".
    assert z == 500


def test_manual_control_clamps_out_of_range():
    fc, m = _srot()
    fc.manual(fwd=5.0, up=-5.0)
    _, x, _, z, _, _ = m.mav.sent[-1]
    assert x == 1000
    assert z == 0


def test_rc_override_is_emulated_as_manual_control():
    fc, m = _srot()
    fc.send_rc_override(forward=1900, lateral=1100, throttle=1500, yaw=1500)
    kind, x, y, z, r, _ = m.mav.sent[-1]
    assert kind == 'manual_control'
    assert x == 1000     # 1900 pwm -> +100 % -> +1000
    assert y == -1000    # 1100 pwm -> -100 %
    assert z == 500      # 1500 pwm -> neutral heave
    assert r == 0


def test_released_channel_becomes_neutral_not_full_scale():
    # 65535 is NO_OVERRIDE. Read as a raw PWM it would clamp to 1900 --
    # full authority on that axis. Getting this wrong is a runaway.
    fc, m = _srot()
    fc.send_rc_override(forward=65535, throttle=65535)
    _, x, _, z, _, _ = m.mav.sent[-1]
    assert x == 0
    assert z == 500


def test_send_neutral_stops_every_axis():
    fc, m = _srot()
    fc.send_neutral()
    _, x, y, z, r, _ = m.mav.sent[-1]
    assert (x, y, r) == (0, 0, 0)
    assert z == 500


# --------------------------------------------------------------------------
# SrotFC -- SROT_MOVE
# --------------------------------------------------------------------------

def test_move_wire_parameters():
    fc, m = _srot()
    fc.move('turn', primary=90.0, speed=45.0, mode=1.0, timeout=30.0)
    kind, cmd, params = m.mav.sent[-1]
    assert kind == 'command_long'
    assert cmd == 31000
    # p1 is the WIRE type (turn = 4), not movement::Type (which is 5).
    assert params[0] == 4.0
    assert params[1:5] == [90.0, 45.0, 1.0, 30.0]
    assert params[5:] == [0.0, 0.0]


def test_move_verb_codes_match_the_wire_table():
    fc, m = _srot()
    expected = {'forward': 0, 'back': 1, 'left': 2, 'right': 3, 'turn': 4,
                'dive': 5, 'stop': 6, 'hold': 7, 'style': 8, 'arc': 9}
    for verb, code in expected.items():
        fc.move(verb)
        assert m.mav.sent[-1][2][0] == float(code), verb


def test_move_rejects_unknown_verb():
    fc, _ = _srot()
    with pytest.raises(ValueError):
        fc.move('backflip')


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), -float('inf')])
def test_move_rejects_non_finite_before_the_wire(bad):
    # The board denies these too, but failing here names the field.
    fc, m = _srot()
    before = len(m.mav.sent)
    with pytest.raises(ValueError):
        fc.move('forward', primary=bad)
    assert len(m.mav.sent) == before


def test_set_target_depth_sends_a_dive_with_positive_metres():
    # Callers pass ArduSub's convention (negative below surface); the dive
    # primitive wants a positive depth.
    fc, m = _srot()
    fc.set_target_depth(-2.5)
    _, cmd, params = m.mav.sent[-1]
    assert cmd == 31000
    assert params[0] == 5.0      # dive
    assert params[1] == 2.5


# --------------------------------------------------------------------------
# SrotFC -- the ACK stream. This is what stops an action hanging.
# --------------------------------------------------------------------------

def _ack(master, result, progress=0):
    master.messages['COMMAND_ACK'] = Msg(
        command=31000, result=result, progress=progress)


def test_move_reports_progress_then_succeeds():
    fc, m = _srot()
    handle = fc.move('forward', primary=3.0, speed=0.5)

    _ack(m, 5, 40)                      # IN_PROGRESS
    assert handle.update() == 40
    assert not handle.done

    _ack(m, 0, 100)                     # ACCEPTED
    handle.update()
    assert handle.done
    assert handle.result.success is True
    assert handle.result.result == 'ACCEPTED'


@pytest.mark.parametrize('code,name', [(6, 'CANCELLED'), (4, 'FAILED'), (2, 'DENIED')])
def test_every_terminal_result_resolves_the_move(code, name):
    # The whole reason this class exists: a client waiting only for ACCEPTED
    # hangs on preemption (CANCELLED) and on an unstartable move (FAILED).
    fc, m = _srot()
    handle = fc.move('forward', primary=1.0)
    _ack(m, code)
    handle.update()
    assert handle.done
    assert handle.result.result == name
    assert handle.result.success is False


def test_cancelled_is_flagged_as_preemption():
    fc, m = _srot()
    handle = fc.move('forward', primary=1.0)
    _ack(m, 6)
    handle.update()
    # Preemption is documented behaviour, not a fault -- callers map it to
    # abort rather than error, and need to be able to tell.
    assert handle.result.preempted is True


def test_temporarily_rejected_is_not_terminal():
    # The board returns this when it could not take its control mutex.
    # Resolving on it would abort a move that never started.
    fc, m = _srot()
    handle = fc.move('forward', primary=1.0)
    _ack(m, 1)                          # TEMP_REJECTED
    handle.update()
    assert not handle.done
    assert handle.temporarily_rejected is True


def test_stale_ack_from_a_previous_move_cannot_resolve_the_next_one():
    fc, m = _srot()
    first = fc.move('forward', primary=1.0)
    _ack(m, 0, 100)
    first.update()
    assert first.done

    second = fc.move('turn', primary=90.0)
    # move() clears the cache, so the previous ACCEPTED must not leak in.
    assert second.update() == 0
    assert not second.done


def test_move_times_out_rather_than_hanging_for_ever():
    # A vehicle that stops acking -- link loss, reboot, a safety abort that
    # disarmed mid-stream -- must still resolve the action.
    fc, _ = _srot()
    handle = fc.move('forward', primary=1.0, timeout=0.0)
    handle._deadline = 0.0              # force expiry
    handle.update()
    assert handle.done
    assert handle.result.result == 'FAILED'
    assert 'terminal' in handle.result.message


# --------------------------------------------------------------------------
# SrotFC -- modes and telemetry
# --------------------------------------------------------------------------

def test_ardusub_mode_names_are_translated():
    fc, m = _srot()
    # duburi.py still says ALT_HOLD in YAW_OK_MODES and _ensure_alt_hold().
    fc.set_mode('ALT_HOLD', timeout=0.05)
    assert m.mav.sent[-1] == ('set_mode', 2)     # DEPTH_HOLD


def test_unknown_mode_is_rejected_by_name():
    fc, _ = _srot()
    ok, reason = fc.set_mode('FLIP', timeout=0.05)
    assert ok is False
    assert 'FLIP' in reason


def test_get_mode_covers_every_enum_value():
    fc, m = _srot()
    for custom, name in [(0, 'STABILIZE'), (2, 'DEPTH_HOLD'), (19, 'MANUAL'),
                         (20, 'MOTOR_DETECT'), (21, 'AUTOTUNE'),
                         (22, 'MOTOR_TUNE'), (23, 'AUTO'),
                         (100, 'STUNT'), (101, 'PATTERN')]:
        m.messages['HEARTBEAT'] = Msg(autopilot=0, base_mode=0, custom_mode=custom)
        assert fc.get_mode() == name


def test_attitude_converts_radians_to_degrees():
    fc, m = _srot()
    m.messages['ATTITUDE'] = Msg(
        yaw=math.radians(90.0), roll=math.radians(10.0),
        pitch=math.radians(-5.0), _timestamp=None)
    m.messages['VFR_HUD'] = Msg(alt=-2.0)
    att = fc.get_attitude()
    assert att['yaw'] == pytest.approx(90.0)
    assert att['roll'] == pytest.approx(10.0)
    assert att['pitch'] == pytest.approx(-5.0)
    # VFR_HUD.alt is already -depth, matching AHRS2.altitude on ArduSub, so
    # it passes through unchanged. A sign flip here would invert every depth
    # comparison in the motion layer.
    assert att['depth'] == -2.0


def test_yaw_is_wrapped_into_0_360():
    fc, m = _srot()
    m.messages['ATTITUDE'] = Msg(yaw=math.radians(-90.0), roll=0.0,
                                 pitch=0.0, _timestamp=None)
    assert fc.get_attitude()['yaw'] == pytest.approx(270.0)


def test_attitude_is_none_before_any_sample():
    fc, _ = _srot()
    assert fc.get_attitude() is None


def test_thruster_pack_is_absent_not_zero_when_the_link_is_down():
    fc, m = _srot()
    m.messages['HEARTBEAT'] = Msg(autopilot=0, base_mode=0, custom_mode=0)
    tel = fc.telemetry()
    # None means no ESP-NOW data. Reporting 0.0 would look like a flat pack
    # and could trip a supervisor's low-battery logic.
    assert tel.thruster_voltage is None


def test_thruster_pack_reads_battery_id_1():
    fc, m = _srot()
    m.messages['BATTERY_STATUS'] = Msg(id=1, voltages=[16200] + [65535] * 9,
                                       current_battery=-1)
    assert fc.telemetry().thruster_voltage == pytest.approx(16.2)
    # id 1 is not the electronics pack -- get_battery() must not return it.
    assert fc.get_battery() is None


def test_esc_status_rpm_lands_at_the_right_indices():
    fc, m = _srot()
    m.messages['ESC_STATUS'] = Msg(index=4, rpm=[100, 200, 300, 400])
    assert fc.get_rpm() == [0, 0, 0, 0, 100, 200, 300, 400]


def test_helpers_match_the_pixhawk_originals():
    # Nine modules import Pixhawk purely for these. They are on the ABC so
    # new code does not have to, which only works if they agree exactly.
    from duburi_control.pixhawk import Pixhawk
    for pct in (-100, -37.5, 0, 12.3, 100):
        assert FlightController.percent_to_pwm(pct) == Pixhawk.percent_to_pwm(pct)
    for target, current in ((10, 350), (350, 10), (0, 180), (0, 181)):
        assert (FlightController.heading_error(target, current)
                == Pixhawk.heading_error(target, current))


def test_move_result_success_only_for_accepted():
    assert MoveResult(result='ACCEPTED', success=True).success is True
    assert MoveResult(result='CANCELLED').success is False
