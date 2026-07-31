"""Single dispatch point: name -> FlightController instance.

Used by `auv_manager_node` at startup. Adding a backend = one line in
`BUILDERS` plus the class itself; nothing else changes.

Mirrors `duburi_sensors/factory.py` deliberately -- same keyword-only
builders, same no-silent-fallback policy, same normalise-then-look-up. One
pattern for both HALs is easier to hold in your head than two.
"""

from __future__ import annotations


def _build_pixhawk(*, master, log=None, **_):
    if master is None:
        raise ValueError("flight_controller='pixhawk' requires master=<mavlink_connection>")
    from ..pixhawk import Pixhawk
    from .pixhawk import PixhawkFC
    return PixhawkFC(Pixhawk(master, log=log), log=log)


def _build_srot(*, master, log=None, target_system=1, target_component=1, **_):
    if master is None:
        raise ValueError("flight_controller='srot' requires master=<mavlink_connection>")
    from .srot import SrotFC
    return SrotFC(master, log=log,
                  target_system=target_system,
                  target_component=target_component)


BUILDERS = {
    'pixhawk': _build_pixhawk,
    'srot':    _build_srot,
}


def make_fc(name, **kwargs):
    """Return a configured FlightController for the given name.

    Raises
    ------
    ValueError
        If `name` is unknown or required kwargs are missing.

    No silent fallback: an unknown backend fails loudly at startup rather
    than quietly flying the wrong one. Getting this wrong actuates
    thrusters, so a typo must not be survivable.
    """
    key = (name or '').strip().lower()
    if key not in BUILDERS:
        known = ', '.join(sorted(BUILDERS))
        raise ValueError(f"unknown flight_controller '{name}'. known: {known}")
    return BUILDERS[key](**kwargs)
