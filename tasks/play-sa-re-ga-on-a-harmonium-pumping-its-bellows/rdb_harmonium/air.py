"""The harmonium's air, a stated rule (a fluid response the rigid-body simulator does not have). NumPy only.

The bellows board at the back is a real hinged body that the robot pushes shut against a real torque (the
reservoir's working pressure times the board's area, plus its reopening spring; see layout.py). What this module
adds is where that air goes:

- every radian the board closes pushes FEEDER_M3_PER_RAD of air into the reservoir inside the case (the board's
  swept volume); opening it draws fresh air in through its intake valve and adds nothing;
- the reservoir holds at most RESERVOIR_M3 (a relief valve lets the rest out) and its spring keeps the pressure
  between EMPTY_PA and FULL_PA; in its last SAG_M3 it settles onto its stops and the pressure falls to zero;
- each reed whose key is down at least KEY_SPEAKS, with pressure above SPEAK_PA, sounds and draws air by the
  orifice law (flow proportional to the square root of the pressure, REED_FLOW_M3S at REED_FLOW_PA); the valves
  leak LEAK_M3S;

so one held key empties a full reservoir in about three seconds, and a note lasts only while the other hand pumps.
"""

from dataclasses import dataclass, field

import numpy as np

from . import layout as L


def pressure(volume):
    """Reservoir pressure (Pa) holding ``volume`` m^3 of air."""
    if volume <= 0.:
        return 0.
    if volume < L.SAG_M3:                    # nearly empty, the reservoir settles onto its stops and the pressure falls
        return L.EMPTY_PA * volume / L.SAG_M3
    return L.EMPTY_PA + (L.FULL_PA - L.EMPTY_PA) * min(volume, L.RESERVOIR_M3) / L.RESERVOIR_M3


def reed_flow(p):
    return L.REED_FLOW_M3S * np.sqrt(max(p, 0.) / L.REED_FLOW_PA)


@dataclass
class Air:
    volume: float = 0.                       # m^3 in the reservoir (starts empty)
    flap: float | None = None                # last board angle seen (rad, 0 = shut)
    fed_m3: float = 0.                       # total pushed in by the board
    vented_m3: float = 0.                    # let out by the relief valve
    sounding: tuple = ()                     # names of the keys sounding now
    strokes: int = 0                         # closing strokes counted (diagnostics)
    _closing: bool = field(default=False, repr=False)

    def step(self, dt, flap_angle, key_depths):
        """One control step of ``dt`` s. ``flap_angle``: board angle now (rad); ``key_depths``: {key name: how far
        down, m}. Returns (pressure in Pa, names of sounding keys)."""
        flap = float(flap_angle)
        if self.flap is not None:
            shut_by = self.flap - flap
            if shut_by > 0.:
                pushed = L.FEEDER_M3_PER_RAD * shut_by
                self.fed_m3 += pushed
                self.volume += pushed
                if not self._closing and shut_by > .002:
                    self.strokes += 1
                    self._closing = True
            elif shut_by < -.002:
                self._closing = False
        self.flap = flap
        if self.volume > L.RESERVOIR_M3:
            self.vented_m3 += self.volume - L.RESERVOIR_M3
            self.volume = L.RESERVOIR_M3
        p = pressure(self.volume)
        open_keys = tuple(sorted(name for name, depth in key_depths.items() if depth >= L.KEY_SPEAKS))
        sounding = open_keys if p >= L.SPEAK_PA else ()
        used = (len(sounding) * reed_flow(p) + (L.LEAK_M3S if self.volume > 0. else 0.)) * dt
        self.volume = max(0., self.volume - used)
        self.sounding = sounding
        return p, sounding

    def seconds_left(self, reeds=1):
        """How long the air now in the reservoir would keep ``reeds`` reeds sounding (diagnostics)."""
        volume, t, dt = self.volume, 0., .01
        while volume > 0. and t < 60.:
            volume -= (reeds * reed_flow(pressure(volume)) + L.LEAK_M3S) * dt
            t += dt
        return t
