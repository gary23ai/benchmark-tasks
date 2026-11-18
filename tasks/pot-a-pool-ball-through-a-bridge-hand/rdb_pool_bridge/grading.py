"""Score the shot: a legal bridged stroke, the 1-ball hit first, into the marked pocket, no foul.

Everything here can be judged from a video of a real trial (a side view and the overhead view) and a tape
measure: where the hands and the cue were when the tip met the white ball, which ball the white touched first,
where the 1-ball went. NumPy only, so the rules can be unit-tested without the simulator.

The environment calls ``update(sample, events)`` once per 20 Hz control step. ``sample`` is the state after
the step (balls, cue, the two grippers from the joint readings); ``events`` lists what the contact sensors saw
during the step's 12 physics steps (see ``EMPTY_EVENTS``).

Rules (README and task.json "scoring" say the same):
- Strike: the first physics step in which the cue's tip touches the white ball.
- Legal strike: at that moment (1) the left gripper's fingertips are on the cloth (at most 12 mm above it);
  (2) the cue's axis passes through the left hand's loop: between its two fingers, above its fingertips, with
  the pads closer together than the shaft is thick (the shaft rests in the hand, it is not pinched); (3) in the
  0.5 s before, the left hand stayed put (moved at most 15 mm) while the cue slid at least 40 mm through it;
  (4) the right gripper holds the cue: closed on it, its fingertip centre within 20 mm of the axis, and the cue
  moved with it (at most 12 mm of slip) over the same 0.5 s.
- Fouls (each a latched 0): a gripper or arm touches any ball; any part of the cue except its tip touches a
  ball, or the tip touches the 1-ball or the 3-ball; the tip touches the white again after it has left it
  (double hit); a ball moves more than 5 mm before the strike; the white ball is pocketed or leaves the table;
  any ball leaves the table.
- Tiers (highest reached, with no foul): 0.1 legal strike; 0.4 and the white's first ball contact is the 1-ball;
  0.7 and the 1-ball's centre crosses the marked pocket's mouth (the line between its jaw points); 1.0 and the
  1-ball drops into the marked pocket while the white stays on the table. An illegal strike scores 0.
- The trial ends once, after the strike, every ball on the table has been still (under 5 mm/s) for 1 s.
"""

from dataclasses import dataclass, field

import numpy as np

from . import layout as L

CONTROL_HZ = 20
WINDOW_S = .5
TIPS_ON_CLOTH = .012          # left fingertips at most this far above the cloth
LOOP_HALF_WIDTH = .012        # the axis between the fingers: within this of the fingers' midplane
LOOP_ABOVE_TIPS = (.013, .085)   # and this far above the fingertips (TCP frame: z from -0.072 to 0)
LOOP_ALONG = .035             # and within this of the fingers' centre along the pad width
BRIDGE_STILL = .015           # the left hand moved at most this (horizontally) in the window
SLIDE_MIN = .040              # the cue slid at least this through the bridge in the window
GRIP_ON_AXIS = .020           # right fingertip centre within this of the cue's axis (the TCP sits 9 mm off
                              # the axis in a pinch; run 2 read 11.8 mm)
GRIP_SLIP = .012              # cue's slip in the right hand over the window, at most
DISTURB = .005                # a ball moved this far before the strike: foul
SEPARATED = .005              # the tip has left the white once the gap exceeds this
STILL_SPEED = .005
STILL_S = 1.0
TIERS = {"legal": .1, "first_one": .4, "jaws": .7, "potted": 1.}
BRIDGE_TOUCH = .5            # the shaft touched the left hand in at least half of the window's control steps
EMPTY_EVENTS = {"tip_cue_ball": 0, "left_on_cue": 0, "impulse_ns": 0., "peak_tip_force_n": 0., "cue_side_balls": (), "tip_object_balls": (),
                "robot_balls": (), "cue_ball_hits": ()}


def tip_frame_point(point, tcp, rotation):
    """A world point in a gripper's TCP frame (x: closing axis, y: pad width, z: along the fingers)."""
    return np.asarray(rotation, dtype=float).T @ (np.asarray(point, dtype=float) - np.asarray(tcp, dtype=float))


def axis_near(line_point, line_dir, tcp, rotation):
    """Point of the cue's axis line closest to the gripper's finger line (through the TCP along the fingers)."""
    p, d = np.asarray(line_point, float), L.unit(line_dir)
    q, a = np.asarray(tcp, float), np.asarray(rotation, float)[:, 2]
    w = p - q
    b = d @ a
    denom = 1. - b * b
    t = (b * (w @ a) - (w @ d)) / denom if denom > 1e-9 else -(w @ d)
    return p + t * d


def distance_to_line(point, line_point, line_dir):
    d = L.unit(line_dir)
    w = np.asarray(point, float) - np.asarray(line_point, float)
    return float(np.linalg.norm(w - (w @ d) * d))


def axial(point, line_point, line_dir):
    """Coordinate of a point along the cue's axis, measured from the tip toward the butt (positive)."""
    return float(-(np.asarray(point, float) - np.asarray(line_point, float)) @ L.unit(line_dir))


def bridge_check(sample):
    """The geometric part of the bridge rule at one instant (rules 1 and 2)."""
    tip, axis = sample["cue_tip"], sample["cue_axis"]
    tcp, rot = sample["tcp"]["left"]
    fingertips = np.asarray(tcp) + L.PAD_TIP_PAST_TCP * np.asarray(rot)[:, 2]
    on_cloth = bool(fingertips[2] <= L.BED_Z + TIPS_ON_CLOTH)
    near = axis_near(tip, axis, tcp, rot)
    local = tip_frame_point(near, tcp, rot)
    s = axial(near, tip, axis)
    shaft = L.cue_diameter(s)
    above = L.PAD_TIP_PAST_TCP - local[2]                  # height of the axis above the fingertips
    in_loop = bool(abs(local[0]) <= LOOP_HALF_WIDTH and abs(local[1]) <= LOOP_ALONG
                   and LOOP_ABOVE_TIPS[0] <= above <= LOOP_ABOVE_TIPS[1] and 0. <= s <= L.CUE_L)
    resting = bool(sample["opening"]["left"] < shaft)      # pads closer than the shaft is thick: not pinched
    return {"on_cloth": on_cloth, "in_loop": in_loop, "resting": resting, "fingertip_z_mm": round(1000 * float(fingertips[2]), 1),
            "loop_x_mm": round(1000 * float(local[0]), 1), "axis_above_tips_mm": round(1000 * float(above), 1),
            "bridge_s_m": round(s, 4)}


def grip_check(sample):
    tip, axis = sample["cue_tip"], sample["cue_axis"]
    tcp, _ = sample["tcp"]["right"]
    s = axial(tcp, tip, axis)
    off = distance_to_line(tcp, tip, axis)
    width = sample["opening"]["right"]
    closed = bool(.006 <= width <= L.cue_diameter(s) + .002)
    return {"on_axis": bool(off <= GRIP_ON_AXIS), "closed": closed, "grip_s_m": round(s, 4),
            "grip_off_axis_mm": round(1000 * off, 1), "opening_mm": round(1000 * width, 1)}


@dataclass
class PoolGrade:
    tick: int = 0
    history: list = field(default_factory=list)
    strike_tick: int | None = None
    strike: dict | None = None
    legal: bool = False
    fouls: list = field(default_factory=list)
    first_hit: str | None = None
    separated: bool = False
    jaws: bool = False
    potted: dict = field(default_factory=dict)
    off: set = field(default_factory=set)
    still_ticks: int = 0
    finished: bool = False
    impulse_ns: float = 0.
    peak_tip_force_n: float = 0.
    peak_speed: dict = field(default_factory=dict)
    starts: dict | None = None
    touch_history: list = field(default_factory=list)

    def foul(self, text):
        if text not in self.fouls:
            self.fouls.append(text)

    def update(self, sample, events=None):
        events = {**EMPTY_EVENTS, **(events or {})}
        self.tick += 1
        balls = {name: (np.asarray(p, float), np.asarray(v, float)) for name, (p, v) in sample["balls"].items()}
        if self.starts is None:
            self.starts = {name: p.copy() for name, (p, _) in balls.items()}
        self.history = (self.history + [sample])[-(round(WINDOW_S * CONTROL_HZ) + 1):]
        self.touch_history = (self.touch_history + [bool(events["left_on_cue"])])[-(round(WINDOW_S * CONTROL_HZ) + 1):]
        for name, (_, v) in balls.items():
            self.peak_speed[name] = max(self.peak_speed.get(name, 0.), float(np.linalg.norm(v)))

        # fouls that can happen at any time
        for name in events["robot_balls"]:
            self.foul(f"a gripper or arm touched the {name} ball")
        for name in events["cue_side_balls"]:
            self.foul(f"the cue's side touched the {name} ball")
        for name in events["tip_object_balls"]:
            self.foul(f"the cue's tip touched the {name} ball")
        for name, (p, _) in balls.items():
            pocket = L.in_pocket(p)
            if pocket and name not in self.potted:
                self.potted[name] = pocket
            if L.off_table(p):
                self.off.add(name)
        if "cue" in self.potted:
            self.foul("the white ball was pocketed")
        if self.off:
            self.foul("a ball left the table: " + ", ".join(sorted(self.off)))

        strike_now = False
        if self.strike_tick is None:
            for name, (p, _) in balls.items():
                if np.linalg.norm(p - self.starts[name]) > DISTURB and events["tip_cue_ball"] == 0:
                    self.foul(f"the {name} ball moved before the strike")
            if events["tip_cue_ball"]:
                strike_now = True
                self.strike_tick = self.tick
                self._judge_strike()
        else:
            gap = self._tip_gap(sample, balls)
            if gap > SEPARATED:
                self.separated = True
            if events["tip_cue_ball"] and self.separated:
                self.foul("the tip touched the white ball a second time")
        self.impulse_ns += float(events["impulse_ns"])
        self.peak_tip_force_n = max(self.peak_tip_force_n, float(events["peak_tip_force_n"]))
        if self.strike_tick is not None and self.first_hit is None and events["cue_ball_hits"]:
            self.first_hit = events["cue_ball_hits"][0]
        if self.potted.get("one") is None and L.past_mouth(balls["one"][0]):
            self.jaws = True
        if self.potted.get("one") == L.TARGET_POCKET:
            self.jaws = True

        # the shot is over when every ball still on the table has been still for 1 s after the strike
        if self.strike_tick is not None and not strike_now:
            moving = [n for n, (p, v) in balls.items() if n not in self.potted and n not in self.off
                      and np.linalg.norm(v) >= STILL_SPEED]
            self.still_ticks = 0 if moving else self.still_ticks + 1
            if self.still_ticks >= round(STILL_S * CONTROL_HZ):
                self.finished = True
        return self.result()

    def _tip_gap(self, sample, balls):
        tip = np.asarray(sample["cue_tip"], float)
        return float(np.linalg.norm(tip - balls["cue"][0]) - L.BALL_R)

    def _judge_strike(self):
        window = round(WINDOW_S * CONTROL_HZ)
        now = self.history[-1]
        bridge = bridge_check(now)
        grip = grip_check(now)
        info = {"bridge": bridge, "grip": grip, "window_ok": len(self.history) > window}
        if len(self.history) > window:
            first = self.history[0]
            left_now, left_then = np.asarray(now["tcp"]["left"][0]), np.asarray(first["tcp"]["left"][0])
            moved = float(np.linalg.norm((left_now - left_then)[:2]))
            # how far the cue moved along its axis relative to each hand (positive: toward the tip)
            slide = axial(left_now, now["cue_tip"], now["cue_axis"]) - axial(left_then, first["cue_tip"], first["cue_axis"])
            right_now, right_then = np.asarray(now["tcp"]["right"][0]), np.asarray(first["tcp"]["right"][0])
            slip = abs(axial(right_now, now["cue_tip"], now["cue_axis"]) - axial(right_then, first["cue_tip"], first["cue_axis"]))
            info.update(bridge_moved_mm=round(1000 * moved, 1), slid_mm=round(1000 * slide, 1), grip_slip_mm=round(1000 * slip, 1))
            touching = float(np.mean(self.touch_history[-window:]))
            info.update(bridge_touch_share=round(touching, 2))
            sliding = moved <= BRIDGE_STILL and slide >= SLIDE_MIN and touching >= BRIDGE_TOUCH
            held = slip <= GRIP_SLIP
        else:
            sliding = held = False
        self.legal = bool(bridge["on_cloth"] and bridge["in_loop"] and bridge["resting"] and sliding
                          and grip["on_axis"] and grip["closed"] and held)
        info["legal"] = self.legal
        self.strike = info

    def result(self):
        score = 0.
        if self.strike_tick is not None and self.legal and not self.fouls:
            score = TIERS["legal"]
            if self.first_hit == "one":
                score = TIERS["first_one"]
                if self.jaws:
                    score = TIERS["jaws"]
                if self.potted.get("one") == L.TARGET_POCKET and "cue" not in self.potted and "cue" not in self.off:
                    score = TIERS["potted"]
        return {"score": score, "struck": self.strike_tick is not None, "strike_tick": self.strike_tick,
                "legal": self.legal, "fouls": list(self.fouls), "first_hit": self.first_hit, "jaws": self.jaws,
                "potted": dict(self.potted), "finished": self.finished, "strike": self.strike,
                "impulse_ns": round(self.impulse_ns, 4), "peak_tip_force_n": round(self.peak_tip_force_n, 1),
                "peak_speed_m_s": {k: round(v, 3) for k, v in self.peak_speed.items()}}


@dataclass(frozen=True)
class PoolScorer:
    name: str = "pool_bridge_score"

    def __call__(self, record, target):
        from inspect_robots.scorer import Score

        if not record.steps:
            return Score(value=0., explanation="No completed control steps")
        g = record.steps[-1].result.info["pool_grade"]
        return Score(value=g["score"], explanation=f"struck: {g['struck']}; legal: {g['legal']}; first hit: "
                                                   f"{g['first_hit']}; potted: {g['potted']}; fouls: {g['fouls']}")


def make_scorer():
    return PoolScorer()
