"""Score the phrase played on the harmonium. NumPy only.

The score is built from one thing per 50 ms frame: which keys' reeds are sounding. In a real trial that is read
from the microphone recording by the calibrated pitch rule in ``tools/judge_audio.py`` (each note's band level
against its own reference level, recorded just before the trial); in simulation it is read from the simulator:
a key's reed sounds while that key is down at least 2.5 mm and the reservoir pressure is above the reed's
speaking pressure (air.py). Both feed the same ``PhraseGrade``.

Definitions (times in seconds, 20 frames a second):
- A note's sounding run: consecutive frames in which it sounds; silences of 0.1 s or less inside a run are
  bridged (a real recording flickers at a pump reversal). Its length runs from its first to its last sounding frame.
- Held: a run of at least HOLD_S (4.0 s). A full reservoir keeps one reed sounding for under 3.1 s, so a held
  note needs air pumped in while the key is down: both hands at once.
- In order: Re's held run starts after Sa's held run starts and ends after it ends; Ga likewise after Re.
- Legato (the sound never stops): the next note starts before the previous one stops, as a player does it, or at
  most MAX_GAP_S (0.1 s, two frames) after it; and no more than MAX_OVERLAP_S (1.0 s) before it stops (longer
  than that the two notes are a chord, not a phrase).
- Detached: joined with a silence of at most DETACHED_GAP_S (0.5 s).

Tiers: 0.25 Sa held; 0.5 Sa then Re held, in order; 0.75 Sa, Re, Ga held, in order. 0.85: in addition both
joins at least detached, no other key sounding for more than OTHER_KEYS_S (0.2 s) in all, the case never moved
more than MOVED_M (10 mm) from its marks, and both grippers off the harmonium (keys, bellows board and case) with
every key up for the last second (RELEASE_S). 1.0: all that with both joins legato. Any time the case is pushed more than VOID_MOVED_M (30 mm) or
tipped more than VOID_TILT_RAD (5 degrees), the score is 0 for the rest of the trial.
"""

from dataclasses import dataclass, field
from itertools import product

import numpy as np

CONTROL_HZ = 20
DT = 1. / CONTROL_HZ
PHRASE = ("C4", "D4", "E4")
HOLD_S = 4.0
BRIDGE_S = .1
MAX_GAP_S = .1
DETACHED_GAP_S = .5
MAX_OVERLAP_S = 1.0
OTHER_KEYS_S = .2
MOVED_M = .010
VOID_MOVED_M = .030
VOID_TILT_RAD = float(np.radians(5.))
RELEASE_S = 1.0
TIERS = (.25, .5, .75, .85, 1.)
_HOLD = int(round(HOLD_S * CONTROL_HZ))          # frames
_BRIDGE = int(round(BRIDGE_S * CONTROL_HZ))
_RELEASE = int(round(RELEASE_S * CONTROL_HZ))


@dataclass
class Run:
    first: int                      # first sounding frame
    last: int                       # last sounding frame

    @property
    def frames(self):
        return self.last - self.first + 1


@dataclass
class PhraseGrade:
    start_xy: tuple = (0., 0.)      # case position on its marks (station frame, m)
    frame: int = -1
    runs: dict = field(default_factory=dict)        # key name -> [Run]
    other_frames: int = 0           # frames in which any key outside the phrase sounded
    moved_max_m: float = 0.
    tilt_max_rad: float = 0.
    void: str | None = None
    released_frames: int = 0
    fed: bool = False               # air has been pumped into the reservoir at some point (diagnostics)
    last: dict | None = None

    def _note(self, name):
        runs = self.runs.setdefault(name, [])
        if runs and self.frame - runs[-1].last <= _BRIDGE + 1:
            runs[-1].last = self.frame
        else:
            runs.append(Run(self.frame, self.frame))

    def held(self, name):
        return [r for r in self.runs.get(name, []) if r.frames >= _HOLD]

    def phrase(self):
        """(tier from holding and order, how the best ordered choice of held runs is joined: "legato",
        "detached" or None, its joins in s)."""
        sa, re, ga = (self.held(n) for n in PHRASE)
        tier, joined, joins = (.25 if sa else 0.), None, None
        rank = {None: 0, "detached": 1, "legato": 2}

        def ordered(a, b):
            return b.first > a.first and b.last > a.last

        def join(a, b):                       # positive: a gap; negative: an overlap (s)
            return (b.first - a.last - 1) * DT

        def kind(a, b):
            j = join(a, b)
            if j < -MAX_OVERLAP_S - 1e-9 or j > DETACHED_GAP_S + 1e-9:
                return None
            return "legato" if j <= MAX_GAP_S + 1e-9 else "detached"

        for a, b in product(sa, re):
            if ordered(a, b):
                tier = max(tier, .5)
                for c in ga:
                    if ordered(b, c):
                        tier = .75
                        k1, k2 = kind(a, b), kind(b, c)
                        both = None if None in (k1, k2) else ("legato" if k1 == k2 == "legato" else "detached")
                        if joins is None or rank[both] > rank[joined]:
                            joined, joins = both, (round(join(a, b), 2), round(join(b, c), 2))
        return tier, joined, joins

    def update(self, sounding, case_position, case_rotation, hands_on, keys_up, fed=False):
        """One 20 Hz frame. ``sounding``: names of the keys sounding; the case pose in the station frame
        (origin on the table under its front edge); ``hands_on``: either gripper touches the harmonium;
        ``keys_up``: every key is back up (less than 1 mm down)."""
        self.frame += 1
        for name in sounding:
            self._note(name)
        if any(name not in PHRASE for name in sounding):
            self.other_frames += 1
        self.fed = self.fed or bool(fed)
        position = np.asarray(case_position, dtype=float)
        rotation = np.asarray(case_rotation, dtype=float)
        moved = float(np.linalg.norm(position[:2] - np.asarray(self.start_xy)))
        tilt = float(np.arccos(np.clip(rotation[2, 2], -1., 1.)))
        self.moved_max_m = max(self.moved_max_m, moved)
        self.tilt_max_rad = max(self.tilt_max_rad, tilt)
        if self.void is None:
            if tilt > VOID_TILT_RAD:
                self.void = "harmonium tipped"
            elif moved > VOID_MOVED_M:
                self.void = "harmonium pushed off its marks"
        self.released_frames = self.released_frames + 1 if (not hands_on and keys_up) else 0
        released = self.released_frames >= _RELEASE
        tier, joined, joins = self.phrase()
        clean = (joined is not None and self.other_frames * DT <= OTHER_KEYS_S + 1e-9
                 and self.moved_max_m <= MOVED_M)
        played = ({"legato": 1., "detached": .85}[joined] if tier == .75 and clean else tier)
        score = 0. if self.void else (played if played <= .75 or released else .75)
        held = {n: round(max((r.frames for r in self.runs.get(n, [])), default=0) * DT, 2) for n in PHRASE}
        # score: the final score (after the release rule and the latched zeros); played: what the playing earned
        # (0 to 1.0) before the release rule; held_in_order: the part of that from holding and order alone
        # (0, 0.25, 0.5 or 0.75), before the joins, other keys and case motion are judged.
        self.last = {"score": score, "played": played, "held_in_order": tier, "joined": joined, "joins_s": joins,
                     "longest_s": held, "sounding": list(sounding), "other_keys_s": round(self.other_frames * DT, 2),
                     "moved_mm": round(1000 * moved, 2), "moved_max_mm": round(1000 * self.moved_max_m, 2),
                     "tilt_max_deg": round(float(np.degrees(self.tilt_max_rad)), 2), "released": released,
                     "void": self.void, "fed": self.fed}
        return self.last

    @property
    def finished(self):
        """Nothing more can change the score."""
        return self.void is not None or (self.last is not None and self.last["score"] == 1.)


@dataclass(frozen=True)
class PhraseScorer:
    name: str = "harmonium_phrase_score"

    def __call__(self, record, target):
        from inspect_robots.scorer import Score

        if not record.steps:
            return Score(value=0., explanation="No completed control steps")
        grade = record.steps[-1].result.info["phrase_grade"]
        return Score(value=grade["score"],
                     explanation=f"Played {grade['played']} (held in order {grade['held_in_order']}, "
                                 f"joined {grade['joined']}); longest runs {grade['longest_s']} s; joins {grade['joins_s']}; "
                                 f"other keys {grade['other_keys_s']} s; moved {grade['moved_max_mm']} mm; "
                                 f"released {grade['released']}; void {grade['void']}")


def make_scorer():
    return PhraseScorer()
