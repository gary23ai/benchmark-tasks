"""The adapter's per-step bookkeeping on fake physics views (no simulator): state reading, contact columns, the
air rule, the scorer and the diagnostics all line up."""

import importlib
import sys
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip("yam_arena")


def _real_inspect_robots():
    """The dry-run stub of inspect_robots (tools/stubs) lacks StepResult; use the real runtime when installed."""
    stubs = [p for p in sys.path if p.replace("\\", "/").endswith("tools/stubs")]
    module = sys.modules.get("inspect_robots")
    if module is not None and hasattr(module, "StepResult"):
        return
    sys.modules.pop("inspect_robots", None)
    for p in stubs:
        sys.path.remove(p)
    try:
        importlib.import_module("inspect_robots").StepResult  # noqa: B018
    except (ImportError, AttributeError):
        pytest.skip("inspect_robots runtime not installed", allow_module_level=True)
    finally:
        sys.path.extend(stubs)


_real_inspect_robots()

from rdb_harmonium import adapter, fixed_scene as fixture  # noqa: E402
from rdb_harmonium import layout as L  # noqa: E402
from rdb_harmonium.air import Air  # noqa: E402
from rdb_harmonium.grading import PhraseGrade  # noqa: E402


class Views:
    def __init__(self):
        self.n = len(fixture.LINKS)
        self.dof = np.zeros(len(fixture.JOINTS))
        self.dof[0] = L.FLAP_OPEN
        self.forces = {arm: np.zeros((9, self.n, 3)) for arm in fixture.ARMS}

    def transforms(self):
        out = np.zeros((self.n, 7))
        out[:, :3] = (*fixture.CASE_START[:2], fixture.TOP)
        out[:, 6] = 1.
        return out


def arena(views):
    a = object.__new__(adapter.HarmoniumArena)
    a.body_view = SimpleNamespace(get_transforms=views.transforms)
    a.rows = {name: i for i, name in enumerate(fixture.LINKS)}
    a.instrument = SimpleNamespace(get_dof_positions=lambda: views.dof.reshape(1, -1).copy())
    a.dofs = {name: i for i, name in enumerate(fixture.JOINTS)}
    a.interarm = SimpleNamespace(get_contact_force_matrix=lambda dt: np.zeros((9, 9, 3)))
    a.arm_contacts = {arm: SimpleNamespace(get_contact_force_matrix=lambda dt, arm=arm: views.forces[arm])
                      for arm in fixture.ARMS}
    a.skin = SimpleNamespace(update=lambda *args: None)
    a.recorder = None
    a._read()
    a.air = Air(flap=a.flap)
    a.pressure = 0.
    a.grading = PhraseGrade(start_xy=fixture.CASE_START[:2])
    a.tick, a.peak_interarm_n = 0, 0.
    a.peak_forces = dict.fromkeys([f"{arm}_arm_on_{p}" for arm in fixture.ARMS for p in ("flap", "case", "keys")], 0.)
    a.diagnostics, a.contact_trace, a.sound_frames = [], [], []
    a._publish_demo_state()
    return a


def test_step_reads_state_feeds_air_and_scores(monkeypatch):
    views = Views()
    a = arena(views)
    obs = SimpleNamespace(state={"joint_pos": np.zeros(14)}, state_time=0.)
    monkeypatch.setattr(adapter.YamArenaEmbodiment, "step", lambda self, action: SimpleNamespace(observation=obs))
    c4 = fixture.JOINTS.index(f"key_{L.prim_name('C4')}_slide")
    left_flap = fixture.LINKS.index("flap")
    right_c4 = fixture.LINKS.index("key_C4")
    for k in range(100):
        views.dof[0] = L.FLAP_OPEN - .15 * (k % 20 < 10) * (k % 10) / 10      # the board pushed in strokes
        views.dof[c4] = -.006 if k >= 10 else 0.                              # C4 pressed from 0.5 s
        views.forces["left"][2, left_flap] = (11.6, 0., 0.)
        views.forces["right"][5, right_c4] = (0., 0., 1.5)
        result = a.step(None)
    grade = result.info["phrase_grade"]
    assert a.demo_state["depths"]["C4"] == pytest.approx(.006) and a.demo_state["flap"] == pytest.approx(a.flap)
    assert grade["longest_s"]["C4"] == pytest.approx(4.5) and grade["score"] == .25
    row = a.contact_trace[-1]
    cols = adapter.CONTACT_TRACE_COLUMNS
    assert row[cols.index("left_arm_on_flap_n")] == pytest.approx(11.6)
    assert row[cols.index("right_arm_on_keys_n")] == pytest.approx(1.5) and row[cols.index("sounding")] == "C4"
    assert a.peak_forces["left_arm_on_flap"] == pytest.approx(11.6) and len(a.diagnostics) == 20
    assert a.diagnostics[-1]["keys_down_mm"] == {"C4": 6.0} and a.air.strokes >= 5


def test_no_pumping_means_no_note(monkeypatch):
    views = Views()
    a = arena(views)
    obs = SimpleNamespace(state={"joint_pos": np.zeros(14)}, state_time=0.)
    monkeypatch.setattr(adapter.YamArenaEmbodiment, "step", lambda self, action: SimpleNamespace(observation=obs))
    views.dof[fixture.JOINTS.index("key_D4_slide")] = -.007
    for _ in range(60):
        result = a.step(None)
    assert result.info["phrase_grade"]["longest_s"]["D4"] == 0. and a.air.volume == 0.
