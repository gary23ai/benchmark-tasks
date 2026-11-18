"""Run one fixed trial in Inspect Robots and preserve its independently scored evidence."""

import argparse
import hashlib
from importlib.metadata import version
import json
import os
from pathlib import Path
import tempfile
import time

import numpy as np

from inspect_robots import Scene, eval as inspect_eval
from inspect_robots.guard import ClampGuard, DeltaLimitGuard, GuardChain
from inspect_robots.logging import JsonLogSink
from inspect_robots.task import Task

from . import fixed_scene as fixture
from . import layout
from .adapter import make_embodiment, make_scripted
from .dream_audit import BudgetedDreamClient, DreamAudit, TRANSITION
from .grading import make_scorer

MODEL = "dreamlabs/dream-1-beta"
DREAM_REQUEST_CAP = 96
SCORER = "child_latch_score"


def live_dream(audit):
    import dream
    from inspect_robots_dream.policy import DreamPolicy
    from inspect_robots_dream.session import DreamSession

    client = BudgetedDreamClient(dream.DreamClient(), DREAM_REQUEST_CAP, audit)
    policy = DreamPolicy(session=DreamSession(make_client=lambda: client),
                         model=MODEL, record=False, timeout_s=90.)
    return policy, client


def experiment(embodiment, policy, log_dir, *, max_steps=fixture.MAX_STEPS, sinks=None):
    space = embodiment.info.action_space
    guards = GuardChain(ClampGuard(space), DeltaLimitGuard(space, max_delta=1.0), names=["clamp", "delta-limit"])
    task = Task("open-a-drawer-past-its-child-safety-latch",
                scenes=[Scene(id="fixed-start", instruction=fixture.INSTRUCTION)],
                scorer=make_scorer(), max_steps=max_steps)
    return inspect_eval(task, policy, embodiment, log_dir=str(log_dir), guard=guards, seed=0,
                        sinks=sinks, grader=None, store_frames=True, store_records=True, fail_on_error=True)[0]


def diagnostics(embodiment, reference):
    """Everything the run log keeps beyond the score, for whoever reads the run afterwards. None of it is ever
    given to a model policy and none of it enters the score."""
    out = {
        "peak_arm_on_latch_n": [round(v, 2) for v in getattr(embodiment, "peak_arm_on_latch_n", [0., 0.])],  # left, right
        "peak_arm_on_drawer_n": [round(v, 2) for v in getattr(embodiment, "peak_arm_on_drawer_n", [0., 0.])],
        # The hook on the catch: what holds the drawer at 38 mm (None if the contact view could not be made).
        "peak_hook_on_catch_n": round(getattr(embodiment, "peak_hook_on_catch_n", 0.), 2),
        "catch_view_error": getattr(embodiment, "catch_view_error", None),
        # The furthest the drawer was open while the latch arm was up, over the whole run. Before the hook is past
        # the catch this cannot exceed about 38 mm; afterwards the arm is up again by design (see the trace).
        "peak_open_with_latch_up_mm": round(getattr(embodiment, "peak_open_with_latch_up_mm", 0.), 2),
        "peak_latch_angle_deg": round(getattr(embodiment, "peak_latch_angle_deg", 0.), 2),
        # The latch's pin should stay on the drawer: more than 1 mm means the joint gave way.
        "peak_latch_pin_offset_mm": round(getattr(embodiment, "peak_pin_offset_mm", 0.), 2),
        "latch_angle_after_reset_deg": getattr(embodiment, "latch_angle_after_reset_deg", None),
        "first_tick_past_catch": getattr(embodiment, "first_tick_past_catch", None),
        "model_numbers": {"catch_open_mm": round(1000 * layout.catch_open(), 1),
                          "hook_clear_open_mm": round(1000 * layout.hook_clear_open(), 1),
                          "hook_overlap_mm": round(1000 * layout.ENGAGE, 1),
                          "release_angle_deg": round(float(np.degrees(layout.RELEASE_ANGLE)), 2),
                          "stop_angle_deg": round(float(np.degrees(layout.LATCH_TRAVEL)), 2),
                          "press_force_21mm_behind_pin_n": round(layout.press_force(layout.RELEASE_ANGLE, layout.HINGE_X + .021), 2),
                          "runner_friction_n": layout.RUNNER_FRICTION_N},
    }
    phase_ends = []
    samples = getattr(embodiment, "diagnostics", [])
    if reference is not None:
        starts = reference.phases + [(embodiment.tick + 1, "end")]
        for (first, name), (after, _) in zip(starts, starts[1:]):
            rows = [row for row in samples if first < row["tick"] <= after]
            if rows:
                phase_ends.append({"phase": name, "first_tick": first + 1,
                                   "peak_arm_on_latch_n": [max(r["arm_on_latch_n"][i] for r in rows) for i in range(2)],
                                   "peak_arm_on_drawer_n": [max(r["arm_on_drawer_n"][i] for r in rows) for i in range(2)],
                                   **rows[-1]})
    out["diagnostics"] = {
        "phase_ends": phase_ends,
        "samples": samples,
        "phases": [list(row) for row in reference.phases] if reference is not None else None,
        "reference": json.loads(json.dumps(reference.log, default=float)) if reference is not None else None,
        "worst_ik_mm": round(1000 * reference.worst_ik, 3) if reference is not None else None,
    }
    # Every control step: tick, open (mm), latch angle (deg), left arm on the latch (N), right arm on the drawer
    # (N), hook on catch (N), right arm on the latch (N), left arm on the drawer (N).
    out["trace_columns"] = ["tick", "open_mm", "latch_deg", "left_on_latch_n", "right_on_drawer_n", "hook_on_catch_n",
                            "right_on_latch_n", "left_on_drawer_n"]
    out["trace"] = getattr(embodiment, "trace", [])
    return out


def write_failure(args, embodiment, policy, start, error):
    """RESULT.json for a run that stopped on an error: the scene checks, the diagnostics so far, the trace."""
    failure = {"policy": args.policy, "score": None, "error": error, "executed_steps": getattr(embodiment, "tick", 0),
               "elapsed_s": round(time.monotonic() - start, 2),
               "setup_problems": getattr(embodiment, "setup_problems", None),
               "peak_interarm_contact_n": getattr(embodiment, "peak_interarm_n", None),
               "final_grade": getattr(embodiment, "final_grade", None),
               **diagnostics(embodiment, getattr(policy, "reference", None))}
    (args.out / "RESULT.json").write_text(json.dumps(failure, indent=1, default=str) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("policy", choices=["scripted", "dream"])
    parser.add_argument("--out", required=True, type=Path)
    args = parser.parse_args()
    start = time.monotonic()
    args.out.mkdir(parents=True, exist_ok=False)
    embodiment = make_embodiment(media_dir=args.out / "media")
    audit = DreamAudit(args.out) if args.policy == "dream" else None
    policy, client = live_dream(audit) if audit is not None else (make_scripted(), None)
    result = None
    try:
        with tempfile.TemporaryDirectory(prefix="child-latch-frames-") as tmp:
            sinks = [JsonLogSink(tmp), audit] if audit is not None else None
            result = experiment(embodiment, policy, Path(tmp), sinks=sinks)
            for path in Path(tmp).glob("*.json"):
                (args.out / path.name).write_bytes(path.read_bytes())
    except BaseException as error:
        write_failure(args, embodiment, policy, start, repr(error))
        raise
    finally:
        try:
            embodiment.close()
        finally:
            if client is not None:
                try:
                    policy.close()
                finally:
                    audit.close()
                (args.out / "DREAM_USAGE.json").write_text(json.dumps(
                    {"action_requests": client.calls, "cap": DREAM_REQUEST_CAP, "trials": 1,
                     "list_price_estimate_usd": round(client.calls * 3 / 2250, 6)}, indent=2) + "\n")
    if result is None or not result.samples:
        write_failure(args, embodiment, policy, start, "Inspect Robots produced no trial")
        raise RuntimeError("Inspect Robots produced no trial")
    scene = result.samples[0]
    score = scene.reduced.get(SCORER)
    if result.status != "success" or score is None:
        detail = scene.error or result.error or result.status
        write_failure(args, embodiment, policy, start, f"trial did not complete: {detail}")
        raise RuntimeError(f"Inspect Robots trial did not complete: {detail}")
    package = Path(__file__).parent
    reference = getattr(policy, "reference", None)
    variant = getattr(policy, "variant", None)
    status = {
        "policy": args.policy, "model": MODEL if args.policy == "dream" else None,
        "score": score, "scorer": SCORER, "control_hz": fixture.CONTROL_HZ,
        "step_limit": fixture.MAX_STEPS, "executed_steps": embodiment.tick,
        "elapsed_s": round(time.monotonic() - start, 2),
        "scene_sha256": hashlib.sha256(Path(fixture.__file__).read_bytes()).hexdigest(),
        "asset_sha256": hashlib.sha256(fixture.USD.read_bytes()).hexdigest(),
        "layout_sha256": hashlib.sha256((package / "layout.py").read_bytes()).hexdigest(),
        "grader_sha256": hashlib.sha256((package / "grading.py").read_bytes()).hexdigest(),
        "inspect_robots": version("inspect-robots"),
        "yam_arena": version("yam-arena"),
        "seed": 0, "observation_state_keys": ["joint_pos"], "observation_cameras": ["top_cam", "left_cam", "right_cam"],
        "policy_inputs_camera_size": [320, 176], "wall_time_limit_is_external": True,
        "scripted_demo_reads_simulator_state": args.policy == "scripted",
        "scripted_demo_variant": variant,
        "dream_chunk_anchor": TRANSITION if audit is not None else None,
        "dream_action_requests": client.calls if client is not None else 0,
        # Diagnostics only, none of them part of the score. The first is read by the batch harness.
        "peak_interarm_contact_n": embodiment.peak_interarm_n,
        "final_grade": embodiment.final_grade,
        "termination_reason": scene.termination_reasons[0] if scene.termination_reasons else None,
    }
    status.update(diagnostics(embodiment, reference))
    print(json.dumps({k: v for k, v in status.items() if k not in ("diagnostics", "trace")}), flush=True)
    (args.out / "RESULT.json").write_text(json.dumps(status, indent=1, default=float) + "\n")
    return int(args.policy == "scripted" and (score != 1. or variant != "full"))


if __name__ == "__main__":
    code = main()
    if code:
        # The result and videos are already closed. An unsuccessful Isaac trial can hang in Kit's exit
        # callbacks; return the recorded failure to the batch gate instead of starting a model trial.
        os._exit(code)
