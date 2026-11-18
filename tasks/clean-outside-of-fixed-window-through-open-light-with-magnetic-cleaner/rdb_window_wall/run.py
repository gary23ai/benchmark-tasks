"""Run one fixed trial in Inspect Robots and preserve its independently scored evidence."""

import argparse
import hashlib
from importlib.metadata import version
import json
import os
from pathlib import Path
import tempfile
import time

from inspect_robots import Scene, eval as inspect_eval
from inspect_robots.guard import ClampGuard, DeltaLimitGuard, GuardChain
from inspect_robots.logging import JsonLogSink
from inspect_robots.task import Task

from . import cleaner_model as model
from . import fixed_scene as fixture
from .adapter import make_embodiment, make_scripted
from .dream_audit import BudgetedDreamClient, DreamAudit, TRANSITION
from .grading import make_scorer

MODEL = "dreamlabs/dream-1-beta"
DREAM_REQUEST_CAP = 64


def live_dream(audit):
    import dream
    from inspect_robots_dream.policy import DreamPolicy
    from inspect_robots_dream.session import DreamSession

    client = BudgetedDreamClient(dream.DreamClient(), DREAM_REQUEST_CAP, audit)
    policy = DreamPolicy(session=DreamSession(make_client=lambda: client),
                         model=MODEL, record=False, timeout_s=90.)
    return policy, client


def phase_diagnostics(trace, phases):
    """The state at the end of each phase of the demonstration (or every 15 s for a model): where the robot's
    readings put each gripper, where each half is, the magnet pull and how far the halves are out of line.
    Positions are in pane coordinates. Diagnostics only."""
    if not trace:
        return []
    if not phases:
        phases = [(f"t={start // fixture.CONTROL_HZ}s", start) for start in range(0, len(trace), 15 * fixture.CONTROL_HZ)]
    rows = []
    for (name, start), (_, stop) in zip(phases, list(phases[1:]) + [("end", len(trace))]):
        start, stop = min(start, len(trace)), min(stop, len(trace))
        if stop <= start:
            continue
        rows.append({"phase": name, "first_step": start + 1, "last_step": stop,
                     "peak_magnet_force_n": max(row["magnet_force_n"] for row in trace[start:stop]),
                     **{key: value for key, value in trace[stop - 1].items() if key != "cells_cleaned"}})
    return rows


def experiment(embodiment, policy, log_dir, *, max_steps=fixture.MAX_STEPS, sinks=None):
    space = embodiment.info.action_space
    guards = GuardChain(ClampGuard(space), DeltaLimitGuard(space, max_delta=1.0), names=["clamp", "delta-limit"])
    task = Task("window-wall-magnetic-cleaner", scenes=[Scene(id="fixed-start", instruction=fixture.INSTRUCTION)],
                scorer=make_scorer(), max_steps=max_steps)
    return inspect_eval(task, policy, embodiment, log_dir=str(log_dir), guard=guards, seed=0,
                        sinks=sinks, grader=None, store_frames=True, store_records=True, fail_on_error=True)[0]


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
        with tempfile.TemporaryDirectory(prefix="window_wall-frames-") as tmp:
            sinks = [JsonLogSink(tmp), audit] if audit is not None else None
            result = experiment(embodiment, policy, Path(tmp), sinks=sinks)
            for path in Path(tmp).glob("*.json"):
                (args.out / path.name).write_bytes(path.read_bytes())
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
        raise RuntimeError("Inspect Robots produced no trial")
    scene = result.samples[0]
    score = scene.reduced.get("window_wall_score")
    if result.status != "success" or score is None:
        detail = scene.error or result.error or result.status
        raise RuntimeError(f"Inspect Robots trial did not complete: {detail}")
    status = {
        "policy": args.policy, "model": MODEL if args.policy == "dream" else None,
        "score": score, "scorer": "window_wall_score", "control_hz": fixture.CONTROL_HZ,
        "step_limit": fixture.MAX_STEPS, "executed_steps": embodiment.tick,
        "elapsed_s": round(time.monotonic() - start, 2),
        "scene_sha256": hashlib.sha256(Path(fixture.__file__).read_bytes() + Path(model.__file__).read_bytes()).hexdigest(),
        "asset_sha256": hashlib.sha256(fixture.USD.read_bytes()).hexdigest(),
        "grader_sha256": hashlib.sha256(Path(__file__).with_name("grading.py").read_bytes()).hexdigest(),
        "inspect_robots": version("inspect-robots"),
        "yam_arena": version("yam-arena"),
        "seed": 0, "observation_state_keys": ["joint_pos"], "observation_cameras": ["top_cam", "left_cam", "right_cam"],
        "policy_inputs_camera_size": [320, 176], "wall_time_limit_is_external": True,
        "dream_chunk_anchor": TRANSITION if audit is not None else None,
        "dream_action_requests": client.calls if client is not None else 0,
        # Diagnostics, not part of the score. The first is read by the batch harness.
        "peak_interarm_contact_n": embodiment.peak_interarm_n,
        "model_pull_through_glass_n": round(model.hold_force()[0], 3),
        "pull_when_paired_n": embodiment.pull_when_paired_n,
        "magnet_substeps": embodiment.substeps,
        "line_display_error": embodiment.line_display_error,
        "demo_readings": getattr(getattr(policy, "reference", None), "report", None),
        "phase_diagnostics": phase_diagnostics(embodiment.trace, getattr(getattr(policy, "reference", None), "phases", None)),
        "final_grade": embodiment.final_grade,
        "termination_reason": scene.termination_reasons[0] if scene.termination_reasons else None,
    }
    (args.out / "RESULT.json").write_text(json.dumps(status, indent=2) + "\n")
    print(json.dumps(status), flush=True)
    return int(args.policy == "scripted" and score != 1.)


if __name__ == "__main__":
    code = main()
    if code:
        # The result and video are already closed. An unsuccessful Isaac
        # trial can hang in Kit's atexit callbacks; return the recorded
        # failure to the batch gate instead of starting a model trial.
        os._exit(code)
