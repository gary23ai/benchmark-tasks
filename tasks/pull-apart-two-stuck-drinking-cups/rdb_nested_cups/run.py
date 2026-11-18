"""Run one fixed trial in Inspect Robots and preserve its independently scored evidence."""

import argparse
import hashlib
from importlib.metadata import version
import json
from pathlib import Path
import tempfile
import time

from inspect_robots import Scene, eval as inspect_eval
from inspect_robots.guard import ClampGuard, DeltaLimitGuard, GuardChain
from inspect_robots.logging import JsonLogSink
from inspect_robots.task import Task

from . import cup_model as cup
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


def phase_snapshots(reference, embodiment):
    """For each phase boundary the demo logged: the robot's readings ("robot") and the cup poses the demo used
    ("demo_saw"), next to the simulator's full record of the same moment ("simulator": cup poses, contact
    forces, the fit's force). For the result file only."""
    if reference is None:
        return None
    rows = []
    for entry in reference.phases:
        index = min(max(entry["step"] - 1, 0), len(embodiment.history) - 1)
        rows.append({"phase": entry["phase"], "step": entry["step"],
                     "robot": {arm: entry[arm] for arm in fixture.ARMS},
                     "demo_saw": {k: v for k, v in entry.items() if k not in ("phase", "step", *fixture.ARMS)},
                     "simulator": embodiment.history[index] if embodiment.history else None})
    return rows


def peaks(history):
    """Largest contact forces of the trial, per arm and cup, with the step at which each occurred."""
    out = {}
    for arm in fixture.ARMS:
        for name in fixture.CUPS:
            squeeze = [sum(row["arms"][arm]["cup_contact"][name]["links_n"].values()) for row in history]
            lift = [row["arms"][arm]["cup_contact"][name]["net_n"][2] for row in history]
            if not history:
                continue
            i, j = max(range(len(history)), key=squeeze.__getitem__), max(range(len(history)), key=lambda k: abs(lift[k]))
            out[f"{arm}_on_{name}"] = {"peak_total_contact_n": round(squeeze[i], 2), "at_tick": history[i]["tick"],
                                       "peak_vertical_net_n": round(lift[j], 2), "vertical_at_tick": history[j]["tick"]}
    pitch = [row["inner_above_outer_mm"] for row in history]
    if pitch:
        out["inner_above_outer_mm"] = {"min": min(pitch), "max": max(pitch), "last": pitch[-1]}
        out["max_cup_tilt_deg"] = {name: max(row["cups"][name]["tilt_deg"] for row in history) for name in fixture.CUPS}
    return out


def experiment(embodiment, policy, log_dir, *, max_steps=fixture.MAX_STEPS, sinks=None):
    space = embodiment.info.action_space
    guards = GuardChain(ClampGuard(space), DeltaLimitGuard(space, max_delta=1.0), names=["clamp", "delta-limit"])
    task = Task("stuck-cups", scenes=[Scene(id="fixed-start", instruction=fixture.INSTRUCTION)],
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
        with tempfile.TemporaryDirectory(prefix="stuck_cups-frames-") as tmp:
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
    score = scene.reduced.get("nested_cups_score")
    if result.status != "success" or score is None:
        detail = scene.error or result.error or result.status
        raise RuntimeError(f"Inspect Robots trial did not complete: {detail}")
    status = {
        "probe": cup.PROBE,
        "policy": args.policy, "model": MODEL if args.policy == "dream" else None,
        "score": score, "scorer": "nested_cups_score", "control_hz": fixture.CONTROL_HZ,
        "step_limit": fixture.MAX_STEPS, "executed_steps": embodiment.tick,
        "elapsed_s": round(time.monotonic() - start, 2),
        "scene_sha256": hashlib.sha256(Path(fixture.__file__).read_bytes() + Path(cup.__file__).read_bytes()).hexdigest(),
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
        # The stuck fit: a bond that breaks at fit_break_force_n. "fit_events" says when it let go and
        # whether it was still holding 5 s into the trial and when a robot first touched a cup.
        "fit_break_force_n": cup.FIT_BREAK_FORCE_N,
        "fit_break_torque_nm": cup.FIT_BREAK_TORQUE_NM,
        "fit_joint_on_stage": embodiment.fit_joint,
        "fit_at_reset": embodiment.reset_fit,
        "fit_events": embodiment.fit_events,
        "bound_physics_materials": embodiment.reset_materials,
        "contact_force_note": "contact forces are normal forces only (no friction), from the last physics step",
        "settled_pitch_mm": round(embodiment.reset_pitch_mm, 3),
        "authored_pitch_mm": round(1000 * cup.SEATED_PITCH, 3),
        "demo_gripper_readings": getattr(getattr(policy, "reference", None), "report", None),
        "demo_phases": phase_snapshots(getattr(policy, "reference", None), embodiment),
        "diagnostic_peaks": peaks(embodiment.history),
        "final_grade": embodiment.last_grade,
        "termination_reason": scene.termination_reasons[0] if scene.termination_reasons else None,
    }
    (args.out / "RESULT.json").write_text(json.dumps(status, indent=2) + "\n")
    print(json.dumps(status), flush=True)
    return int(args.policy == "scripted" and score != 1. and not cup.PROBE)


if __name__ == "__main__":
    raise SystemExit(main())
