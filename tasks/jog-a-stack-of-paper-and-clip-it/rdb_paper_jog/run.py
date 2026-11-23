"""Run one fixed trial in Inspect Robots and preserve its independently scored evidence."""

import argparse
import hashlib
from importlib.metadata import version
import json
import os
from pathlib import Path
import tempfile
import time
import traceback

from inspect_robots import Scene, eval as inspect_eval
from inspect_robots.guard import ClampGuard, DeltaLimitGuard, GuardChain
from inspect_robots.task import Task
from inspect_robots.logging import JsonLogSink

from . import fixed_scene as fixture
from . import paper_model as m
from .adapter import make_embodiment, make_scripted
from .dream_audit import BudgetedDreamClient, DreamAudit, TRANSITION
from .grading import make_scorer

MODEL = "dreamlabs/dream-1-beta"
# 2,400 steps are 50 chunks of 48. When the simulator is slow the server answers "expired" and the request is sent
# again at once, which also counts here: 94 requests were used up to step 2,288 in one run. 144 leaves room for
# every chunk to be asked for twice and more; the model still gets exactly the run limit in control steps.
DREAM_REQUEST_CAP = 144
SCORER = "paper_jog_score"


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
    task = Task("jog-a-stack-of-paper-and-clip-it",
                scenes=[Scene(id="fixed-start", instruction=fixture.INSTRUCTION)],
                scorer=make_scorer(), max_steps=max_steps)
    return inspect_eval(task, policy, embodiment, log_dir=str(log_dir), guard=guards, seed=0,
                        sinks=sinks, grader=None, store_frames=True, store_records=True, fail_on_error=True)[0]


def diagnostics(embodiment, reference):
    """Everything logged for whoever reads the run afterwards. Nothing here is ever given to a model policy."""
    phase_ends = []
    if reference is not None:
        starts = reference.phases + [(embodiment.tick + 1, "end")]
        for (first, name), (after, _) in zip(starts, starts[1:]):
            rows = [row for row in embodiment.diagnostics if first < row["tick"] <= after]
            if rows:
                phase_ends.append({"phase": name, "first_tick": first + 1, **rows[-1]})
    return {
        "phase_ends": phase_ends,
        "lift_checks": embodiment.lift_checks,
        "samples": embodiment.diagnostics,
        "phases": [list(row) for row in reference.phases] if reference is not None else None,
        "reference": json.loads(json.dumps(reference.log, default=float)) if reference is not None else None,
        "worst_ik_mm": round(1000 * reference.worst_ik, 3) if reference is not None else None,
    }


def model_numbers():
    return {"plates": m.K, "sheets_per_plate": m.PER_PLATE, "plate_mm": round(1000 * m.PLATE_T, 3),
            "plate_mass_g": round(1000 * m.PLATE_MASS, 2), "paper_friction": list(m.PAPER_FRICTION),
            "clip_force_on_stack_n": round(float(m.clamp_force(m.STACK_T)), 2),
            "clip_lip_gap_open_mm": round(1000 * float(m.JAW_GAP_OPEN), 2), "physics_hz": m.PHYSICS_HZ,
            "solver_iterations": list(m.SOLVER_ITERATIONS), "contact_offset_mm": 1000 * m.CONTACT_OFFSET,
            "start_misalignment_mm": [round(1000 * v, 2) for v in m.misalignment(list(m.start_poses().values()))]}


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
        try:
            with tempfile.TemporaryDirectory(prefix="paper-jog-frames-") as tmp:
                sinks = [JsonLogSink(tmp), audit] if audit is not None else None
                result = experiment(embodiment, policy, Path(tmp), sinks=sinks)
                for path in Path(tmp).glob("*.json"):
                    (args.out / path.name).write_bytes(path.read_bytes())
        except BaseException:
            # Failure record first, so a crashed trial still tells us how far it got.
            failure = {"policy": args.policy, "failed": True, "error": traceback.format_exc(limit=12),
                       "executed_steps": embodiment.tick, "final_grade": embodiment.final_grade,
                       "peak_interarm_contact_n": getattr(embodiment, "peak_interarm_n", None),
                       "state_after_reset": getattr(embodiment, "state_after_reset", None),
                       "model_numbers": model_numbers()}
            try:
                failure["diagnostics"] = diagnostics(embodiment, getattr(policy, "reference", None))
            except Exception as error:      # never hide the original failure
                failure["diagnostics_error"] = repr(error)
            (args.out / "RESULT.json").write_text(json.dumps(failure, indent=1, default=str) + "\n")
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
        raise RuntimeError("Inspect Robots produced no trial")
    scene = result.samples[0]
    score = scene.reduced.get(SCORER)
    if result.status != "success" or score is None:
        detail = scene.error or result.error or result.status
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
        "model_sha256": hashlib.sha256((package / "paper_model.py").read_bytes()).hexdigest(),
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
        "peak_contact_n": {k: round(v, 2) for k, v in embodiment.peaks.items()},
        "state_after_reset": embodiment.state_after_reset,
        "model_numbers": model_numbers(),
        "final_grade": embodiment.final_grade,
        "termination_reason": scene.termination_reasons[0] if scene.termination_reasons else None,
    }
    print(json.dumps(status), flush=True)
    status["diagnostics"] = diagnostics(embodiment, reference)
    (args.out / "RESULT.json").write_text(json.dumps(status, indent=1, default=float) + "\n")
    return int(args.policy == "scripted" and (score != 1. or variant != "full"))


if __name__ == "__main__":
    code = main()
    if code:
        # The result and videos are already closed. An unsuccessful Isaac trial can hang in Kit's exit
        # callbacks; return the recorded failure to the batch gate instead of starting a model trial.
        os._exit(code)
