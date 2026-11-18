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

from . import cake_model as cake, fixed_scene as fixture
from .adapter import make_embodiment, make_scripted
from .dream_audit import BudgetedDreamClient, DreamAudit, TRANSITION
from .grading import make_scorer

MODEL = "dreamlabs/dream-1-beta"
# Dream-1 returns 48 steps at a time, so a full trial needs 34 replies. In the first recorded trial every
# reply was preceded by one request that the service reported as expired, which is two requests per reply;
# the cap allows four. A request refused by the cap does not stop the trial: the arms hold their last
# target to the run limit and the trial is scored as it stands (dream_audit.BudgetedDreamClient).
DREAM_CHUNK_STEPS = 48
DREAM_REQUEST_CAP = 4 * -(-fixture.MAX_STEPS // DREAM_CHUNK_STEPS)      # 136


def outcome(result, failure, final_grade):
    """(score, completed, why not) of a trial, whether or not Inspect Robots finished it.

    A trial that finished is scored by the scorer. One that was stopped by an error keeps the grade of its
    last executed step, which is what the scorer would have read; with no executed step there is no score.
    """
    sample = result.samples[0] if result is not None and result.samples else None
    scored = sample.reduced.get(fixture.SCORER) if sample is not None else None
    if failure is None and result is not None and result.status == "success" and scored is not None:
        return scored, True, None
    if failure is not None:
        detail = f"{type(failure).__name__}: {failure}"
    elif sample is None:
        detail = "Inspect Robots produced no trial"
    else:
        detail = str(sample.error or result.error or result.status)
    return (final_grade["score"] if final_grade else None), False, detail


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
    task = Task(fixture.TASK_ID, scenes=[Scene(id="fixed-start", instruction=fixture.INSTRUCTION)],
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
    result = failure = None
    try:
        with tempfile.TemporaryDirectory(prefix="cake_cut-frames-") as tmp:
            sinks = [JsonLogSink(tmp), audit] if audit is not None else None
            try:
                result = experiment(embodiment, policy, Path(tmp), sinks=sinks)
            finally:
                for path in Path(tmp).glob("*.json"):
                    (args.out / path.name).write_bytes(path.read_bytes())
    except Exception as error:      # reported below, after the result file has been written
        failure = error
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
                    {"action_requests": client.calls, "cap": DREAM_REQUEST_CAP,
                     "cap_reached_at_step": client.capped_at, "trials": 1,
                     "list_price_estimate_usd": round(client.calls * 3 / 2250, 6)}, indent=2) + "\n")
    # Written for every trial that started, finished or not: a trial stopped by an error keeps the score of
    # its last executed step, marked as not completed, with the reason.
    final_grade = getattr(embodiment, "final_grade", None)
    score, completed, detail = outcome(result, failure, final_grade)
    scene = result.samples[0] if result is not None and result.samples else None
    status = {
        "policy": args.policy, "model": MODEL if args.policy == "dream" else None,
        "score": score, "scorer": fixture.SCORER, "completed": completed, "error": detail,
        "control_hz": fixture.CONTROL_HZ,
        "step_limit": fixture.MAX_STEPS, "executed_steps": getattr(embodiment, "tick", 0),
        "elapsed_s": round(time.monotonic() - start, 2),
        "scene_sha256": hashlib.sha256(Path(fixture.__file__).read_bytes()).hexdigest(),
        "asset_sha256": hashlib.sha256(fixture.USD.read_bytes()).hexdigest(),
        "grader_sha256": hashlib.sha256(Path(__file__).with_name("grading.py").read_bytes()).hexdigest(),
        "inspect_robots": version("inspect-robots"),
        "yam_arena": version("yam-arena"),
        "seed": 0, "observation_state_keys": ["joint_pos"], "observation_cameras": ["top_cam", "left_cam", "right_cam"],
        "policy_inputs_camera_size": [320, 176], "wall_time_limit_is_external": True,
        "dream_chunk_anchor": TRANSITION if audit is not None else None,
        "dream_action_requests": client.calls if client is not None else 0,
        "dream_request_cap": DREAM_REQUEST_CAP if client is not None else None,
        "dream_request_cap_reached_at_step": client.capped_at if client is not None else None,
        # Diagnostics. None of them enters the score.
        "peak_interarm_contact_n": getattr(embodiment, "peak_interarm_n", None),
        "peak_wire_tension_n": getattr(embodiment, "peak_tension", None),
        "peak_handle_force_n": getattr(embodiment, "peak_handle_force", None),
        "wire_force_limit_n": fixture.END_PULL_LIMIT,
        "peak_cake_push_n": getattr(embodiment, "peak_cake_push", None),
        "cake_slide_force_n": cake.CAKE_SLIDE_FORCE,
        "demo_gripper_readings": getattr(getattr(policy, "reference", None), "report", None),
        "final_grade": final_grade,
        "termination_reason": scene.termination_reasons[0] if scene is not None and scene.termination_reasons else None,
    }
    (args.out / "RESULT.json").write_text(json.dumps(status, indent=2) + "\n")
    print(json.dumps(status), flush=True)
    if not completed:
        raise RuntimeError(f"Inspect Robots trial did not complete: {detail} (RESULT.json has the score reached)")
    return int(args.policy == "scripted" and score != 1.)


if __name__ == "__main__":
    code = main()
    if code:
        # The result and video are already closed. An unsuccessful Isaac
        # trial can hang in Kit's atexit callbacks; return the recorded
        # failure to the batch gate instead of starting a model trial.
        os._exit(code)
