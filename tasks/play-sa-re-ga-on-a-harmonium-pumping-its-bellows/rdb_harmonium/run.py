"""Run one fixed trial in Inspect Robots and preserve its independently scored evidence."""

import argparse
import hashlib
import importlib
from importlib.metadata import version
import json
import os
from pathlib import Path
import tempfile
import sys
import time
import traceback


from inspect_robots import Scene, eval as inspect_eval
from inspect_robots.guard import ClampGuard, DeltaLimitGuard, GuardChain
from inspect_robots.logging import JsonLogSink
from inspect_robots.task import Task

from . import fixed_scene as fixture
from . import layout as L
from .air import Air
from .dream_audit import BudgetedDreamClient, DreamAudit, TRANSITION
from .grading import make_scorer

MODEL = "dreamlabs/dream-1-beta"
DREAM_REQUEST_CAP = 128
SCORER = "harmonium_phrase_score"


def ensure_yam_arena():
    """Make yam_arena importable before the adapter (which needs it) is imported.

    Run 6's Dream-1 trial stopped at import with "No module named 'yam_arena'" although the scripted trial just before
    it, in the same shell, imported it fine. So the adapter is now imported only here, inside main(); if yam_arena is
    missing, the station's own source tree named by the job environment (YAM_UPSTREAM, or ARENA_DIR's upstream) is put
    on the path; and if that fails too, a RESULT.json says what the path held instead of a bare traceback."""
    tried = []
    for candidate in [None] + [os.environ.get(k) for k in ("YAM_UPSTREAM", "YAM_ARENA_SRC")] + (
            [str(Path(os.environ["ARENA_DIR"]).parent / "upstream")] if os.environ.get("ARENA_DIR") else []):
        if candidate is not None:
            if not candidate or candidate in sys.path or not (Path(candidate) / "yam_arena").is_dir():
                continue
            sys.path.insert(0, candidate)
            importlib.invalidate_caches()
        try:
            importlib.import_module("yam_arena")
            return candidate
        except ModuleNotFoundError as error:
            tried.append([candidate, repr(error)])
    raise ModuleNotFoundError(f"yam_arena not importable; tried {tried}; sys.path={sys.path}")


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
    task = Task("play-sa-re-ga-on-a-harmonium-pumping-its-bellows",
                scenes=[Scene(id="fixed-start", instruction=fixture.INSTRUCTION)],
                scorer=make_scorer(), max_steps=max_steps)
    return inspect_eval(task, policy, embodiment, log_dir=str(log_dir), guard=guards, seed=0,
                        sinks=sinks, grader=None, store_frames=True, store_records=True, fail_on_error=True)[0]


def diagnostics(embodiment, reference):
    """Everything logged for whoever reads the run afterwards. Nothing here is ever given to a model policy."""
    from .adapter import CONTACT_TRACE_COLUMNS

    phase_ends = []
    if reference is not None:
        starts = reference.phases + [(embodiment.tick + 1, "end")]
        for (first, name), (after, _) in zip(starts, starts[1:]):
            rows = [row for row in embodiment.diagnostics if first < row["tick"] <= after]
            trace = [row for row in embodiment.contact_trace if first < row[0] <= after]
            if rows:
                peaks = ({col: max(r[i] for r in trace) for i, col in enumerate(CONTACT_TRACE_COLUMNS)
                          if col.endswith("_n")} if trace else {})
                lowest_flap = min(r[CONTACT_TRACE_COLUMNS.index("flap_rad")] for r in trace) if trace else None
                phase_ends.append({"phase": name, "first_tick": first + 1, "peak_forces_n": peaks,
                                   "lowest_flap_rad": lowest_flap, **rows[-1]})
    return {
        "phase_ends": phase_ends,
        "samples": embodiment.diagnostics,
        "contact_trace_columns": CONTACT_TRACE_COLUMNS,
        "contact_trace": embodiment.contact_trace,
        "skin_error": getattr(embodiment, "skin_error", None),
        "sound_error": getattr(embodiment, "sound_error", None),
        "phases": [list(row) for row in reference.phases] if reference is not None else None,
        "reference": json.loads(json.dumps(reference.log, default=float)) if reference is not None else None,
        "worst_ik_mm": round(1000 * reference.worst_ik, 3) if reference is not None else None,
        "pump_recoveries": getattr(reference, "recoveries", None),
        "key_dz_mm": round(1000 * reference.key_dz, 2) if reference is not None else None,
        "trims_mm": {arm: [round(1000 * float(v), 2) for v in reference.offset[arm]] for arm in fixture.ARMS}
        if reference is not None else None,
    }


def model_numbers():
    return {"flap_torque_nm": round(L.FLAP_TORQUE_NM, 4), "top_edge_push_n": round(L.TOP_EDGE_PUSH_N, 2),
            "feeder_l_per_rad": round(1000 * L.FEEDER_M3_PER_RAD, 3), "reservoir_l": 1000 * L.RESERVOIR_M3,
            "pressure_pa": [L.EMPTY_PA, L.FULL_PA], "reed_flow_l_s": 1000 * L.REED_FLOW_M3S,
            "full_reservoir_one_reed_s": round(Air(volume=L.RESERVOIR_M3).seconds_left(1), 2),
            "key_speaks_mm": 1000 * L.KEY_SPEAKS, "key_travel_mm": 1000 * L.KEY_TRAVEL, "mass_kg": L.MASS}


def failure_record(args, embodiment, policy, error):
    failure = {"policy": args.policy, "failed": True, "error": error,
               "executed_steps": embodiment.tick, "final_grade": embodiment.final_grade,
               "peak_interarm_contact_n": getattr(embodiment, "peak_interarm_n", None),
               "state_after_reset": getattr(embodiment, "state_after_reset", None),
               "model_numbers": model_numbers()}
    try:
        failure["diagnostics"] = diagnostics(embodiment, getattr(policy, "reference", None))
    except Exception as problem:      # never hide the original failure
        failure["diagnostics_error"] = repr(problem)
    (args.out / "RESULT.json").write_text(json.dumps(failure, indent=1, default=str) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("policy", choices=["scripted", "dream"])
    parser.add_argument("--out", required=True, type=Path)
    args = parser.parse_args()
    start = time.monotonic()
    args.out.mkdir(parents=True, exist_ok=False)
    try:
        yam_path = ensure_yam_arena()
    except ModuleNotFoundError:
        (args.out / "RESULT.json").write_text(json.dumps(
            {"policy": args.policy, "failed": True, "error": traceback.format_exc(limit=4),
             "pythonpath": os.environ.get("PYTHONPATH"), "yam_upstream": os.environ.get("YAM_UPSTREAM"),
             "arena_dir": os.environ.get("ARENA_DIR")}, indent=1) + "\n")
        raise
    from .adapter import make_embodiment, make_scripted

    embodiment = make_embodiment(media_dir=args.out / "media")
    audit = DreamAudit(args.out) if args.policy == "dream" else None
    policy, client = live_dream(audit) if audit is not None else (make_scripted(), None)
    result = None
    try:
        try:
            with tempfile.TemporaryDirectory(prefix="harmonium-frames-") as tmp:
                sinks = [JsonLogSink(tmp), audit] if audit is not None else None
                result = experiment(embodiment, policy, Path(tmp), sinks=sinks)
                for path in Path(tmp).glob("*.json"):
                    (args.out / path.name).write_bytes(path.read_bytes())
        except BaseException:
            # Failure record first, so a crashed trial still tells us how far it got.
            failure_record(args, embodiment, policy, traceback.format_exc(limit=12))
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
        failure_record(args, embodiment, policy, str(detail))
        raise RuntimeError(f"Inspect Robots trial did not complete: {detail}")
    package = Path(__file__).parent
    reference = getattr(policy, "reference", None)
    status = {
        "policy": args.policy, "model": MODEL if args.policy == "dream" else None,
        "score": score, "scorer": SCORER, "control_hz": fixture.CONTROL_HZ,
        "step_limit": fixture.MAX_STEPS, "executed_steps": embodiment.tick,
        "elapsed_s": round(time.monotonic() - start, 2),
        "scene_sha256": hashlib.sha256(Path(fixture.__file__).read_bytes()).hexdigest(),
        "asset_sha256": hashlib.sha256(fixture.USD.read_bytes()).hexdigest(),
        "layout_sha256": hashlib.sha256((package / "layout.py").read_bytes()).hexdigest(),
        "air_sha256": hashlib.sha256((package / "air.py").read_bytes()).hexdigest(),
        "grader_sha256": hashlib.sha256((package / "grading.py").read_bytes()).hexdigest(),
        "inspect_robots": version("inspect-robots"),
        "yam_arena": version("yam-arena"),
        "yam_arena_path_added": yam_path,
        "seed": 0, "observation_state_keys": ["joint_pos"], "observation_cameras": ["top_cam", "left_cam", "right_cam"],
        "policy_inputs_camera_size": [320, 176], "wall_time_limit_is_external": True,
        "scripted_demo_reads_simulator_state": args.policy == "scripted",
        "dream_chunk_anchor": TRANSITION if audit is not None else None,
        "dream_action_requests": client.calls if client is not None else 0,
        # Diagnostics only, none of them part of the score. The first is read by the batch harness.
        "peak_interarm_contact_n": embodiment.peak_interarm_n,
        "peak_contact_n": {k: round(v, 2) for k, v in embodiment.peak_forces.items()},
        "state_after_reset": embodiment.state_after_reset,
        "model_numbers": model_numbers(),
        "air": {"fed_l": round(1000 * embodiment.air.fed_m3, 3), "vented_l": round(1000 * embodiment.air.vented_m3, 3),
                "strokes": embodiment.air.strokes},
        "final_grade": embodiment.final_grade,
        "termination_reason": scene.termination_reasons[0] if scene.termination_reasons else None,
    }
    print(json.dumps(status, default=float), flush=True)
    status["diagnostics"] = diagnostics(embodiment, reference)
    (args.out / "RESULT.json").write_text(json.dumps(status, indent=1, default=float) + "\n")
    return int(args.policy == "scripted" and score != 1.)


if __name__ == "__main__":
    code = main()
    if code:
        # The result and videos are already closed. An unsuccessful Isaac trial can hang in Kit's exit
        # callbacks; return the recorded failure to the batch gate instead of starting a model trial.
        os._exit(code)
