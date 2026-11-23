"""Run a complete trial. Install requirements.txt before running."""

import argparse
import base64
from contextlib import ExitStack
import io
import json
from pathlib import Path
import time

import imageio_ffmpeg
import mujoco as mj
import numpy as np
from PIL import Image
from scene import LIMIT
from trial import Trial
from demo import targets

PROMPT = (
    "Use the spatula to split the last spoonful of carrot and potato filling between two wraps. "
    "Leave three pieces of each vegetable in each compact pile, with a spatula-width of bare pan "
    "between the piles. Keep all the filling in the pan and lift the spatula clear."
)
CHECKPOINT = "dreamlabs/dream-1-beta"


def dream_actions(env, renderer):
    import dream

    client = dream.DreamClient()
    try:
        client.policy(
            dict(
                checkpoint=CHECKPOINT,
                strict=False,
                timeout_ms=45000,
                action_hz=0.001,
                obs_window=9,
                obs_fps=5.0,
                _record=False,
            )
        )

        def observe(stamp):
            state = {
                "robot.arm": "yam",
                "robot.exec_head": "joint",
                "robot.max_joint_vel_rad_s": 0.0,
                "robot.gripper_open": 1.0,
                "prompt": PROMPT,
            }
            for side, arm in zip(("l", "r"), env.action()):
                state[f"state.arm_{side}.joint_pos"] = arm[:6].tolist()
                state[f"state.arm_{side}.gripper_pos"] = [float(np.clip(arm[6], 0, 1))]
            frames = {}
            for name in ("top", "left", "right"):
                renderer.update_scene(env.data, camera=name)
                pixels = renderer.render()[60:420]
                img = Image.fromarray(pixels).resize(
                    (320, 176), Image.Resampling.BILINEAR
                )
                stream = io.BytesIO()
                img.save(stream, format="JPEG", quality=85)
                frames["image_window." + name] = base64.b64encode(
                    stream.getvalue()
                ).decode()
            client.set_observation(frames, state, timestamp=stamp)

        stamp = time.monotonic()
        for i in range(9):
            observe(stamp - (8 - i) * 0.2)
        last = None
        while env.step_count < LIMIT:
            commanded = (
                None
                if last is None
                else {"q_l_chunk": last[0].tolist(), "q_r_chunk": last[1].tolist()}
            )
            reply = client.get_action(commanded=commanded)
            arms = [
                np.asarray(reply[k], dtype=float) for k in ("q_l_chunk", "q_r_chunk")
            ]
            if (
                arms[0].shape != arms[1].shape
                or arms[0].ndim != 2
                or arms[0].shape[1] != 7
                or not len(arms[0])
            ):
                raise ValueError("Invalid Dream action chunk")
            print(
                f"Dream returned {len(arms[0])} action rows at step {env.step_count}",
                flush=True,
            )
            for l, r in zip(arms[0][:24], arms[1][:24]):
                last = np.stack([l, r])
                yield last
                if env.step_count % 4 == 0:
                    observe(stamp + env.step_count / 20)
                if env.step_count >= LIMIT:
                    break
    finally:
        client.close()


def video_frame(env, renderer):
    frames = []
    for name in ("overview", "pan_view"):
        renderer.update_scene(env.data, camera=name)
        frames.append(renderer.render().copy())
    return np.concatenate(frames, axis=1)


def run(args):
    env = Trial()
    directory = args.out
    directory.mkdir(exist_ok=True, parents=True)
    with ExitStack() as resources:
        renderer = (
            resources.enter_context(mj.Renderer(env.model, height=384, width=512))
            if args.media
            else None
        )
        if args.media:
            initial = video_frame(env, renderer)
            Image.fromarray(initial).save(directory / "setup.png")
            writer = imageio_ffmpeg.write_frames(
                str(directory / (args.policy + ".mp4")),
                (1024, 384),
                fps=10,
                codec="libx264",
                output_params=["-crf", "18", "-movflags", "+faststart"],
                ffmpeg_log_level="error",
            )
            writer.send(None)
            resources.callback(writer.close)
            writer.send(initial)
        if args.policy == "demo":
            commands = targets(env)
        elif args.policy == "idle":
            commands = (env.action().copy() for _ in range(LIMIT))
        elif args.policy == "replay":
            commands = iter(np.load(args.actions, allow_pickle=False))
        else:
            sensor = resources.enter_context(
                mj.Renderer(env.model, height=480, width=640)
            )
            commands = dream_actions(env, sensor)
            resources.callback(commands.close)
        actions = []
        record = [env.data.qpos.copy()]
        checks = {0: env.score()}
        for a in commands:
            env.step(a)
            actions.append(np.array(a).copy())
            record.append(env.data.qpos.copy())
            if args.media and env.step_count % 2 == 0:
                writer.send(video_frame(env, renderer))
            if env.step_count % 10 == 0:
                checks[env.step_count] = env.score()
        if env.step_count != LIMIT:
            raise RuntimeError("Incomplete trial")
        np.save(directory / "actions.npy", np.array(actions), allow_pickle=False)
        np.savez_compressed(directory / "trajectory.npz", qpos=np.array(record))
        result = dict(policy=args.policy, steps=env.step_count, **env.score())
        if args.policy == "dream":
            result["model"] = CHECKPOINT
        (directory / "result.json").write_text(json.dumps(result, indent=2))
        (directory / "progress.json").write_text(json.dumps(checks, indent=2))
        print(json.dumps(result), flush=True)


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("policy", choices=("demo", "idle", "dream", "replay"))
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--actions", type=Path, help="Action array for replay")
    p.add_argument("--media", action="store_true")
    run(p.parse_args())
