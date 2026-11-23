# Open a flat shopping tote, pack two groceries and lift it by both handles

Put the box of crackers and the pack of toilet rolls into the bag so that both rest on the bag's base and the bag stands upright on its own. Then lift the bag by its two handles, set it back down, let go and move both arms away.

At a grocery checkout a reusable tote can arrive folded flat. Open its walls, pack the small carton and the bulky bundle, and lift the loaded bag by both handles. O*NET lists bagging merchandise among cashier duties (41-2011.00).

## Real-trial grading

Scored on the state at the end of the trial (the run limit, or as soon as the 1.0 state has lasted 2 seconds). Nothing is scored for the order in which the items go in or for how they are held. Definitions. An item rests on the base: its lowest point is within 3 mm of the bag's base board, it stands on one of its faces (that face within 10 degrees of the board) and, seen from above, it lies wholly within the board's outline; an item lying on a folded-in wall is higher than that and does not count. The bag stands upright: its base board is on the table (lowest point within 5 mm of it), level within 10 degrees, its middle within 50 mm of its tape marks, and the top edge of each of its four walls is at least 144 mm above the table (80 % of 180). The lift is done once, for 2 seconds without a break, the base board's lowest point is at least 50 mm above the table, both handle grips are pulled at least 30 mm above the bag's top edge and both items remain in the bag. In the lift, each item’s centre stays over the base outline and its lower edge stays below the top of the bag. Check that neither exits on the video. Check the 3 mm base gap only after set-down. 1.0: both items rest on the base, the bag stands upright, the lift is done, and for the last 2 seconds no part of the robot touches the bag or an item and both grippers' centre points are at least 50 mm from the bag (its outline up to its top edge, and its handle grips). 0.75: both items rest on the base and the bag stands upright, but the lift is not done or the robot has not let go. 0.5: the pack of toilet rolls rests on the base and the bag stands upright (the box is anywhere). 0.25: only the box of crackers rests on the base and the bag stands upright. 0: anything else, including a bag that at the end is held up, lying over, slumped or more than 50 mm off its marks. Also 0, whatever happens later, once an item has fallen off the worktable, an item has come out of the bag while the bag was off the table, or a corner seam of the bag has been pulled more than 30 mm apart (torn). On a real trial: film from the side; at the end hold a ruler against each wall, photograph the bag from above and from the side, and try a 3 mm card under each item inside the bag. Possible scores: 0, 0.25, 0.5, 0.75, 1.0.

The physical reference is a reusable gusseted checkout tote with a flat base insert. Measure wall, base and handle dimensions against the setup. Film the lift from the side with a 50 mm height reference. Use the end view or an overhead view to confirm the two groceries remain in the bag. Check their seating only after set-down. [Cashier tasks](https://www.onetonline.org/link/summary/41-2011.00).

## Recorded evidence

In the full-score recording both items stayed seated throughout the lift, meeting the stronger original lift test; both remained in the bag and passed the final seating check. Dream ended with the bag collapsed and neither item on its base, which scores zero regardless of lift history. The scene, instruction, actions and videos are unchanged.

## Run one trial

Use Python 3.12, Isaac Sim 6.1, Isaac Lab-Arena 0.3.1, Inspect Robots 1.0.0,
inspect-robots-yam 0.38.0 and yam-arena 0.1.0 on Linux with an NVIDIA GPU.
The stock YAM station, arm meshes and cameras are runtime dependencies; they are not bundled.
Prepare the [pinned YAM Lab-Arena runtime](https://github.com/kairosiann/yam-isaac-lab-arena/tree/bd62771e60320433893bd1d2df0fc79b5ddcd6d1) and install its Dream client adapter at commit b5e936a201cec66c54c297e522c920d98445e339. Install this folder in that environment:

```sh
uv pip install --no-deps .
python -m rdb_bag_load.run scripted --out out/scripted
python -m rdb_bag_load.run dream --out out/dream
```

The Dream run requires a Dream Labs credential configured through its client. No credential is included.
Both runs use Inspect Robots, the same scene and scorer, seed 0 and zero object jitter.
Run limit: 2,400 steps (2 minutes at 20 Hz). Use the default layout; the controls in the recorded qualification summary are not the submitted trial.

| Recorded run | Score | Steps | Wall time |
| --- | --- | --- | --- |
| Scripted physical-feasibility reference | 1.0 | 1,617 | 879 s |
| Dream-1 | 0.0 | 2,400 | 998 s |

Dream-1 receives only 320 x 176 RGB from the overhead and two wrist cameras, plus joint and gripper positions.
The scripted reference uses simulator object state for feedback; it is not an observation-matched model baseline
or a demonstration of perception from RGB. It commands robot joints and grippers without writing object poses.
The review-only side camera is never a model input.

Each run writes `RESULT.json`, `media/setup.png`, `media/top_cam.mp4`, `media/side_cam.mp4`
and `media/grader-trace.jsonl`. Read `score` in `RESULT.json` for the final score.
The scripted command exits successfully only at full score. Startup and model latency add wall time.



## Third-party assets and licenses

Task code and the procedural scene: MIT (LICENSE). No manufacturer CAD, images or meshes are included; the box
and pack carry drawn-on patches of plain colour, no brand. Runtime dependencies keep their own licenses: Isaac
Sim and Isaac Lab-Arena (NVIDIA), Inspect Robots, inspect-robots-yam, yam-arena, the Dream Labs client.

