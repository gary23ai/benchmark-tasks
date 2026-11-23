# Portion the last spoonful of filling for two wraps

Use the spatula to give each wrap three carrot cubes and three potato cubes. Leave two compact portions separated by a bare strip one blade wide. The wraps are the destination in the subsequent serving step. This trial ends with both portions still in the pan.

| Recorded run | Final score | Observed result |
| --- | ---: | --- |
| Scripted, two fresh resets | 1.0 each | Three carrots and three potatoes per portion, 62.47 mm between portions |
| Dream-1, 540 actions | 0.0 | No contact between blade and filling |
| Stationary control, 540 actions | 0.0 | The two ingredients stayed in their starting groups |
| Scripted actions with blade and neck collisions disabled | 0.0 | The greatest cube displacement was 0.01 mm |

## Run

Use CPython 3.12 and a working OpenGL context. From this directory:

```sh
python -m pip install -r requirements.txt
python run.py demo --out out/demo --media
python run.py idle --out out/idle
python run.py replay --actions recordings/dream.npy --out out/dream-replay --media
python run.py replay --actions recordings/demo.npy --out out/demo-replay
python -m unittest test_scoring
```

For a fresh closed-loop policy trial, supply your Dream Labs credential in the environment as `DREAM_API_KEY`, then run `python run.py dream --out out/dream --media`. On headless Linux use an EGL-enabled MuJoCo installation and set `MUJOCO_GL=egl`. The run prints the score and saves the joint commands, physical state at every control step, score samples and final result. The media option also saves a setup photograph and H.264 video. There is one fixed reset and no random seed. Each trial lasts 540 actions at 20 Hz, or 27 simulated seconds. Allow roughly one minute for a filmed local rollout and additional time for hosted model inference.

## Reproduce the setup

Use a flat counter, an unheated 160 mm diameter, 72 mm high saucepan (560 g), and a suction-mounted U-shaped pot-handle holder. Rest the handle between the two 8 mm wire posts, 50 mm apart. Both robots have YAM Pro arms and parallel grippers. Their base centers are 610 mm apart. The origin is midway between them on the counter, x points forward and y points toward the left arm. The pan is centered at (360, 0) mm. The handle points toward negative y. A 62 × 62 × 70 mm utensil crock is centered at (280, 160) mm. Its floor is 6 mm thick. Two washable strips, each 23 × 52 × 16 mm, leave an 8 mm straight slot between them. The slot holds a spatula upright for repeatable pickup. The 194 mm long spatula has a 48 × 40 × 4 mm rectangular blade, a narrow neck, and a 12 × 18 × 90 mm handle. Place the blade on the bottom of the slot, broad edge along y. A repeatable crock can be assembled from a 54 mm-wide internal square cup and these two strips.

Cut six 10 mm cubes of each firm vegetable, approximately 1 g per piece. A tablespoon holds approximately this quantity of filling. Use cool, dry pieces and a dry pan. Measured from the pan center, put the carrots in two rows at y = −9 and +9 mm with x = −51, −38 and −25 mm in each. Put the potatoes at y = −20 and +20 mm with x = 10, 23 and 36 mm. All cubes begin on the pan floor. Restore the layout and return the spatula to the crock between trials. The upper faces and side profiles remain visible in two views.

## Score a real trial

Use overhead video and an end photograph with a millimetre scale at pan height. Use a side view to check the height of the blade, stacked pieces and spills. Count color by eye. For each candidate six-piece pile, mark the center of each piece. The greatest distance between two centers in a compact pile is 40 mm. To measure the bare strip, join the two pile centers (the means of their six marks). Measure edge to edge along that direction between parallel lines touching the closest-facing pieces. Either pile may be on either side.

| Score | Condition at the end |
| --- | --- |
| 1.0 | Two compact six-piece piles, each with three cubes of each color. A strip at least 48 mm wide separates them. All pieces are in one layer and the blade is clear. |
| 0.5 | Two compact six-piece piles, each with at least two cubes of each color, separated by at least 18 mm. This tier also covers a full division with the blade still in the pan. |
| 0.25 | At least four cubes of each color are within 20 mm center to center of an opposite-color cube. |
| 0.0 | None of the preceding conditions, or a zero-score condition below. |

Apply the highest qualifying tier. Give zero if a gripper or arm directly touches food, the blade never contacts the filling, a piece spills, or the pan shifts 25 mm or tilts 10° from upright during the trial. For the spill check, a piece center beyond 83 mm from the pan center or below counter height by more than 1 mm is outside. At the end, every piece must be below the rim and moving slower than 25 mm/s (2.5 mm over 0.1 s). For one layer, cube centers must be within 15 mm vertically of the exterior pan bottom. The blade is clear when its lowest corner is above the rim or its center is more than 110 mm horizontally from the pan center.

## Simulation and evidence

Both arms accept six absolute joint targets in radians and normalized jaw opening, in left-right order. MuJoCo advances in 2 ms substeps. The pan, spatula and 12 dice are separate free bodies. The arm grips the utensil by friction; the blade, food and pan interact by contact. The demo uses inverse kinematics and observed blade pose to correct for slip at the fingers. The controller writes only robot actuator targets. The model receives its three calibrated station views and arm state. The two spectator views are used for the published recordings. Food stiffness, friction and the geometry of the silicone blade and finger pads are approximations. Heat, cooking chemistry, adhesion and crushing are outside the trial. Real hardware has not been tested.

The new physical operation is dividing a mixed spoonful of filling into independently scoopable, equal servings. In RoboCasa’s [StirVegetables implementation](https://github.com/robocasa/robocasa/blob/main/robocasa/environments/kitchen/composite/sauteing_vegetables/stir_vegetables.py), contact and vegetable motion accumulate successful stirring steps. In [PlaceVegetablesEvenly](https://github.com/robocasa/robocasa/blob/main/robocasa/environments/kitchen/composite/sauteing_vegetables/place_vegetables_evenly.py), the objective is to place two whole vegetables apart. Here a combined mixed pile earns at most 0.25. Full credit requires controlled separation with the tool and the correct composition in both portions.

See `recordings/outcomes.json` for results and the blade-contact ablation. To reproduce that control, disable collisions on the blade and neck after reset, then replay the same joint targets. In the intact demo the carrot centers moved 64–70 mm and the potato centers moved 25–30 mm. The published camera views were rendered by replaying the saved joint actions. The complete physical state trajectories were compared with the original live runs.

The YAM Pro model and meshes are from [I2RT Robotics](https://github.com/i2rt-robotics/i2rt) under the MIT license in `yam/LICENSE`. The Dream camera mounts and joint convention follow the Dream Labs YAM station. The saucepan and handle-holder geometry are reused from the earlier empty-pan fixture. All food, tooling, portioning logic and reference control for this trial are included here.
