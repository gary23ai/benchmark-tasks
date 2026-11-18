# Clamp a glass vacuum filter holder

A 47 mm glass vacuum filter holder stands on its 1 L filtering flask with a new membrane on the frit. Its funnel
stands on a wipe beside it and its aluminium spring clamp lies shut on the bench. The robot sets the funnel
square on the membrane with one hand and keeps holding it down. With the other hand it squeezes the clamp open
by its handles, pushes it round both ground-glass flanges and lets it close. Then it lets go and moves back.

Everything is at true size from the maker's figures. The funnel and clamp are held by finger contact only, and
the clamp stays on the flanges because of its spring and the V of its jaws. Only the creasing of the membrane is
a stated rule.

The scripted demonstration scores 1.0 in the simulator (993 steps). Dream-1 scores 0.0: it never picked up the
funnel or the clamp. Both are from run 4, on the final scene and scorer; runs 1 to 3 are kept under "Simulator
runs".

| Path | What it is |
| --- | --- |
| `task.json` | the task card |
| `rdb_filter_clamp/layout.py` | every dimension of flask, base, funnel and clamp, each marked PUBLISHED or ESTIMATE; the geometry the scorer uses |
| `rdb_filter_clamp/filter_clamp.usda` | the scene asset, generated from `layout.py` by `tools/build_assets.py` |
| `rdb_filter_clamp/fixed_scene.py` | where things stand, instruction, run limit, review camera |
| `rdb_filter_clamp/grading.py` | the scorer (NumPy only) |
| `rdb_filter_clamp/adapter.py` | adds the parts to the stock Inspect Robots YAM station, feeds the scorer, logs diagnostics |
| `rdb_filter_clamp/scripted.py` | the scripted demonstration (closed loop on the measured part poses) |
| `rdb_filter_clamp/run.py` | runs one trial and writes the result, videos, scorer trace and diagnostics |
| `tests/`, `tools/` | unit tests, the dry run without the simulator, the asset generator |

## The holder, and where its numbers come from

| Quantity | Value used | Source | Status |
| --- | --- | --- | --- |
| Holder | MilliporeSigma XX1014700, 47 mm, borosilicate funnel and fritted base, anodised aluminium clamp, size 8 silicone stopper with a 15 mm hole | MilliporeSigma user guide "47 mm Glass Vacuum Filter Holders" (parts list and specifications) | read in the document |
| Assembled height, holder diameter | 23.5 cm, 7.6 cm | same guide ("Approximate dimensions"); the Fisher Scientific listing of XX1014700 gives the same | read in the document and on the page |
| Funnel | 300 mL; effective filter area 9.6 cm2 | same guide | read in the document |
| Replacement parts | base and stopper XX1014702, clamp XX1014703, funnel XX1014704 | same guide | read in the document |
| How the clamp opens; the tipping risk | "Pinch clamp handle together and remove clamp from the flange"; "Vacuum should be applied slowly to avoid ... accidentally tipping the holder" | Thermo Scientific Nalgene filter funnel with clamp, instruction sheet 8-0404-27 0110 | read in the document (a plastic holder with the same kind of clamp) |
| Flange thickness 6 mm each, bevel from r 35 to 38 mm over 3 mm | | no maker publishes it | estimate |
| Funnel: 64 mm across the body, 125 mm tall, 2.5 mm wall, neck and taper | chosen so that funnel, base and stem give the published 23.5 cm and about 300 mL | | estimate |
| Masses | funnel 0.20 kg; flask 0.55 kg; base and stopper 0.17 kg | glass volumes at 2.23 g/cm3 | estimate |
| Clamp | two jaws on a pin, ring 90 mm across and 18 mm tall, V faces on the line z = 41 mm - r, handles 30 mm long; 46 g | | estimate |
| Clamp spring | 0.30 N m/rad on each jaw, wound 40 degrees past shut; about 17 N at the handles when they are pressed together | no maker publishes an opening force; see below | estimate |
| Clamp opening | the jaws open 27 degrees each with the handles together; the mouth then clears the 76 mm flanges by 0.9 mm | | estimate |
| 1 L heavy-wall filtering flask | 125 mm across at the foot, rim 205 mm up, neck 46 mm | | estimate |
| Friction | glass 0.40 static, 0.35 dynamic (ground glass on a wet membrane); clamp 0.35/0.30; stopper 0.9 | | estimate |

**Opening force (the number that killed two sibling ideas).** No maker or seller we could open publishes the
clamp's opening force, and we had no clamp to measure. Two things bound it. The clamp is made to be opened
between thumb and finger many times a day (the Nalgene sheet: "pinch clamp handle together"). Published pinch
norms (Mathiowetz et al. 1985, Arch Phys Med Rehabil 66:69) put adult tip and key pinch at roughly 50 to 115 N,
so a clamp meant for routine use must sit well below that. We model 17 N at the handles. The program guide gives about 40 N per YAM finger; run 4 measured 26.7 N per fingertip on the clamp. The actual clamp opening force must be measured before a hardware trial. A clamp requiring more than the hardware limit would not
open. **Measure a real XX1014703 with a spring balance before a bench trial** and put the spring into `layout.py`.

No product image or maker's drawing is included in this folder.

Station coordinates (mm): holder axis at (330, -30), joint plane (top of the base's flange) 288 above the
table, funnel top 413; funnel standing on its wipe at (270, 175); clamp ring centre at (300, -215), lying shut.

## Physics probe: does the second hand matter?

Before building we checked whether an unheld funnel really moves when the clamp goes on. We used MuJoCo 3.15 as a
stand-in, since the target simulator is PhysX on a GPU we do not have locally. The scene had the same glass and
clamp geometry and masses, a hand holding the clamp by its knuckle, and the jaws let close over 1 s. "Held"
means a 400 N/m compliant hold that presses the funnel down with 3 N, which is about the arm's compliance.
"Slide" is the funnel's flange against the base's, in the holder's frame. The results (workdir files, not
bundled):

| Spring | Funnel | Clamp placed off the axis by | Funnel slide (max / end) | Clamp closed |
| --- | --- | --- | --- | --- |
| 0.30 N m/rad, 40 degrees preload (the model) | free | 0 to 6 mm, or pushed 4 to 8 mm past | 17 to 285 mm / 14 to 44 mm: thrown off the base every time | no (jaws stuck at 22 degrees on the displaced funnel) |
| same | held | 0 to 6 mm, or pushed 8 mm past | 0.01 to 0.37 mm | yes (1.4 degrees, both jaws on the bevels) |
| half the spring, 30 degrees preload | free | 0 / 3 / 6 mm | 0.07 / 0.5 to 0.8 / 2.4 mm | yes |
| same | held | 0 / 3 / 6 mm | 0.02 / 0.05 to 0.07 / 0.2 to 0.33 mm | yes |

So with the modelled spring the closing jaws throw a loose funnel off. With a weak spring a loose funnel is
shoved by about the clamp's misplacement, which the arm's compliance makes several millimetres. The held funnel
stays within 0.4 mm in every case. PhysX may differ, so the scorer also asks for the hold directly (a gripper on
the funnel's body all the while the clamp is round the joint and not yet shut). In PhysX the held funnel moved
0.4 mm when the clamp closed (runs 3 and 4). The control `DEMO_VARIANT=no_hold` lets go of the funnel before the clamp
is fitted; the scorer caps it at 0.6 whatever PhysX does to the funnel. It is not one of the recorded runs.

## Reach

Every waypoint was solved with the station's IK from eight seeds and checked along the whole path, with the
arms' collision hulls (an external clearance helper, not bundled; `--clearance` in the dry run).

- Left on the funnel: pads closing along x on two flats of the 64 mm body, 65 mm above its foot, fingers pointing
  -y and 30 degrees down. Joint margins: 23 degrees at the pick, 25 to 31 degrees holding it on the base. A level
  grip cannot reach the funnel standing on the bench, and a top-down grip cannot reach it on the flask (its top is
  41 cm up), so this pitched grip is used for both. The carry goes up and across through (310, 50) so that the
  funnel's foot is above the flange's height before it comes near the holder; a straight path would drag it over
  the flange.
- Right on the clamp: pads closing along y on the two handles, fingers pointing +x and 45 degrees down. Margins:
  11 degrees at the pick (fingertips about 1 mm above the bench), 21 degrees with the clamp on the joint, and 5.5
  degrees (2.9 with extra sag) at the point 75 mm behind the joint where the push starts. That point is as close
  as the start can be: nearer, and the open mouth already overlaps the flange.
- After the clamp is on, the right hand leaves sideways through two points, (-130, -60, +2) and (-140, -170, -38) mm
  from the joint, before it swings home (run 2 showed why: see "Simulator runs").
- Clearances in the dry run: arms at least 29 mm apart; right arm at least 30 mm from the funnel and, once it has
  let go, 21 mm from the fitted clamp; inverse kinematics error at most 0.1 mm. In run 4 the worst inverse
  kinematics error was 0.46 mm and no arm touched the other.

## Run and score a trial

Runtime: the RoboCurve YAM runtime with Python 3.12, Isaac Lab-Arena 0.3.1 on Isaac Sim 6.1 (Linux, NVIDIA RTX
GPU), Inspect Robots 1.0.0, inspect-robots-yam 0.38.0 and yam-arena 0.1.0
([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena)). Install this folder into that
environment with `uv pip install --no-deps .`. For the Dream-1 run, put a Dream Labs key where the Dream client
looks for it (`~/.dreamlabs/credentials`); no key is stored here.

```sh
python -m rdb_filter_clamp.run scripted --out out/scripted                    # the scripted demonstration
python -m rdb_filter_clamp.run dream --out out/dream                          # one Dream-1 trial
DEMO_VARIANT=no_hold python -m rdb_filter_clamp.run scripted --out out/nohold  # control: must score 0.6 or less
```

Each command runs one trial through Inspect Robots with the scorer `filter_clamp_score`, seed 0, zero object
jitter and a limit of 2,400 actions at 20 Hz (120 simulated seconds). The trial ends early once 1.0 has held for
1 s. `run.py scripted` exits 0 only if the score is 1.0. RESULT.json is written even if the trial fails with an
error. Measured on one data-centre GPU (run 4): the scripted demonstration took 250 s of wall time for its 993
steps; the Dream-1 trial took 658 s for the full 2,400 steps, with 90 requests to the model.

Each run writes these files:
- `RESULT.json`
- `media/setup.png`: the review camera's first frame
- `media/setup_top_cam.png`: the overhead camera's first frame
- `media/top_cam.mp4`: the overhead policy camera, recorded as rendered at 640 x 360
- `media/side_cam.mp4`: a review-only camera beyond the table's far side, 1280 x 704, never given to a policy
- `media/grader-trace.jsonl`

Policies receive 320 x 176 RGB from the overhead and the two wrist cameras plus joint and gripper positions, and
nothing else.

**What the policy cameras see** (from the rendered first frame of runs 3 and 4). The overhead camera has everything in
view from the first frame:
- the flask with the stopper and base on it, about 23 pixels wide at 320 x 176;
- the white membrane on the frit, seen from above, about 16 pixels across;
- the funnel standing on its wipe, about 18 pixels;
- the blue clamp lying shut with its two handles, about 18 pixels.

At that size the membrane and the flanges are visible as shapes, but a 1 mm offset is not; the wrist cameras
come close enough to show the joint. An earlier pinhole estimate in this README put the joint above the overhead
image's edge; the rendered frame shows that it is not. The review camera's frame is used as the setup image
because it shows the glassware larger.

## Scoring

The scorer is `rdb_filter_clamp/grading.py`; `task.json` states the same rules in words.

- **0.2 seated:** the funnel's ground face on the base's (lowest point within 1 mm, axes within 2.5 degrees), off centre.
- **0.4 centred:** axis within 1.5 mm of the base's.
- **0.6 clamped:** both jaws shut on the flanges' bevels (each jaw between -0.5 and 7 degrees open; shut on nothing
  it stops at -2), the ring centred within 3 mm, level within 5 degrees, within 2.5 mm of the joint's height.
- **0.8 careful:** the membrane is still on its marks at the end: no more than 1 mm from where it was laid. The
  membrane is drawn, not simulated. The stated rule carries it with the funnel's ground face whenever the funnel
  slides while resting on the base (faces within 0.3 mm); it stays wherever it is left, so lifting the funnel off
  and setting it down elsewhere moves nothing. This is an end-state check, as on the bench. What it gives up: a
  drag that later returns to its marks is not penalised (the largest offset at any time is logged as
  `membrane_peak_mm`, for information only). A gripper also held the funnel's body
  (fingertip centre within 15 mm of its axis, pads stopped at 56 to 68 mm on the 64 mm body) at every step while
  the clamp's ring was within 25 mm of the axis and 15 mm of the joint and not yet closed on it. The watch ends
  once the clamp has stayed closed for 0.5 s.
- **1.0 done:** in addition every fingertip centre at least 80 mm from the holder's axis (or 50 mm above the
  funnel's top), funnel and clamp moving less than 1 mm in the last second, flask within 3 degrees of upright,
  all for 1 s. The trial then ends.
- **Latched 0:** flask tipped past 10 degrees or pushed more than 3 cm, funnel tipped past 45 degrees, or clamp dropped.

Outcomes: nothing done, 0; funnel left on the wipe or held above the base, 0; funnel set down off centre, 0.2;
centred but no clamp, or a clamp left open or shut on nothing, 0.4; clamp on but the funnel unheld while it went
on, or the membrane left more than 1 mm off its marks, 0.6; done properly but still holding at the run limit, 0.8; done and stood back, 1.0;
flask knocked over at any time, 0.

### Scoring a real trial

Before the trial, draw two fine ink ticks across the membrane's edge onto the base's flange, one at the front
and one at the side. From the end photo, with a ruler: seated and square; off centre by how much (the two
flanges' edges line up within 1.5 mm); clamp closed round both flanges and level. From the video: a hand was on
the funnel while the clamp went on; then the robot let go and moved back. After the trial, take the clamp off
(press the handles) and lift the funnel straight up. Under a loupe against a millimetre scale, each tick on the
membrane must be within 1 mm of its tick on the flange, with no fold, for 0.8 and above. Then, not scored, refit
it and pour 50 mL of water in without vacuum, and watch the joint for a minute.

## Real-world build list

| Item | Details | Cost |
| --- | --- | --- |
| Glass filter holder | MilliporeSigma XX1014700 (Fisher list price seen: USD 502 to 584); the same pattern for less: Kimble 47 mm assembly (USD 328 seen), Chemglass CG-1420-01 (USD 358 seen); any 47 mm glass holder with 76 mm ground flanges and a spring clamp of this pattern will do | about USD 330 to 580 |
| 1 L filtering flask | heavy-wall, size 8 stopper neck (e.g. MilliporeSigma XX1014705) | about USD 60 to 150 |
| Spare funnel and clamp | XX1014704, XX1014703 (clamp alone about USD 40 to 100) | about USD 180 |
| Membranes | 47 mm, white, 0.45 um mixed cellulose ester, any maker, a box of 100 | about USD 60 |
| Spring balance | 0 to 50 N, to measure the clamp's opening force | about USD 15 |

Glass on a shared rig. Run 4 measured a peak 19.8 N per fingertip on the 2.5 mm glass wall and 26.7 N per fingertip on the aluminium handles. These are simulator measurements, not a glass-strength test; inspect the glass and use the rig owner’s approved pad protection before hardware operation. Run the trial dry, without
vacuum and without solvent. Put a rubber mat under the flask and keep a spare funnel. Pad covers can be fitted
if the rig's owner prefers. Reset takes about a minute (see `reset_notes`).

## The scripted demonstration

`scripted.py`. The demonstration reads the robot's joint and gripper readings and, as the review rules allow for a
scripted demonstration, simulator state: the poses of the holder, the funnel and the clamp and the jaws'
angles. It changes nothing in the simulation.

1. **Funnel.** The left hand goes to 45 mm back along its fingers from the funnel's body, corrects for sag from its
   joint readings, closes to about 20 N a finger and checks the measured opening (retried once 4 mm lower).
2. **Seat it.** Lift 135 mm, carry up and across, servo the funnel's measured foot to 22 mm over the joint, then to
   5 mm over it within 0.5 mm while turning the hand until the funnel's axis is within 0.8 degrees of the holder's.
   Lower straight down at 8 mm/s with no sideways correction until it touches. Loosen the pinch to about 6 N a pad,
   press 8 mm further so the ground faces square up, then grip again at about 20 N. If it ends more than 1.1 mm
   off centre or more than 2.2 degrees out of square, lift it and set it down again (up to three placings; the
   last one is left down whatever it reads, never hovering over the base).
   Every servo here is move-and-settle: a moving goal is followed feed-forward, and 60 % of the measured error
   is corrected every 4th step (see "Simulator runs").
3. **Clamp.** The right hand comes 50 mm back along its fingers to the handles, then closes fully: the handles
   meet and the jaws open 27 degrees. The jaws' measured angle is checked (retried once).
4. **Fit it.** Lift 130 mm, servo the clamp's measured ring centre to 75 mm behind the joint at the joint's
   height (within 0.7 mm), then push it on at 15 mm/s by feedback on its measured pose until it is centred on the
   axis within 0.8 mm. Open the fingers over 2 s so that the jaws close at the fingers' pace. Open them wide
   (85 mm), back 50 mm along -x and 8 mm up, go out sideways to the right through two points, then home.
5. **Let go.** The left hand opens, backs 60 mm along its fingers, goes home; the scorer's 1 s hands-off ends the trial.

## Tests and dry runs

```sh
python -m pytest -q tests                      # set YAM_ARENA=<yam-isaac-lab-arena checkout> for the demonstration tests
python tools/dryrun_demo.py --yam-arena <yam-isaac-lab-arena checkout> [--clearance <folder with clearance.py>]
```

The tests cover every tier and the cheap strategies. These include the clamp fitted with nobody holding the
funnel, the funnel pushed across the membrane, a clamp left open or shut on nothing or too high, a clamp
appearing on the joint without being fitted, a robot still holding at the end, unreadable grippers, the flask
knocked over or pushed, the clamp dropped and a funnel lifted and reset. They also check the scene file against
`layout.py` (bodies, masses, hinges, springs and stops, convex hulls of at most 64 vertices, glass colliders
hidden, display parts without collision), the maker's sizes, the clamp's geometry (open mouth clears the
flanges, the rest position does not, V faces on the bevels, handle force 15 to 25 N), `task.json` against the
code, and the demonstration in the stand-in world.

`tools/kinematic_world.py` has no contact physics (see its docstring). Local results of `dryrun_demo.py`:

- nominal: 1.0 in 932 steps (46.6 s); worst joint margin 6.9 degrees; inverse-kinematics error 0.1 mm; funnel
  membrane 0.18 mm off its marks at the end; the hand on the funnel all 88 steps the clamp was being fitted.
- more sag, slower arm, funnel 2 mm off in the hand: 1.0 in 1,011 steps. No sag, quick arm, off the other way:
  1.0 in 868 steps. Part poses handed to the demonstration 3 steps late with a slow, sagging arm: 1.0 in 1,127
  steps (worst joint margin 1.5 degrees at the clamp's push start); with a normal arm 1.0 in 1,020 steps.
- control `no_hold`: 0.6 (clamp on, but fitted with nobody holding the funnel). Idle robot: 0.


## Reading a run's diagnostics

`RESULT.json` holds the score, file hashes, `after_reset` poses, `diagnostics.samples` (every 5 steps: every
scorer field, poses, both TCPs, openings, where the funnel sits in the left hand and the clamp in the right, every
arm-link and fingertip force on each body) and `diagnostics.contact_trace`. The trace has every step while the
clamp is near the joint and every 2nd while the funnel sits unclamped: jaw angles, funnel and clamp positions,
fingertip forces on funnel and jaws, and each jaw's and the knuckle's force on the funnel and on the base, each
from its own contact view. It also holds `peak_contacts_n`, the demonstration's phases, grasps, seatings,
fittings and servo log, and `peak_interarm_contact_n`. What a healthy scripted run shows (run 4 matched every item):

- **Reset:** holder, funnel and clamp within 4 mm of their marks, upright; `jaw_open_deg` near -2; masses holder
  0.72, funnel 0.20, jaws 0.02, knuckle 0.006; `clamp_dof_names` hingeL and hingeR.
- **Funnel grip** (`reference.grasps[0]`): `holding` true, `opening_mm` about 63 to 64, left fingertip forces
  about 20 N each, funnel in the hand about (0, 56, 32) mm.
- **Seating** (`reference.seatings`): offset under 1 mm, gap near 0, tilt under 2.2 degrees (`tilt_loose_deg`
  under 1); `funnel_on_base_n` about 1 N or more while pressed (run 4: 1.3 N). Servo rows `align` converge without
  the 2 to 4 mm swings of run 1.
- **Clamp grip:** `squeezed` true with both jaws at 26 to 27 degrees, `opening_mm` about 11; right fingertip
  forces on the jaws about 26 N each at the stop (run 1: 26.5 N); clamp tilt in the hand under 3 degrees.
- **Fitting** (`reference.fittings`, `contact_trace`): before release, ring within 1 mm of the axis and of the
  joint's height; jaw contact on the glass only after release. After release the jaws settle at about 0.5 to 3
  degrees with forces on funnel and base, `membrane_offset_mm` under 1 (run 4: 0.57), `held` true on every traced step,
  no arm force on the clamp after release, score 0.8 and then 1.0 when both hands are home.
- **Departures and what they mean:** jaws at -2 after release means the clamp missed the flanges (look at
  `clamp_height_mm`). Jaws stuck near 20 degrees means a jaw rides on a flange edge (the ring was too high or
  low). `membrane_offset_mm` over 1 means the push or the closing shoved the funnel against the hand. A funnel opening
  near 58 mm with the funnel tilting in the hand means the pinch is too light. Clamp tilt growing in the hand
  means the handle grip turns about the line between the pads.

What remains an estimate: the clamp's spring and opening (to be measured on a real clamp), the flange bevel, the
funnel's details and the glass friction. The simulator result is one trial (seed 0, no jitter). No real bench
has been built.

## Simulator runs

**Run 4 (final scene and scorer): demo 1.0, Dream-1 0.0.** The scene, layout, asset and scorer hashes in both
RESULT.json files match the files in this folder.

| Run | Score | Steps | What happened |
| --- | --- | --- | --- |
| Scripted demonstration | 1.0 | 993 of 2,400 (49.7 s simulated) | see the list below |
| Dream-1 (`dreamlabs/dream-1-beta`) | 0.0 | 2,400 of 2,400 | see the paragraph below |

What the scripted demonstration did:
- **Funnel grip:** one grip, opening 63.9 mm, 19.8 N peak on a fingertip.
- **Seating:** one placing, 0.88 mm off centre and 0.33 degrees out of square (0.65 degrees while the pinch was
  loose), 1.3 N on the base.
- **Clamp grip:** squeezed once, both jaws at 27.0 degrees, 26.7 N on each fingertip.
- **Push on:** 0.36 mm off the axis and 0.47 mm low.
- **After release:** the jaws closed onto the bevels at 1.39 and 1.59 degrees. In the last frame the jaws pressed
  the funnel with 6.9 and 7.4 N and the base with 7.4 and 5.1 N, the funnel pressed the base with 12.9 N, and
  the ring stood 0.65 mm off the axis and 1.2 degrees from level.
- **Hold:** the funnel was held by the left hand on all 101 steps of the fitting.
- **Membrane:** `membrane_offset_mm` 0.57 at the end (limit 1 mm), `membrane_peak_mm` 0.58; `membrane_dragged` false.
- **Funnel at the end:** 1.29 mm off centre (limit 1.5 mm). The clamp's closing moved it 0.4 mm in the hand.
- **Right hand's exit:** it left sideways and touched nothing after the release (no arm force on the clamp,
  funnel or flask from step 814 on).
- **Flask:** it did not move (0.0 mm).
- **Arms:** no arm-to-arm contact.
- **Worst inverse kinematics error:** 0.46 mm.
- **End:** score 0.8 from step 813 and 1.0 at step 993, when both hands had been home for 1 s.

What Dream-1 did: its left hand went to within 3 cm of the funnel's axis, 10 to 28 cm above the bench, opening and
closing its gripper. It brushed the funnel once (1.0 N): the funnel rocked by 3 mm and settled where it stood. Its
right hand came within 1 cm of the clamp in plan, 12 to 34 cm up, opening and closing. Later it drew back toward
the robot without touching the clamp. The clamp and the flask did not move.

Appearance, from the rendered setup image and the review video: the scene reads as a filtration set on a lab
bench.
- A clear conical filtering flask with its side-arm, a grey silicone stopper and the glass base with the white
  membrane on its frit.
- The graduated clear funnel standing on its wipe.
- The blue anodised two-jaw clamp lying shut with its two handles.
- In the frames of the fitting the clamp closes round the two flanges under the funnel, with the left hand on the
  funnel's body.

Two rendering points remain:
- The flask's far wall shows some refraction banding.
- The fritted base is drawn as a long cone, where the real base is a shorter cup.


**Run 3 (previous scorer): demo 1.0 (993 steps), Dream-1 0.0.** The scene and demonstration were the same
as in run 4, and every scripted figure above was the same. The scorer of that time counted the funnel's slide
while it rested on the membrane (at most 0.71 mm). Dream-1 hovered over the funnel and the clamp and brushed the
funnel once (2.5 N). After run 3 the membrane criterion was changed to the end-state check (below), and run 4
recorded both runs again.

**Run 1 (scripted, 2,400 steps): 0.0.** These matched the predictions: reset (parts on their marks, jaws at
-2.0 degrees, masses 0.72 / 0.20 / 0.02 / 0.006 kg, joints hingeL and hingeR), the funnel grip (opening 63.9 mm,
19.4 N on each fingertip, funnel in the hand at (0, 56, 33) mm), and the clamp grip (jaws 27.0 degrees, opening
10.7 mm, 26.5 N on each fingertip, tilt 0.9 degrees). As the pads opened the jaws closed smoothly, 27 to -1.8 degrees
in 1.5 s. The handles left the pads at about 9 to 10 N, because the pads press near the handles' ends; that is
about 0.3 N m at the pin, as modelled. What went wrong:

1. *The servo shook.* Correcting 35 % of the measured error at every step, against an arm that lags two to three
   control steps, made the held funnel swing 2 to 4 mm about its goal (and the held clamp 1.5 mm). The funnel's
   align stage never converged.
2. *The funnel sat tilted.* The wrist's sag held the funnel 2.2 to 3 degrees out of square. Pinched at 19 N it
   could not square itself on the flange: it touched on one edge, carrying 0.6 N against its own 2 N weight. Both
   placings were rejected (0.93 mm off and 2.18 degrees out; then 1.93 mm and 2.33 degrees). The slides while it
   touched were recorded as a crease (3.1 mm).
3. *The demonstration then went on with the funnel lifted 12 mm off the base.* The clamp closed on the base's
   flange alone, so the jaws read -1.8 degrees: "clamp missed the flanges". The right hand backed out with its
   fingers 58 mm apart, caught a handle and dragged the loose clamp 40 mm. When the left hand let go, the
   hovering funnel fell and the trial latched 0.

Changes for run 2 (nothing in the scene, the scorer or the task text changed):
- Move-and-settle servo: feed-forward on a moving goal, and 60 % of the measured error corrected every 4th step.
- The hand is turned until the funnel's axis is within 0.8 degrees of the holder's before lowering.
- The pinch is loosened to about 6 N while the funnel is pressed 8 mm onto the base, then re-gripped.
- Up to three placings, and the last is left down.
- The right hand opens to 85 mm and backs out 8 mm upward.
- The worst IK error of each phase is logged (run 1: 5.9 mm somewhere), and so is the funnel's force on the base
  at each placing.

**Run 2 (scripted, 2,400 steps): 0.0, but the task itself was done.** Every stage met its prediction:
- Seating: one placing, 0.88 mm off centre, 0.33 degrees out of square (0.65 with the pinch loose), 1.3 N on the
  base. The align servo converged with no swings, and the worst IK error was 0.46 mm (funnel carry).
- Clamp on: pushed on 0.36 mm off the axis and 0.47 mm low. After release the jaws stopped at 1.4 and 1.6
  degrees, closed on the bevels, with 7 to 26 N from the jaws on funnel and base.
- The funnel was held on all 101 fitting steps.
- The score reached 0.8 at step 800, and the hands-off condition was met.

What went wrong: on the way home the right arm swung in joint space from just behind the clamp's handles
(ticks 852 to 864). Its fingertip hooked one handle, with up to 11 N, and dragged the clamped holder 34 mm along
the bench, so "flask pushed along the table" latched 0. A hull check of that swing shows the hand passing 6 mm
from the ring.

A scorer point also came up. The 0.94 mm "slide" (just under the 1 mm crease limit) was counted at the first
touch, while the funnel hovered up to 0.6 mm above the base and drifted. A funnel that is not resting on the
membrane cannot drag it.

Changes for run 3:
- *Demonstration:* after backing off the handles, the right hand goes out sideways through two Cartesian
  points, (-130, -60, +2) and (-140, -170, -38) mm from the joint, before the joint-space swing home. The arm's
  hulls then stay at least 80 mm from the ring and the funnel (worst joint margin 10.6 degrees on that path).
- *Scorer and text:* slide on the membrane is counted only while the ground faces touch (gap within 0.3 mm).
  task.json and the test of a hovering funnel were updated to match. Nothing else changed.

**Change after run 3 (scorer and text only).** Three review runs with the media found that the "careful" tier
could not be judged on a real bench. A running count of slide, gated by whether the glass faces are within 0.3
mm, cannot be read from a video or a photo. The membrane criterion is now an end state: ink ticks on the membrane
and the flange, read after lifting the funnel off, within 1 mm and no fold. The simulated scorer reads the same
thing, the membrane's offset from its marks at the end, as given by the rule above. Run 3 re-scored this way: 0.52
mm, and run 4 then measured 0.57 mm with the new scorer.

## Third-party assets and licenses

- Flask, base, funnel and clamp are convex hulls and revolved surfaces generated by `tools/build_assets.py`; no
  meshes, textures or images from any third party are included. Product names are used only to identify what was
  modelled.
- Robot, station and cameras: `yam-arena` (the yam-isaac-lab-arena repository above; its README states that it
  has no license yet) with I2RT's YAM meshes (MIT, `LICENSE.i2rt` in that repository). They are dependencies,
  not copied here.
- `dream_audit.py` and `recording.py` follow the pattern of other task folders by the same authors in this
  repository (MIT).
- Inspect Robots, inspect-robots-yam and the Dream client are installed dependencies under their own licenses.
- The code in this folder is under the MIT license (`LICENSE`).
