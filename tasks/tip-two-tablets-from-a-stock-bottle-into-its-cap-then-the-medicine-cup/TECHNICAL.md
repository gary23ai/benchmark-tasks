# Tip two tablets from a stock bottle into its cap, then into the medicine cup

An open stock bottle holding 30 tablets, its cap lying open side up, and an empty medicine cup stand on the
table in front of two YAM arms. The robot has to take a dose of two tablets the way most people do at home, and
the way nurses are taught to when tablets come loose in a bottle: tip them into the bottle's cap, tip any extra
back, tip the two from the cap into the cup, and never touch a tablet. The task is filed under Societal
relatability; the nursing sources below show the method is also taught, not that 500,000 workers do it. Everything is at true size, every body is a free rigid body, and no
code counts, moves or releases a tablet.

| Path | What it is |
| --- | --- |
| `task.json` | the task card |
| `rdb_tablets_cap/layout.py` | every dimension and mass, each marked PAGE (read on a product or label page), EST (estimate) or MODEL (simulation choice) |
| `rdb_tablets_cap/tablets.usda` | the scene asset, generated from `layout.py` by `tools/build_assets.py` |
| `rdb_tablets_cap/fixed_scene.py` | where things stand, instruction, run limit |
| `rdb_tablets_cap/grading.py` | the scorer (NumPy only) |
| `rdb_tablets_cap/adapter.py` | adds the objects to the stock Inspect Robots YAM station, feeds the scorer, logs diagnostics |
| `rdb_tablets_cap/scripted.py` | the scripted demonstration |
| `rdb_tablets_cap/run.py` | runs one trial and writes the result, videos and scorer trace |
| `tests/`, `tools/` | unit tests, a dry run without the simulator, the asset generator |

## Results

Final scene and scorer (run 5), Isaac Sim, 20 Hz, 320 x 176 policy cameras, seed 0, zero jitter:

| Policy | Score | Steps | Wall time | What happened |
| --- | --- | --- | --- | --- |
| scripted demonstration | 1.0 | 1,382 (ended on success) | 314 s | The left hand picked the cap up and held it level, open side up, its underside 80 mm above the table. The right hand took the bottle and tipped it over the held cap while the left kept pinching it. At 119 degrees a tablet entered the neck and the bottle was tipped back; one tablet slid into the cap, a second was at the lip, and the bottle went back upright at 115 degrees: 2 in the cap, 28 in the bottle, none on the table. The right hand stood the bottle back on its mark; the left hand tipped the cap over the cup and both tablets left at 104.4 degrees. No gripper touched a tablet (largest arm-to-tablet contact 0.0 N); no arm touched the other. |
| Dream-1 | 0.0 | 4,800 (step limit) | 881 s | Both arms moved about over the table for the whole trial (100 action requests). The right gripper brushed the bottle (at most 1.2 N) and the cap (0.25 N) without moving either by more than 0.1 mm; nothing was lifted or tipped, and all 30 tablets stayed in the bottle. |

Runtime per trial: about 5 minutes for the demonstration and 15 minutes for a full 4,800-step trial on one GPU.

**Both hands at once, as measured.** From the moment the right hand began the pour (step 450) until it stood the
bottle down (step 863):
- the left hand's two fingertips each pressed on the cap with 8.41 to 8.43 N (the scorer needs 1 N on each);
- the right hand held the bottle with 23 to 27 N;
- the bottle and the cup pushed on the cap with 0.0 N;
- the cap's lowest point stayed 79.9 to 80.1 mm above the table, and it tilted at most 0.2 degrees.

Over the whole time tablets were in the cap (steps 785 to 1,205, through the tip into the cup), the scorer counted
the cap as held at every sampled step, the lowest fingertip force was 7.4 N, and the rule for a tablet in a cap
that is not held never fired (`void` null throughout).

**Expected against measured.**

| Quantity | Expected | Measured (run 5) |
| --- | --- | --- |
| each left fingertip on the cap during the pour | about 8 N | 8.41 to 8.43 N |
| bottle and cup pushing on the cap | about 0 | 0.0 N |
| cap height during the pour | about 80 mm, never under 5 mm | 79.9 to 80.1 mm |
| cap tilt in the hand during the pour | under about 3 degrees | at most 0.2 degrees |
| first tablet into the neck | 115 to 122 degrees | 119.2 degrees |
| tablets in the cap after the pour | 2 or 3, at most 1 stray | 2, no stray |
| cap tilt when the tablets left it over the cup | about 105 degrees | 104.4 degrees |
| demonstration length | about 1,380 steps (as run 4) | 1,382 steps |
| arm-to-arm contact | none | 0.0 N |

**The step that tips extra tablets back into the bottle did not run in Isaac**, in run 5 or in any earlier run:
every successful pour put exactly two tablets in the cap. That code path (step 4 of the demonstration below) has
been run only in the dry run without the simulator. A pour that gives three or more would exercise it.

### What the frames look like

`setup.png` (the overhead policy view at the start) shows the white bottle with its blue-banded label, the white
cap lying open side up and the translucent cup, each about 10 pixels across, and both arms. During the pour the
overhead view shows both arms meeting over the middle of the table: the left gripper holding the cap and the right
gripper tipping the bottle over it. The review camera shows the threaded neck of the tipped bottle above the cap
held in the left gripper, though the grippers hide most of the cap; afterwards it shows the bottle standing with
its label and the ribbed cap. A tablet is not visible in the overhead view. (Frames checked from run 4; run 5 is
the same scene and the same demonstration, step for step.)

## Earlier runs

Run 4 (same scene and demonstration; the previous scorer tested only the cap's height, which a review showed
would pass a cap propped on the upturned cup): demonstration 1.0 at 1,382 steps (314 s), with the cap 79.9 to
80.1 mm up and 16.8 N of the left hand on it through the pour (per-fingertip forces were not logged then);
Dream-1 0.0 over 4,800 steps (908 s), touching nothing.

Run 3 (previous version: the cap lay on the table during the pour, which the current scorer zeroes; same objects):

| Policy | Score | Steps | Wall time | What happened |
| --- | --- | --- | --- | --- |
| scripted demonstration | 1.0 | 1,380 (ended on success) | 296 s | The right arm tipped the bottle over the cap lying on the table. At 119 degrees a tablet entered the neck and the bottle was tipped back 8 degrees; that tablet slid into the cap, and a second followed at 123 degrees, when the bottle went back upright: 2 in the cap, 28 in the bottle, none on the table. The left arm then picked the cap up and tipped it over the cup; both tablets left at 106.8 degrees. No gripper touched a tablet (largest arm-to-tablet contact 0.0 N); grasps centred within 1 mm. |
| Dream-1 | 0.0 | 4,800 (step limit) | 908 s | Both arms moved over the table for the whole trial; the left gripper hovered near the cup, the right one pushed the bottle 7 mm (7.5 N) and later brushed the cap (0.7 N, 1 mm). Nothing was lifted or tipped; all 30 tablets stayed in the bottle. |

In run 3 the step that tips extra tablets back did not run either.

Runs 1 and 2 (earlier versions, the cap on the table):

- Run 1 (score 0.0): the first pour put 3 tablets in the cap with none spilled. The bottle was then set down
  where it overlapped the cap and fell over. The cap was tipped only to 65 degrees, which does not empty a
  dish. Fixed: the bottle stands clear of the cap; the cap is tipped past 90 degrees, closed loop on the count.
- Run 2 (score 0.0): the tablets held on the bottle's shoulder until 125 degrees and then ten left within a
  quarter of a second, six of them onto the table. The cap's closed-loop tip into the cup worked (4 tablets out,
  one at a time, between 92 and 98 degrees). Fixed: the bottle pauses at 105 degrees, then tips at 2 degrees a
  second and tips back 8 degrees the moment a tablet enters the neck, as in run 3.

## Where the method comes from, and who uses stock bottles

- Nursing Skills (Open RN, Chippewa Valley Technical College, CC BY), "Checklist for Oral Medication
  Administration", step "Prepare the required medications": "When removing tablets or capsules from a multi-dose
  bottle, pour the necessary number into the bottle cap and then place the tablets or capsules in a medication
  cup. If it is necessary to touch the tablets, wear gloves."
  <https://med.libretexts.org/Courses/University_of_South_Carolina_Upstate/Nursing_Skills_(OpenRN)/05%3A_Administration_of_Enteral_Medications/5.04%3A_Checklist_for_Oral_Medication_Administration>
- Iowa Department of Education, School Nurse Toolkit, "General Medication Administration Skills Checklist:
  Tablets & Capsules": "Remove the bottle cap. Hold cap in one hand, bottle in the other. Pour the correct number
  of capsules or pills into the cap. Pour the correct capsules or pills from the cap to a clean medicine cup."
  <https://educate.iowa.gov/pk-12/student-supports/specialized-support/school-nurse-toolkit/section-8-medication-administration-delegation-school-nurse/general-medication-administration-skills-checklist/tablets-capsules>
- OpenStax Clinical Nursing Skills, 12.1: bottles "contain multiple doses of the medication (multidose
  packages)" and "Medications that come prepared in a bottle require handling", against blister packs that
  "protect the pills from being handled". <https://openstax.org/books/clinical-nursing-skills/pages/12-1-administering-oral-medications>
- Where loose stock bottles are used: Maryland Department of Health, Office of Health Care Quality, transmittal
  to all nursing homes of 25 March 2026, "2017 Revised List of Approved Over the Counter Medications that Nursing
  Homes May Purchase in Bulk" (acetaminophen 325 and 500 mg, aspirin 81 and 325 mg, senna, docusate, vitamins
  and others, "exempt from resident specific labeling requirements").
  <https://health.maryland.gov/ohcq/docs/Transmittals/TRANSMITTAL_LTC-2017-Revised-List-OTC-Bulk-Medications-03.25.2026-A.pdf>
- O*NET Core tasks: 29-2061.00 licensed practical and licensed vocational nurses, "Administer prescribed
  medications or start intravenous fluids, noting times and amounts on patients' charts"; 31-9092.00 medical
  assistants, "Prepare and administer medications as directed by a physician"; 29-1141.00 registered nurses,
  "Administer medications to patients and monitor patients for reactions or side effects".

What these sources do not give is a count of how often a nurse pours from a stock bottle rather than opening a
unit-dose pack. Hospitals mostly dispense unit doses. The task card says so.

The school-nurse checklist has the cap held in one hand while the other tips the bottle; the nursing textbook only
says to pour into the cap. The task follows the checklist: the instruction asks for the cap to be held, and the
scorer zeroes a run in which a tablet lies in the cap while the cap is not held in a hand. Tipping the dose from the
cap into the cup is a one-handed step.

## The objects, and where their numbers come from

| Object | Value used | Source | Status |
| --- | --- | --- | --- |
| Bottle | 120 cc white HDPE packer, 38-400 neck, 1.989 in (50.5 mm) across, 3.402 in (86.4 mm) tall, 13.63 g | Container & Packaging item B723WH, <https://www.containerandpackaging.com/products/hdpe-packer-bottle/B723WH> | read on the page |
| Bottle bore, shoulder, neck | bore 31 mm; shoulder from 62 to 71 mm up; neck 35.2 mm across under the thread | none: the page gives no drawing | estimates, to be measured on the bottle |
| Cap | 38-400 white polypropylene push-down-and-turn child-resistant cap, 1.756 x 0.677 in (44.6 x 17.2 mm) outside | The Cary Company item 68WARG, <https://www.thecarycompany.com/38mm-38-400-white-child-resistant-cap-argus-loc-pdt-te-liner> | read on the page |
| Cap inside, cap mass | 36 mm across, 13.2 mm deep; 6 g | none | estimates |
| Tablets | round, brown, 10 mm, coated, 200 mg ibuprofen | DailyMed label of a 250-count bottle (Chain Drug Marketing Association, NDC 63868-268-25): "Color: Brown, Shape: Round, Size: 10 mm", <https://dailymed.nlm.nih.gov/dailymed/drugInfo.cfm?setid=c13455a3-307a-4fe1-e053-2a95a90adf8e> | read on the label page |
| Tablet thickness, edge, mass | 5 mm at the middle, 2.6 mm edge band, 0.33 g | none | estimates, to be measured |
| Medicine cup | 1 oz graduated polypropylene cup, top 1 7/8 in (47.6 mm), bottom 1 1/8 in (28.6 mm), 1 3/8 in (34.9 mm) tall | WebstaurantStore, Choice item 346MEDCUP1Z, <https://www.webstaurantstore.com/choice-1-oz-disposable-polypropylene-graduated-medicine-cup-case/346MEDCUP1Z.html> | read on the page |
| Cup mass | 1.1 g | the same page: 12.52 lb shipping weight for 5,000 cups | upper bound |
| Friction | tablet on plastic and tablet on tablet 0.35; plastic 0.6 | none | estimates |

The bottle named is an empty packer bottle of the kind over-the-counter tablets are sold in, and the cap is one
that fits its neck; the tablets are put into it for the trial. Any 100- to 250-count bottle of round 10 mm
tablets with a screw cap about 45 mm across would do as well, with its own measurements. Ibuprofen tablets are
named because their label gives the size and because brown tablets show against a white cap; nothing depends on
the drug. No part number is claimed beyond the three item numbers above, and no product image is in this folder.

## Run and score a trial

Runtime: the RoboCurve YAM runtime with Python 3.12, Isaac Lab-Arena 0.3.1 on Isaac Sim 6.1 (Linux, NVIDIA RTX
GPU), Inspect Robots 1.0.0, inspect-robots-yam 0.38.0 and yam-arena 0.1.0. Install this folder into that
environment with `uv pip install --no-deps .`. For the Dream-1 run, put a Dream Labs key where the Dream client
looks for it; no key is stored here.

```sh
python -m rdb_tablets_cap.run scripted --out out/scripted    # the scripted demonstration
python -m rdb_tablets_cap.run dream --out out/dream          # one Dream-1 trial
```

Both commands run one trial through Inspect Robots with the scorer `tablets_cap_score`, seed 0, zero object
jitter and a limit of 4,800 actions at 20 Hz (240 simulated seconds). The scene is fixed, so there are no other
seeds. Runtime per trial: about 5 minutes (demonstration) to 15 minutes (a full 4,800 steps). Each run writes `RESULT.json`, `media/setup.png`, `media/top_cam.mp4` (the
overhead policy camera, 320 x 176), `media/side_cam.mp4` (a review-only camera 0.34 m above the table in front of
the left arm, 1280 x 704, never given to a policy) and `media/grader-trace.jsonl` (the scorer's state at every
step). `run.py scripted` exits 0 only if the score is 1.0.

Policies receive 320 x 176 RGB from the overhead and the two wrist cameras plus joint and gripper positions, and
nothing else.

### What the cameras can show (computed from the camera calibration, and checked against the rendered frames)

The overhead camera is 0.94 m above the table and about 1.08 m from the cap; at 320 x 176 its focal length is
196 pixels, so one pixel covers about 5.5 mm there. The bottle is about 9 pixels wide and 14 tall, the cap and
the cup about 8 to 9 pixels across, and a tablet 1.8 pixels. A brown tablet in the white cap can at best show as
a darker pixel or two: a policy cannot count tablets from the overhead view, and two tablets side by side cannot
be told from three. The wrist cameras (focal length 195 pixels at this size) sit about 0.10 to 0.15 m from
whatever their gripper holds, where a tablet is 13 to 20 pixels across; a gripper holding the cap, or held over
it, can show the tablets in it well enough to count. Whether a wrist camera has the cap in view during the pour
depends on how the policy holds things. The 1280 x 704 review camera is for
the person scoring.

## Scoring, in the simulator and on a real table

The scorer (`grading.py`) is given, every control step, the centre of each tablet, the poses of the bottle, the
cap and the cup, which tablets a gripper link is pressing on (contact force of 0.005 N or more), and the two
gripper centre points from the joint readings. The rules are the ones in `task.json`. A tablet "has lain in the
cap" once its centre has been inside the cap for five steps running (0.25 s); going back into the bottle clears
that. The cap counts as held when both fingertips of one gripper press on it with at least 1 N each (read from
the simulator's contact forces between each finger link and the cap; the demonstration pinches with about 8 N a
finger). It counts as supported by the bottle or the cup when either pushes on it with 0.02 N or more (the cap
with two tablets weighs 0.066 N), and as resting on the table when its lowest point is less than 5 mm above it.
A tablet in the cap for 0.25 s while the cap is not held, or is supported by the table, the bottle or the cup,
zeroes the run; shorter lapses are forgiven, and an empty cap may lie anywhere. On a real table the same is judged from an overhead video for the
three history rules (straight into the cup; a gripper handling a tablet; a tablet in a cap that is not held in a
hand or is resting on something) and by counting at the end: tablets in the cup, in the cap, on the table and
floor, and the bottle poured out to count the rest.

Strategies the scorer refuses, each with a test in `tests/test_grading.py`: pouring into the cap while it lies
on the table (one hand doing the whole task); pouring into the cap balanced on the upturned cup or on the bottle,
with or without a finger on it; pouring into a cap no gripper is pinching; pouring from the bottle straight into the cup (0 even if exactly two land); a tablet that only bounces through the cap; pushing, pinching or
carrying a tablet with a gripper; putting tablets back in the bottle and then pouring them into the cup; leaving
with the cup knocked over; stopping with a gripper still on the bottle (capped at 0.2). Leaving extras in the
cap, or up to three tablets on the table, costs 0.3; four or more is a spill and scores 0.

## The simulation model, and where it is not the real thing

- Bottle, cap, cup and 30 tablets are 33 free rigid bodies. There are no joints, no kinematic bodies, no attach
  or release rules, and the task code never writes a pose after the reset.
- What the cameras see are lathe-turned display meshes at the true outside shapes (bottle with thread and
  label, ribbed cap, translucent graduated cup). What collides are invisible rings of 16 convex staves and a
  floor disc per container. The bottle's collision wall is 4 mm thick with the true outside, so its inside is
  42.5 mm across instead of about 48.5 mm; the cup's collision wall is 2.5 mm thick outside its true inner
  surface. The staves give the gripper pads flat faces to close on.
- The cap is one rigid dish. Its thread, liner and the push-and-turn mechanism are not modelled; the task starts
  with the cap off and never asks for it to be put back on.
- A tablet is a 50-point convex shape with a flat edge band and domed faces; continuous collision detection is
  on for tablets. Contact offset 1 mm, rest offset 0, 16 solver position iterations for tablets and 32 for the
  containers, 240 Hz physics.
- Real coated tablets chip, shed dust and can stick slightly; a real HDPE bottle gives a little when squeezed.

## The scripted demonstration

`scripted.py` reads the joint readings and simulator state (object poses, tablet positions), which the review
rules allow for a demonstration, and changes nothing in the simulation.

**Which parts use both arms.** The pour is done by both arms at the same time, as nurses are taught ("Hold cap
in one hand, bottle in the other"): the left hand holds the cap level, open side up, 80 mm above the table, while
the right hand tips the bottle over it. The scorer requires this: a tablet in a cap that is not pinched by a
gripper, or that rests on the table, the bottle or the cup, zeroes the run. Tipping the cap, by itself, into the medicine cup (and, if needed, back over the bottle) is done
with one hand, the left, while the right arm waits.

1. The left gripper picks the cap up by its skirt (coming down from above, from the robot's left-front side,
   hand 30 degrees below level) and holds it level at (0.340, 0.000), its underside 80 mm up.
2. The right gripper takes the bottle by its body (coming straight down onto it from 55 mm above, hand 20
   degrees below level, from the right and a little behind), carries it over the held cap and tips it toward the
   robot's left and back (azimuth 135 degrees): quickly to 70 degrees, at 8 degrees a second to 105 degrees,
   where it holds 1 s so the tablets settle on the shoulder, then at 2 degrees a second. The moment a tablet
   enters the neck the bottle is tipped back 8 degrees: the neck still slopes down steeply enough for that
   tablet to slide out, but the pile behind it stops. Tipping resumes when the neck is empty. The bottle goes
   back upright (60 degrees a second to 95 degrees, then faster) as soon as enough tablets have left it or lie
   within 8 mm of its lip. The lip is kept 12 mm short of the cap's centre and the bottle's underside 20 mm above
   the cap's rim, clear of the fingers holding the cap. This metering gave exactly two tablets in Isaac runs 3 and
   4.
3. Two in the cap: the right arm stands the bottle back on its mark and lets go. Fewer: the pour is repeated for
   the missing ones, with the cap still held.
4. More than two: the right arm stands the bottle at (0.420, 0.000) and lets go. The left hand lifts the cap,
   carries it across above the bottle and lowers it until its inner rim is 26 mm above the bottle's lip and 6 mm
   short of the mouth's centre, then tips it about a level axis (quickly to 80 degrees, then 8 degrees a second,
   holding 1.5 s after each tablet leaves) until only two are left in it, and tips it back level. If fewer than
   two are left, the left hand holds the cap up again, the right takes the bottle and pours again.
5. Last, the left hand holds the cap's inner rim 30 mm above the cup's rim and tips it the same way until it is
   empty, then puts the cap down.

Pouring stops if two or more tablets have missed the cap or the bottle is not standing when it is to be taken.
The flow of tablets is not repeatable tablet for tablet, so the demonstration's length varies with the number of
pours; a tablet dropped on the table cannot be recovered without touching it, which caps the run at 0.7.
The cap is never put down with a tablet in it.

## Checks without the simulator

- `pytest tests`: the scorer's tiers and refused strategies, which container a tablet counts in where two
  overlap, asset against layout, `task.json` against the code, and the dry run below.
- `python tools/dryrun_demo.py --clearance <folder with clearance.py>`: the demonstration's commands played
  through lagging arms and a toy tablet rule (tablets leave a bottle tipped past 120 degrees in a set pattern,
  spend 0.3 s in the neck and land under the lip; a cap tipped past 105 degrees lets its tablets go one at a time, or two at once, from its low rim),
  scored by the real scorer.
- Reach, from several starting guesses: the bottle's tipping arc from upright to 125 degrees over the cap held
  80 mm up; both grasps on the table; the cap's tipping from level to 130 degrees with its rim held over the cup
  and over the mouth of the bottle standing at (0.420, 0.000).

| Case | Score | Steps | Pours (tilt where it stopped, tablets then in the held cap) | Cap tipped (largest tilt, tablets left in the cap) |
| --- | --- | --- | --- | --- |
| one tablet at a time | 1.0 | 1,495 | 112 deg, 2 | over the cup: 106 deg, 0 |
| a rush of four, then singles | 1.0 | 1,701 | 112 deg, 4 | over the bottle: 106 deg, 2; over the cup: 106 deg, 0 |
| three in the cap, two slide back together | 1.0 | 2,593 | 112 deg, 3; again: 112 deg, 2 | over the bottle: 106 deg, 1; over the cup: 106 deg, 0 |
| one, a pause, then more; arm lag 0.2 s, joint bias | 1.0 | 1,508 | 113 deg, 2 | over the cup: 106 deg, 0 |
| every tablet lands 30 mm past the cap | 0.0 | 4,800 | 112 deg, 0 (two strays; pouring stops) | - |

In every case the least joint-limit margin is 7 degrees or more and the largest IK error 0.1 mm. Least gaps from
the arm collision hulls (first three cases), with the cap held up for the pour: between the two arms 20 mm;
the left hand (holding the cap) to the tipped bottle 8 mm; the right arm to the held cap 35 mm; the cap to the
standing bottle 6 mm while it is tipped over the mouth; the left arm to the cup 14 mm; arm to table 2.6 mm (a
pad corner while the left gripper takes the cap). Fingers come within 2 to 5 mm of the bottle or the cap only
while closing on or opening from them. Reach was searched over the pour direction, both hands' grasp directions
and the cap's position: with the bottle tipped toward the robot's left at 120 degrees the two grippers met
(under 2 mm apart); tipped toward 135 degrees with the bottle taken from the right and a little behind, they keep
20 mm apart through the whole arc to 125 degrees.

What the dry run cannot show: whether the finger pads hold the bottle and the cap, how the tablets really leave
the bottle and the cap and where they land, and whether the 1 g cup stays put when they land in it.

## Third-party assets and licenses

All geometry is procedural, generated by `tools/build_assets.py` from `layout.py`; the scene code, the scorer
and the scripted controller are written for this task and provided under the MIT License in `LICENSE`. No
manufacturer mesh, image or CAD model is bundled; product pages were read for dimensions only.

| Asset or dependency | Use | License | Bundled here |
| --- | --- | --- | --- |
| YAM arm and gripper model | as distributed with yam-arena | MIT (LICENSE.i2rt) | No |
| Standard station (table, cameras, lighting) | yam-arena 0.1.0 | its README states no license yet | No |
| Isaac Lab-Arena, Isaac Sim | simulation runtime | Apache 2.0; NVIDIA license | No |
| Inspect Robots, inspect-robots-yam | evaluation harness | as distributed | No |
