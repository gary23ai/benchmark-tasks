## The real objects, and where the numbers come from

| Quantity | Value used | Source | Status |
| --- | --- | --- | --- |
| Sheet | US Letter, 8.5 x 11 in (215.9 x 279.4 mm) | ANSI/ASME Y14.1 size A | standard |
| Paper | 20 lb bond copy paper = 75 g/m2; 4.52 g a sheet; about 0.1 mm thick | basis weight by definition (500 sheets of 17 x 22 in weigh 20 lb); thickness as usually quoted for 20 lb copy paper (0.097 to 0.104 mm) | weight derived; thickness quoted from memory, not read on a page: measure a ream (500 sheets about 50 mm) |
| Stack | 36 sheets: 3.6 mm, 163 g | derived | derived |
| Start skew | nine groups of four sheets, each shifted 2 to 4 mm along the short side, 4 to 7 mm along the long side and turned 1 to 2.5 degrees (table in `task.json`); 18 mm and 20 mm out of flush | our choice | fixed table, no random numbers |
| Paper on paper friction | 0.30 static, 0.25 dynamic | none | **estimate**; office paper is usually quoted between 0.3 and 0.5 |
| Binder clip | medium, 1-1/4 in (32 mm) wide, 5/8 in capacity | the usual catalogue size for a "medium" binder clip | catalogue size, not read on a maker's page in this work |
| Clip's parts | back 16 mm wide, jaws 21 mm deep, sheet metal drawn 1.2 mm thick, handles reaching 28 mm behind the back and standing 33 mm apart at their ends, 9 g | none | **estimates** to be measured on the bought clip (ruler, kitchen scale) |
| Clip's spring | 6 N at the shut lips, 8 N on the 3.6 mm stack (0.25 N m/rad about the back, 0.126 N m preload); about 9 to 16 N on each handle to open it fully | none | **estimate**, softer than a new clip; measure with a spring scale at the lips |
| Clip on paper friction | 0.35 static, 0.30 dynamic | none | **estimate** |
| What the stack lies on | two unopened 500-sheet reams of 20 lb Letter copy paper, stacked, long side away from the robot: 279 x 216 x 102 mm together, 4.5 kg; one fixed box in simulation | 500 sheets of the 0.1 mm paper above are 50 mm, 51 mm with the wrapper; a ream of 20 lb Letter weighs 5 lb by the definition of basis weight | the 102 mm height is what lets a YAM gripper (102 mm across its closing direction) get a finger under the overhanging paper; thickness with wrapper to be measured on the bought reams |
| Clip stand | clear acrylic fin 40 x 3 x 35 mm, fixed | our choice | |
| Who does this | nearly everyone who handles printed pages at home, at school or at work (the task is submitted under Societal relatability). It is also routine office work: general office clerks, O*NET 43-9061.00, "Employment (2025) 2,600,000 employees", listed task "Compile, copy, sort, and file records of office activities, business transactions, and other activities."; secretaries and administrative assistants, 43-6014.00, "Employment (2025) 1,889,800 employees", listed task "Make copies of correspondence or other printed material." | <https://www.onetonline.org/link/summary/43-9061.00>, <https://www.onetonline.org/link/summary/43-6014.00> | read on the pages on 20 November 2026 |

## How it is modelled

Everything is a rigid body with box or capsule colliders. There is no cloth, no soft body, no attachment and no
software rule anywhere in the scene.

- **The stack** is nine loose rigid plates, each standing for four sheets: 215.9 x 279.4 x 0.4 mm, 18.1 g, one
  box collider that is also the drawn sheet. The plates slide on each other, on the fingers, the table and the
  reams. So the stack can be skewed, jogged flush, sheared by a careless grip and dropped. It cannot bend, curl,
  crease or tear, and the four sheets inside a plate cannot slide on each other.
- **The clip** is two rigid halves joined by one hinge along the clip's back. Each half is a jaw plate, a rolled
  lip (a capsule along the clip's width), half of the back, and the wire handle as a thin plate fixed to the
  half in its folded-back position. A torsion spring between the halves shuts the lips. Squeezing the two
  handles together turns the halves about the hinge and opens the lips to 12.4 mm; nothing else opens the clip.
  It holds paper by the friction of its two lips.
- **Fixed things**: the two reams (one box) and the fin are static colliders. The fin's foot, the seam between the reams and the bands on their wrappers are drawn only. Nothing is marked on the table.
- **Solver**: GPU PhysX at 240 Hz; plates and clip halves with 64 position and 8 velocity iterations, contact
  offset 0.5 mm, rest offset 0. These are solver settings, not physical quantities.
- **Friction**: every task material combines by averaging. The stock arm and table have PhysX's default 0.5, so
  paper on a finger pad or on the table comes out at 0.40 and the clip on a finger pad at 0.43.

What this leaves out, and what it does to the task:

- Real paper droops when held by one edge. The plates do not, so the demonstration never relies on it: the
  stack is held by two hands, hangs, stands on the table or lies on the reams. The one place a rigid stack is
  briefly held out sideways (laying it back on the reams) is marked below.
- Contact between two rigid plates is carried at their corners, so a pinch anywhere resists twisting more than
  a pinch on real paper does. A real stack held by one hand at one spot fans more easily.
- Under the clip the plates sink into each other by 0.3 to 0.7 mm in total (solver contact compliance): the lip
  gap on the 3.6 mm stack reads 2.9 to 3.3 mm.
- Air, static cling and curl, which make real sheets stick or float, are not modelled.

## Physics checks without the robot

`tools/physx_probe.py` runs the task's own scene file in PhysX with stand-in hands: a moved palm with two
fingers on slides, all three carrying the YAM gripper's own collision hulls, each finger with the gripper's own
position drive (5000 N/m, 100 N s/m, at most 50 N). Results are in `results/probe-*.json`.

| Check (`physx_probe.py <mode>`) | Goal | Measured |
| --- | --- | --- |
| `jog`: the skewed stack in a loose cradle (stack + 3 mm), dropped, then lifted 15 mm and dropped four times | long edges flush within 1 mm | 17.06 mm at the start; 0.0 mm after the first drop; at most 0.02 mm after each of the four |
| `pat`: a loose hand over each upper corner closes in to a sheet's length plus 2.5 mm, then four 12 mm strokes from side to side | short and long edges flush within 1 mm | short edges 12.06 mm before, 0.94 mm closed in, 0.0 mm after the strokes, 0.0 mm pinched; long edges 0.0 mm |
| `slip`: one hand pinching the top edge with a capped force, lifting 30 mm | held when pinched, sliding when loose | held at 10 N, 5 N, 3 N; plates slide out at 2 N, 1 N, 0.5 N (Coulomb estimate for the middle plates: 2.37 N) |
| `carry`: fingers commanded shut (about 9 N a finger) | plates together within 0.5 mm | two hands: swing to 35 degrees and back, carry 30 cm: 0.01 mm; one hand at the middle of the top edge: carry 30 cm: 0.01 mm. Recorded but not used: one hand: quarter turn in the stack's own plane: 0.64 mm, the plates slip |
| `clip`: a hand closes on the two handles, lifts the clip off its fin, pushes it onto the short edge of a held stack and lets go; then the lift check | opens wider than stack + 2 mm; stays in the fingers; shuts on all nine plates; carries them | lip gap 2.89 mm on the fin, 12.63 mm squeezed, 3.32 mm on the stack (36 sheets in the jaws, stack not moved); lift check: 36 sheets carried, peak pull 2.39 N |
| `demo` | the whole demonstration with stand-in hands on the planned gripper paths | development aid; the recorded evidence is the simulator run |

All at 240 Hz with nine plates of 0.4 mm: neither a higher physics rate nor thicker colliders were needed. These checks were run on an earlier scene file (an MDF block where the two reams now are, and a drawn tape mark); none of them touches what the stack starts on.

Two things were learned from the gripper's real collision shape and are used by the demonstration:

- The fingertips' end is rounded and the fingers' backs carry two prongs with a gap between them. Neither is a
  surface to push thin sheet edges with: pushed off centre, the sheets slide off to one side. (An earlier
  version of the demonstration pushed the short edges against an end stop with a fingertip. It worked with
  box-shaped stand-in fingers and failed with the real ones; the stop has been removed from the scene.)
- Behind the finger pads, 62 mm behind the gripper's centre point, each finger's carriage has a cross-bar that
  spans the gap between the fingers. With the fingers loosely on either side of the paper it is what the
  sheets' ends meet: the gripper's equivalent of the heel of a hand.



## Physical grading

Judged at the end of the trial (the run limit, or as soon as the robot has let go with the 1.0 conditions met) from a photo, with a steel rule and one lift test. Let go: for the last 2 s no part of the robot has touched the paper or the clip and both gripper centre points have been at least 50 mm from them. Clamped: a sheet lies between the clip's shut jaws, the middle of their lips at least 3 mm in from the sheet's edge. Lift test (by the referee after the end, not by the robot; only if let go): the clip is taken by its back and lifted until the whole stack hangs from it, its lowest corner 152 mm above the table (50 mm above the reams), and held 2 s. Held sheets: after a lift test, the sheets still clamped that slid no more than 3 mm under the lips (a sheet that slid shows as a step at the stack's edges); without one (not let go), the sheets clamped. Flush, read before the lift test: on the clipped stack, measure on each of its four sides the step between the sheet edge that sticks out most and the one that sits back most; take the larger of the two long sides and the larger of the two short sides. Clip placed: the clip is on a short edge (its jaws point along the long side within 25 degrees) and the middle of its lips is within 20 mm of the middle of that edge. 0 for the whole trial if a sheet ever leaves the table (a corner beyond the table's edge or more than 30 mm below its top). Count only full-size sheets lying unfolded in the clipped stack. Temporary fanning while jogging the stack is allowed. Otherwise 1.0: let go, all 36 sheets held, both flush values at most 2 mm, clip placed. 0.6: let go, all 36 sheets held, both flush values at most 6 mm. 0.3: at least 18 sheets held. 0: anything else, including nothing done. Possible scores: 0, 0.3, 0.6, 1.0. In simulation the 36 sheets are nine rigid plates of four, each counting as four sheets. Which hand does what, how the stack is squared and where it is left are not scored.
