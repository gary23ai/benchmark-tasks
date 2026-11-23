## The objects, and where the numbers come from

| Quantity | Value used | Status |
| --- | --- | --- |
| Mouse pad | 250 x 210 x 3 mm, 105 g, cloth top, rubber-foam underside | size: the common "medium" office size; mass: **estimate** (90 to 120 g for this size) |
| Pad bending stiffness | EI = 0.001 N m^2 for the full 210 mm width; one hinge = 0.10 N m/rad | **estimate**; to be set from two measurements on the bought pad: droop of a 100 mm overhang (about 45 mm with this value) and time to lie flat after being let go (about 1 s) |
| Hinge damping, hinge stop | 0.01 N m s/rad, 50 degrees | **estimates** |
| Slide between strips | +-1.5 mm against 150 N/m | a stand-in for helical winding, not a property of a real pad |
| Friction | rubber 1, cloth 0.5, carton 0.8, band 0.5, gripper fingers 0.5 (simulator default) | **estimates** |
| Carton | 280 x 220 x 120 mm, fixed | chosen so that both arms reach the pad's edge and the roll's end |
| Band | inside diameter 70 mm, wall 3 mm, length 80 mm, 39 g | **derived** from the roll the demonstration makes: 61 mm across when finished and held, up to 67 mm after being slid along the carton, plus a few millimetres |
| Cradle | two V saddles 12 mm thick, 40 mm apart, faces at 35 degrees | only where the band starts; not part of the scoring |

No measurement has been made on a real pad, band or robot.

## How it is modelled

The pad is one floating articulation of 25 rigid strips 10 mm wide. Neighbours are joined through a light link by a sprung hinge along the strip (bending) and a sprung slide along the same line (a stand-in for the offset of a crooked, helical roll). Self-collision between the turns is on; each strip collides as a rubber underside and a cloth top. The band is one rigid body whose wall is 16 flat boxes round an open bore. PhysX, 240 Hz physics, 20 Hz control, solver iterations 16/4 for the pad and the band.

What this model cannot do, and what follows from it:

- It bends only the way it rolls. It cannot twist, stretch, crease or bend across the roll, and it has no shape memory.
- Each strip is rigid along the roll. A strip held, or inside the band, at one point is held along its whole length: the 80 mm band keeps all 210 mm of the roll shut, where a real pad would flare a little beyond the band; and the carton under part of the roll's length holds the pad's free edge shut along all of it.
- The slide between strips is about a hundred times softer than rubber foam in shear. In the robot-free test a roll tipped 30 degrees keeps its place but its turns sag along the roll by up to 20 mm.
- Rolling up, springing open and being held shut by the band all come from contact and the hinge springs. No software rule locks the roll.



## Physical grading

Scored on the state at the end of the trial (the run limit, or as soon as the 1.0 state is reached); nothing done earlier is held against the robot. On a real trial: note where the grippers are, then photograph the pad and band from above and end-on, and measure with a ruler. The pad counts as a roll if, seen end-on, it winds one way through at least one full turn, so that its two short edges overlap, and nowhere bends back the other way by more than 20 degrees: a pad folded flat in two or zigzag is not a roll, whatever is put round it. 0: the pad is not a roll, or less than a third of the band's length (27 mm) is over it. 0.3: the pad is a roll and the band is started on it: at least a third of the band's length (27 mm) is over the roll. 0.6: the pad is a roll and the band is on it: the whole roll passes through the band and stands at least 10 mm out of each of its ends. 1.0: in addition (a) the roll is square: measured along the roll, the turn that stands furthest out at one end and the turn that stands furthest back from it differ by no more than 10 mm; and (b) the banded roll has been put down and let go: both grippers' centre points have been at least 50 mm away from the band and from the pad for 2 seconds, so that nothing but the band is holding the roll shut. Where it is put down does not matter (the cradle, the table or the carton). A banded roll that is telescoped or still held scores 0.6. Possible scores: 0, 0.3, 0.6, 1.0. Inspect both exposed ends after release. Radial flaring beyond the sleeve is allowed; measure the 10 mm end offset along the roll axis, not across its diameter. The sleeve must surround all layers, and an open flap that unwinds the overlap fails the roll check.

[Manufacturer discussion of bands on rolled pads](https://darshion.com/why-mouse-pads-warp-after-shipping/). For a physical trial, record the two exposed ends against a ruler with hands clear. Read axial offset separately from radial flare. The source documents commercial practice; the 70 mm sleeve and 80 mm width are benchmark dimensions, not manufacturer test results.

The reference ends at 1.086 turns. The margin above one full turn is about 31 degrees. In a real pad one exposed end can unwind further than the other. Use the poorer of the two end views to check overlap. A wider end with overlap may pass; an end that has opened short of one turn fails even if the centre looks round. This margin is geometric and does not measure rubber stiffness.
