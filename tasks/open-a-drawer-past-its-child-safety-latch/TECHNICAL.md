# Technical reference

Dimensions and material values below distinguish published figures, measurements reported by sources, and estimates. Product pages identify the physical objects; no manufacturer assets are bundled.

## The objects, and where their numbers come from

| Quantity | Value used | Source | Status |
| --- | --- | --- | --- |
| Storage drawer, outside | 432 front to back x 365 wide x 175 high mm | Sterilite 16 Qt. Stacking Drawer, model 2301, maker's page <https://www.sterilite.com/product/16-qt-stacking-drawer/> ("17 x 14 3/8 x 6 7/8 in"; drawer stops "help prevent accidental removal") | read on the page |
| Drawer, inside at the bottom | 416 x 302 x 124 mm | the same page ("16 3/8 x 11 7/8 x 4 7/8 in") | read on the page |
| Drawer front, pull | front panel 343 wide, 3 thick, 12 to 148 mm above the table; pull a lip 120 wide, 8 thick, standing 12 proud, centred 132 above the table | assumed from the product's general shape | estimate |
| Underside of the frame's top | 152 mm above the table | drawer floor taken as 22 mm above the table plus the published 124 mm, plus 6 mm | estimate; measure on the unit |
| Drawer stop, slide force, mass | stop at 300 mm open; 3 N; 0.5 kg | the page says only that there are stops; the rest assumed for an empty plastic drawer on plastic runners | estimate |
| Latch | Safety 1st Spring-Loaded Cabinet & Drawer Latch, model 48392 (sold in 10-packs), white plastic, 2.5 in long x 1 in high x 1.5 in wide | maker's user guide for 48447, 48392, HS205, HS220 (PDF hosted by a retailer, <https://assets.unilogcorp.com/187/ITEM/DOC/Safety_1st_100018122_Instruction_Installation_Manual.pdf>); size from a retailer's listing, <https://www.target.com/p/-/A-14061586> ("2.5 Inches (L), 1 Inches (H) x 1.5 Inches (W)", UPC 052181483929) | read on the pages |
| Where the latch goes in a drawer | catch "on upper frame at center of drawer space with curved edge facing out"; latch on the drawer front, placed with the supplied template | the same guide | read |
| Opening at which it catches | 1.5 in (38 mm): "When drawer is opened 1.5 in the latch will engage catch. Press down on latch with finger to open." | the same guide; the sister models 48444/48390 give the same figure and place their catch "1 in from inside edge of cabinet frame" (<https://safetystore.iu.edu/content/Product%20Instructions/Safety%201st/SF%20Cab%20%26%20Drawer%20Latches%20instructions.pdf>) | read |
| Catch | 25 wide, 10 deep, hanging 10 below the frame's top; its catching face 23 mm behind the frame's front face | derived: latch length 2.5 in + 1.5 in opening, with the 3 mm front; consistent with the sister models' "1 in" | derived / estimate |
| Latch arm and hook | arm 5 thick, its top 138 above the table; hook 5 long, rising 9 to 147; base plate 3 thick | assumed from the latch's overall 1 in height | estimate |
| Latch spring | 0.30 N m/rad, wound 0.2 rad past the up stop; 10 degrees of travel down | assumed: about 1.1 N at the hook at rest, about 4 N to free it pressed 21 mm behind the pivot | estimate; measure with a kitchen scale |
| Friction | frame and drawer 0.20, latch 0.30, the pull's lip where the jaws hold it 0.60 | assumed | estimate |

Consequences that follow from these numbers (checked in `tests/`): the hook overlaps the catch by 5 mm and is
free once the arm is pressed 5.1 degrees; every part of the hook that reaches above the catch's underside is in
front of it once the drawer is open 58 mm, so the 0.6 tier at 60 mm needs the hook pressed; at the catch the
gap between the latch's base and the frame's front face is 32 mm; the arm lies 10 mm below the top edge of the
drawer front.

The maker's guide also warns that the latch "may be difficult to access if used on a top drawer that has a
counter top with a 1 in overhang"; this unit has no overhang, and the gap is open from above.

No product image, mesh or texture is included; every part is drawn as the box that is its collider: a white
frame, a clear drawer with its moulded pull, and the white latch and catch.

### How the latch is modelled

- The frame and the catch are fixed boxes. The drawer is one rigid body on a slide along x whose 3 N friction
  is a force-capped damper.
- The latch arm (with its hook and a 45-degree ramp on the hook's back for the catch's rounded edge) is one
  rigid body, 12 g, on a pin at its base in the drawer: a revolute joint about y with an angular drive used as a
  torsion spring whose target lies 0.2 rad above the up stop, so the arm rests on the stop; it can be pressed
  down 10 degrees. The latch does not collide with the drawer it is screwed to; it does collide with the
  catch, the frame and the robot.
- While the arm is up, the hook's front face meets the catch's back face when the drawer is open 38 mm and
  holds it there. A pull on the drawer presses the hook against the catch and turns the arm up into its stop,
  so pulling harder only holds it tighter, as on the real latch. Pressing the arm down while the drawer is
  pulled hard is harder (friction on the hook's face); easing the drawer back 1 to 2 mm first is easier.
- Once the hook's top is below the catch's underside, the drawer can be pulled past; the arm springs up again
  as soon as the fingertip leaves it.

What this leaves out: the real latch arm is a moulding that bends a little as well as turning on its pivot; the
real drawer has some play on its runners (the reset notes ask to check that pressing the front down by hand
does not free the hook); the frame and drawer walls are modelled 8 mm thick (real about 3) with the true inside
and outside sizes.

To regenerate the asset after editing `layout.py`: `python tools/build_scene.py`.



## Licence

MIT (see `LICENSE`). All code and the scene asset were written for this task; no third-party assets are used.


## Full simulation fidelity statement

Isaac Lab-Arena (PhysX, 240 Hz physics, 20 Hz control). Frame and catch are fixed boxes; the drawer is one rigid body on a slide whose 3 N friction is a force-capped damper, with a stop at 300 mm. The latch arm is a rigid body on a pin in the drawer with a torsion spring (0.30 N m/rad, wound 0.2 rad past its up stop: about 1.1 N at the hook at rest, about 4 N needed 21 mm behind the pivot to free the hook) and a stop 10 degrees down; the real latch's moulded spring is replaced by this. The hook's rounded back is a 45-degree ramp. All parts are boxes at their true sizes (frame and drawer walls modelled 8 mm thick with the true inside and outside sizes; front panel 3 mm). The drawer has no play on its runners. No software rule moves, holds or releases anything: the hook stops the drawer by contact with the catch, and only a fingertip pressing the arm down frees it. Sizes read on the maker's and retailers' pages are listed in the README; heights inside the unit, the pull's shape, the latch's spring and the drawer's friction are estimates. Not run on hardware.
