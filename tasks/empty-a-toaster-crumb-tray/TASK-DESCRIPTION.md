# Empty a toaster's crumb tray and slide it back in

People empty toaster crumb trays during ordinary kitchen cleaning; accumulated bread debris can char or burn during later use. Breville's BTA840 Care & Cleaning instructions (printed pp14–15, https://assets.breville.com/Instruction-Booklets/USCM/BTA840-instruction-manual.pdf) identify cooling/unplugging, pulling the tray, emptying and replacing it. Dualit's NewGen instructions (https://help.dualit.com/en-US/toaster-instruction-manuals-259361, printed p16) likewise describe frequent tray emptying. This task isolates dry emptying of loose bread chunks before any damp wiping or deeper internal clean.

## Instruction

Empty the cold, unplugged toaster's crumb tray into the waste bin, then slide the empty tray all the way back into the toaster.

## Fixed starting scene

Stock calibrated bimanual YAM station with unchanged robot geometry, grippers, gains and force limits. Control at 20 Hz; native CPU PhysX at 960 Hz with sweep and angular speculative CCD; GPU RTX cameras. Three stock RGB views resized to 320 × 176 and 14 joint/jaw readings; 14 stock joint/jaw targets. Coordinates: x forward from bases, y left, z above the 747.5 mm tabletop; bases at (0, ±305, 750) mm. Both robots start at stock READY. A generic full-size 170 × 300 × 190 mm steel-housing two-slot toaster is free on four rubber feet, mass 1.6 kg. Its tray datum is (730, 170, 20) mm with a 90° Z rotation, so the pull handle points toward the robots along −X. The removable 140 × 260 × 0.4 mm stainless floor has 6 mm flanges and a connected 44 × 20 × 14 mm ABS pull handle; calculated tray mass 143.2944 g. Forty individual dry bread chunks, 4–5.8 × 3–4.6 × 2–2.9 mm, rest in the tray; total mass 0.331632 g at an estimated 180 kg/m³. Their exact analytic-box dimensions are shared by physics, visuals and scoring. A stationary open plastic waste bin at (400, 360) mm has a 270 × 220 mm actual interior, a 4 mm floor and a 120 mm rim. Straight tray rails and a rear stop are modeled; no particular OEM click-latch is claimed. Supplied tools/build_assets.py and fixed reset coordinates reproduce the reconstruction. A disconnected plug and cord are visible; the appliance is already cold and unplugged.

## Scoring

Read actual native poses, velocities and robot/task contacts at every 960 Hz physics step. The complete tray floor must first clear the appliance guide front by 5 mm. Every chunk's full transformed corners must remain in the real tray or within the waste-bin opening (0.1 mm contact tolerance), without crossing the bin's outside bottom. Direct robot-to-chunk contact or an actual spill invalidates the trial. Final containment requires all 40 chunks above the inside floor top (0.5 mm contact tolerance), inside the opening and below the rim. The actual tray must return to its appliance-relative inserted datum within 1.5 mm horizontally, 1 mm vertically and 2° about both axes. Require tray linear speed below 10 mm/s, angular speed below 0.1 rad/s, all chunk speeds below 20 mm/s, and no robot contact with tray, appliance or bread for one continuous second between the first and last qualifying native samples. Housing displacement over 20 mm, tilt over 10° or an off-bench task object invalidates the trial. Score 0 initially or if invalid; 0.25 after complete withdrawal; 0.6 after emptying; 1.0 only after released reinsertion and stability. Only 1.0 is success.

## Action limit

1100 steps (55 seconds at 20 Hz)

## Fidelity and estimates

Full-size generic two-slot toaster and steel tray; free native rigid bodies. Dimensions, lumped housing inertia, bread density and friction are engineering estimates. Forty actual-size analytic bread boxes avoid convex-cooking shrinkage. Straight guides and a rear stop; no OEM click-lock claim. No heat, electricity, washing, adhering residue, fine dust, bread deformation or hardware calibration. Static bin and cord; no task drives or attachments.

## Reset

Restore all 42 task-body transforms and zero native velocities only at episode reset, with the loaded tray inserted and bread 0.6 mm above its floor for gravity settling. Return unchanged stock robots to READY. Thereafter native gravity and contact evolve all task bodies. Reference feedback changes ordinary robot joint/jaw targets; no task-body writes, snapping, injected task forces or proximity attachments.

## Procedure and product references

- [BTA840 care instructions, page 15](https://assets.breville.com/Instruction-Booklets/USCM/BTA840-instruction-manual.pdf)
- [BTA735 instructions, page 14](https://assets.breville.com/BTA735/BTA735_USCM_IB_E24_LR.pdf)
- [NewGen care manual, printed page 16](https://help.dualit.com/en-US/toaster-instruction-manuals-259361)
- [a real two-slot replacement tray](https://www.dualit.com/products/spare_01132)
