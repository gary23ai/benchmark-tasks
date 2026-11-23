# Hang a full-length broom in a household wall gripper

Hang the broom in the wall clip. Leave at least 30 mm of space under the entire broom head and take both hands away.

In a household cleaning closet, a wall or door gripper keeps a broom off the floor and close to hand. Step 4 of the manufacturer's example shows the user pressing a cleaning-tool handle between the two rubber faces. [3M cleaning closet example](https://www.command.com/3M/en_US/command/projects/all-projects/cleaning-closet-makeover/)

## Reproduce the setup

Use a 991 mm corn lobby broom with a 216 mm wide head, a 23.8 mm dry hardwood shaft and a nominal mass of 430 g. Stand its head on the marked floor plate. Use a clean household wall gripper with two flexible grey contact faces. Install the 3M Command 17007 following the manufacturer’s directions, including the one-hour wait after applying the adhesive. The published range is 20.3 to 25.4 mm in handle diameter, with a maximum load of 1.81 kg. The full article is approximately 79 mm wide, 47 mm deep and 85 mm tall. [Manufacturer's product and dimensions](https://www.command.com/3M/en_US/p/d/b00028133/) · [Installation and use](https://multimedia.3m.com/mws/media/1122014O/17007-es-34871767462.pdf) · [Broom dimensions](https://brushman.com/item/lobby/hpa-l/)

Place the gripper center 654 mm above the plate. With x pointing toward the wall and y toward the left arm, the shaft starts at (296, 33) mm. The center of the receiving pocket is at (398, 33) mm. Place the three-sided starting fork at 770 mm above the plate with 32 mm between its arms and the open side facing the gripper. The arm bases stand 610 mm apart on 355 mm pedestals. Set the broom down with the broad head parallel to the wall and the shaft inside its open fork.

The simulated shell is fixed to the wall. In horizontal section, the front edge of the inward-curved grey lip is 22 mm forward of the pocket origin. Each rubber shoe has a rounded nose, a flat middle section and a rounded rear section. The pair has approximately 12 mm of clearance at the unloaded throat and 15.6 mm in the flat receiving section. Faces are 17 mm high. Each shoe can compress outwards by up to 11 mm. The two lateral spring constants are 2,400 N/m, with 11 N·s/m damping. These are effective parameters for compressible rubber. The published product data give the outside envelope, handle range and rated load. They do not specify this inner contour or a stiffness curve. The test uses a 430 g broom. All forces that suspend it come from surface contact.

## Demonstration and human scoring

The arms grasp the same wooden shaft above and below the wall grip. They lift the bristles, bring the shaft out of its starting fork, press it between the two grey halves, lower it slightly and release it. The thin painted stripes are 5 mm apart and add no ridge or shoulder.

Watch the final 3 seconds. The shaft must sit between both grey faces, with its front edge behind the clip lips at their height. The holder alone bears the load, both hands are clear, and the marked shaft moves at most one 5 mm stripe spacing at the clip. Compare the lowest bristles with the two clearance scales. Give 1 if a 30 mm spacer fits under the whole broom head throughout, 0.5 if only a 5 mm spacer fits, and 0 otherwise. A handle held at the entrance or supported by the floor, fork, wall or robot earns 0.

Use a view from above and to one side to compare the handle with the front ends of the two grey lips. The outermost edge of the round shaft should be inward of that line. This corresponds to a shaft center at least 11.9 mm beyond the line. Follow the same paint stripe at the height of the holder for the whole final interval. The two floor scales show 5 mm and 30 mm. Look along the full width and depth of the bristles. A loose swing below the holder can be judged separately from actual shaft slipping at the grip.

## Referee camera views

The recorded view includes the three stock station cameras and four fixed optical views. The policy receives the stock observations. Camera positions and look-at targets are expressed in metres in the station frame, with elevation measured from its table datum. Lens focal lengths are in millimetres.

- Handle seated in the clip camera at [0.286, -0.143, 0.475], aimed at [0.398, 0.033, 0.301], focal length 39.0 mm.
- Front floor scale camera at [0.148, -0.522, -0.285], aimed at [0.379, 0.033, -0.314], focal length 40.0 mm.
- Whole broom camera at [-0.702, -0.752, 0.39], aimed at [0.357, 0.025, 0.11], focal length 20.0 mm.
- Rear floor scale camera at [0.088, 0.438, -0.255], aimed at [0.388, 0.033, -0.32], focal length 34.0 mm.

## Recorded evaluation

Dream-1 run: 0 (716 steps); demo: 1.0 (716 steps). The scripted and model trials used seed 0, 20 Hz and the same frozen scene. Measured runtime including startup was 316.93 seconds for the demo and 294.53 seconds for Dream-1.

Install the [pinned dual-YAM station](https://github.com/kairosiann/yam-isaac-lab-arena/tree/bd62771e60320433893bd1d2df0fc79b5ddcd6d1) on Linux, source `yam.env` and install `inspect-robots==1.0.0`, `inspect-robots-yam==0.38.0`, `dreamlabs-client==1.0.14`, `numpy`, `scipy`, `pillow`, `imageio[ffmpeg]` and `pytest`. Use the Inspect Robots Dream adapter at revision `b5e936a201cec66c54c297e522c920d98445e339`. Set the Dream credential privately. Run from this task directory with fresh output paths.

```sh
python -m pytest tests
python -m task_impl.run scripted --task . --out /tmp/robocurve-scripted
python -m task_impl.run dream --task . --out /tmp/robocurve-dream
```

An earlier Dream-1 trial ended at its local request cap after a service rate-limit response. The same account restarted after the service cooldown. The submitted model video records one complete new trial from the same initial scene, seed and control horizon.
