# Hang a full-size dining chair upright by both armrests

Hang the chair from the dining tabletop by both armrests. Leave its four feet clear so the floor can be cleaned, and let go.

A 3.5 kg rigid birch chair (510 mm wide, 404 mm deep; 360 mm round seat at 450 mm) stands centered at the front edge of a fixed 690 mm high, 690 mm wide, 26 mm thick tabletop. The two armrests project forward from rear supports; each has a blue 10 mm side band. Align the chair square to the edge on its four floor marks, with the front ends of the arms 20 mm short of the tabletop. Mount both arms on 490 mm pedestals above a 1 m floor deck. Align their bases 610 mm apart. Leave the tabletop underside open at the front.

The floor measures 1,000 by 920 mm. Both robot bases sit on 490 mm pedestals. The tabletop measures 400 mm front to back. Use its open front edge. The armrests are 404 mm long, 30 mm wide and 28 mm high. Each lower face starts 601 mm above the floor. The assembled chair weighs 3.5 kg. Its location is free throughout the test. Both armrests are fixed to the chair, as they are on real furniture.

## Human scoring

Keep the front edge of the table, both blue armrest bands, the four feet and the hands in view. A side view with a ruler shows gaps. Give 0.5 when both band centres are at least 20 mm inside the front and side edges, at least one armrest rests on the top, the chair is within 6 degrees of upright, and all four feet clear 35 mm for the final second. For 1, lower the chair until BOTH armrests bear on the tabletop before withdrawing the hand or hands. At both blue bands the lower faces must meet the top, within 4 mm on a side-view ruler. Both hands then stay at least 5 mm clear of every chair surface and the chair stays supported only by its armrests for the final two seconds. A chair released in open air, with the hands visibly clear and neither armrest on the tabletop, earns zero. If a foot stays on the floor or a band is outside the top, award zero.

Record and compare the final continuous one- and two-second intervals. In the side views compare the height of both marked lower faces to the visible table edge. Their blue bands are 10 mm wide. A ruler in the same plane can resolve the 4 mm contact limit and the 5 mm hand clearance. Each armrest section is 28 mm high. In a side-on frame at the blue band, a 4 mm vertical gap is one seventh of that local height. Check the near edge of the tabletop and the lower face at the same band. A view along the far edge can hide an open gap. Inspect both sides. In a floor-level view the four foot bottoms must each clear a 35 mm reference. A calibrated upright line and the straight rear posts show chair tilt in two perpendicular views. A 240 mm vertical ruler displaced horizontally by 25 mm corresponds to about 6 degrees. Combine both orthogonal projections if the chair leans diagonally. At full credit both armrests must take the weight before the last hand withdraws. A recording that hides either support or a decisive gap requires another view.

## Physical reference

The reference grips the rear half of both armrests, raises the chair with its seat facing up and advances in short stages. The armrest ends can begin bearing before the bands reach the tabletop. After both bands enter, the arms lower the chair. The left hand holds its armrest while the right hand moves behind the chair. The right hand presses the rear crossrail inward and steadies the chair as the left fingers slide out. With both armrests flat and bearing, the right hand withdraws. The four feet remain clear of the floor. Adjacent closed-grip control segments preserve the commanded joint deflections while measuring the real joints to compute the next target. No attachment carries the free chair.

The [Jennifer 78 furniture catalog](https://www.inputinterior.fi/wp-content/uploads/2018/11/prislista-ovrigt-sortiment-e-miljo-och-lobby-input-interior-sono-skola.pdf) supplies the selected chair dimensions, mass and intended suspension. [IKEA US shows upright hanging for sweeping after dinner](https://www.ikea.com/us/en/p/yngvar-chair-anthracite-80417636/). [Bruunmunch specifies the same armrest-supported cleaning arrangement](https://bruunmunch.com/en-eu/products/play-arm-chair-tube-fabric). The commercial products have different shapes. The geometry used here is original. The [arm manufacturer](https://i2rt.com/products/yam-6-dof-arm) rates one standard YAM for a nominal 2 kg payload. With the chair centered, its 3.5 kg static weight is shared across both arms during the lift. The recorded trials use the articulated simulator.

Full-size 3.5 kg birch furniture with two fixed armrests. The commercial Jennifer 78 gives the mass, 510 mm width, 360 mm seat diameter and 450 mm seat height. The test chair uses straight 404 mm armrests within a 404 mm deep frame. The catalog lists a 440 mm overall depth. Rear profile, surface friction and inertia are estimates.

## Referee camera views

The recorded view includes the three stock station cameras and four fixed optical views. The policy receives the stock observations. Camera positions and look-at targets are expressed in metres in the station frame, with elevation measured from its table datum. Lens focal lengths are in millimetres.

- Left armrest camera at [0.26, 0.585, 0.28], aimed at [0.535, 0.235, 0.198], focal length 27.0 mm.
- Right armrest camera at [0.26, -0.585, 0.28], aimed at [0.535, -0.235, 0.198], focal length 27.0 mm.
- Four feet camera at [0.105, -0.75, -0.27], aimed at [0.515, 0, -0.41], focal length 25.0 mm.
- Whole chair camera at [-0.53, -0.82, 0.7], aimed at [0.4, 0, -0.065], focal length 20.0 mm.

## Recorded evaluation

Dream-1 run: 0 (908 steps); demo: 1.0 (908 steps). The scripted and model trials used seed 0, 20 Hz and the same frozen scene. Measured runtime including startup was 398.0 seconds for the demo and 352.96 seconds for Dream-1.

Install the [pinned dual-YAM station](https://github.com/kairosiann/yam-isaac-lab-arena/tree/bd62771e60320433893bd1d2df0fc79b5ddcd6d1) on Linux, source `yam.env` and install `inspect-robots==1.0.0`, `inspect-robots-yam==0.38.0`, `dreamlabs-client==1.0.14`, `numpy`, `scipy`, `pillow`, `imageio[ffmpeg]` and `pytest`. Use the Inspect Robots Dream adapter at revision `b5e936a201cec66c54c297e522c920d98445e339`. Set the Dream credential privately. Run from this task directory with fresh output paths.

```sh
python -m pytest tests
python -m task_impl.run scripted --task . --out /tmp/robocurve-scripted
python -m task_impl.run dream --task . --out /tmp/robocurve-dream
```

An earlier Dream-1 session expired. The submitted model video records one complete new trial from the same initial scene, seed and control horizon.
