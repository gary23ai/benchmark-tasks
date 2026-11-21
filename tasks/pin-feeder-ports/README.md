# Refit both feeding ports in a bird feeder

## Measured native simulation results

| Policy | Score | Control steps | Runtime including startup |
|---|---:|---:|---:|
| Dream-1 | 0 | 600 | 220.51 s |
| Scripted demonstration | 1 | 506 | 214.36 s |
| Independent scripted repeat | 1 | 506 | 211.63 s |
| Hold initial pose | 0 | 600 | 197.12 s |

Dream-1 held its measured starting pose for 32 control steps. The broker served 12 of 13 action requests. The final chunk began at step 561 and supplied actions through the last recorded step, 600.

The design notes below were recorded before these native trials. The table above and validation.json report the measured results.

<details><summary>Original pre-trial design record</summary>

# Refit both feeding ports in a bird feeder

Task folder `pin-feeder-ports/`. A feeder owner removes the ports for cleaning and replacement. The manufacturer's assembly steps 2–3 call for holding the opposed ports together and passing the ring rod through them. Raising the rod releases the pair. This dry service fixture isolates one pair and gives each port a shoulder that can be inspected and pulled individually. [Classic Ring Pull instructions](https://www.perkypet.com/media/wysiwyg/pp/pdf/Classic-Ring-Pull-Sunflower-or-Nyer-Seed-Feeder-0815XX.pdf)

## Drawing and reset

Dimensions below are in millimetres. Table X points forward, Y left and Z up. The feeder datum is the center of the base top at table coordinates (360, 0, 8). The object positions below use that datum. All body frames start unrotated. The left and right YAM bases are at table coordinates (0, +305, 2.5) and (0, −305, 2.5). The ports run along Y. Face the inspection window along X toward the camera.

| Object | Construction and starting position |
|---|---|
| Feeder, 240 g | A bench-mounted 90 × 80 × 8 aluminum plate supports a 90-high faceted polycarbonate sleeve, 56 across its outer flats with 2-thick walls. The upper and lower acetal guides span radial distances 20–28 and each has a 16.4 × 4.4 passage. The bridge spans 56 along X at heights 84–94. Its split 6.6-wide guide admits the shaft and leaves space for the friction shoes. |
| Orange port, 5 g | The upper acetal tongue is 36 long, 16 wide and 4 thick. Its closed faceted eye measures 10 across flats and starts at (0, +13, 61). At the seat, the tongue extends 8 past the axis inward and 28 outward. Its 20-wide, 24-high, 2-thick shoulder has an outer face 30 from the axis at the seat. |
| Blue port, 5 g | The opposing acetal tongue and shoulder have the same dimensions. The lower eye starts at (0, −13, 49). Both shoulder centers are 55 high. Each independent slide permits 24 of outward travel from the seat. The tongues have 8 of vertical clearance where they overlap. |
| Ring rod, 18 g | A blunt stainless shaft, diameter 6, extends 53 from its tip to the underside of an 8 × 12 × 2 foot. The bonded ring lies in Y–Z, is 6 thick along X and measures 24 across outer flats and 12 across inner flats. Its top flat is 6 × 9.94. The tip starts at (0, 0, 73) and finishes at height 41 when the foot bears on the bridge at 94. |
| Two friction shoes, 1 g each | Each rigid dry guide shoe measures 5 × 4 × 4 and starts at (0, ±5.02, 89). Each 850 N/m lateral spring takes up a 0.02 initial gap to the shaft, giving about 0.71 N nominal preload per shoe. The modeled shoe friction coefficient is 0.65. The rod and port slides have zero spring stiffness. |

Use the fixed layout with seed 0 and the stock dual-YAM arm reset. Empty the sleeve and leave the cap off. Raise the rod first, to its upper stop. Set each shoulder with a 13 mm spacer between shoulder and seat. Remove both spacers. The span across the outer faces is then 86 mm and the tip clears the upper tongue by 10 mm. Check that the rod and both shoulders hold these positions hands-free.

## Scripted manipulation

Each hand approaches its own shoulder from outside the feeder with fingers pointing along the slide. The 4 mm jaw opening places the fingertips over the shoulder face. The hands advance from 43 mm off axis to a target 29.6 mm off axis, 0.4 mm inside the nominal seated grip plane. The right hand holds its configuration at the blue shoulder. The left hand retracts along +Y before rising outside the sleeve.

The left hand presses the crown from the orange side. It pitches the fingers 15° below horizontal toward −Y, keeps the jaw axis along X, and sets a 2 mm gap before crossing above the ring. The target is centered at table X = 360, Y = −2. That Y coordinate falls on the 9.94 mm crown flat. In projection the 6 mm crown extends 2 mm beyond each inner jaw plane. These dimensions locate the intended bearing surface. Actual contact depends on the installed fingertip profile and achieved wrist pose.

The raised crown surface is at table Z = 159.5 and the seated surface is at 127.5. The fingertip target descends from 169.5 to 161.5, then to 158.5 for contact take-up. Four 8 mm downward segments finish at 126.5, with 1 mm nominal overtravel against the working stop. All press targets use the table frame. The intended release is a vertical lift off the crown, followed by outward withdrawal of both hands. The 499 scheduled ticks leave 101 ticks at the final targets. The operator may use any sequence that produces the released assembly.

## Scoring and bench checks

The binary grader requires 40 consecutive samples at 20 Hz satisfying every condition. Each shoulder gap must lie between −0.5 and +3 mm, with the negative allowance covering numerical contact tolerance. At both faces of both eyes, the horizontal eye-to-shaft offset must be at most 2 mm. This equals the eye inradius less the shaft radius along the keyed slide. The foot gap must be within ±0.5 mm of flush, and the tip must project 5.5–6.5 mm below the lower tongue. Each arm's normal contact with each port and the ring rod must be at most 0.01 N throughout the hold. An interrupted hold can start again.

On the bench, record the released hold first. Read each shoulder with a seat feeler. Check the foot against the bridge and measure tip projection from the lower tongue with a depth reference. Then pull each shoulder outward separately along its slide using a spring scale. Read displacement from that port's own released position with a ruler pointer and check that the foot stays down. These are evaluator measurements.

Calibrate the pointer provisionally at 1 N using a correctly pinned pair, each free port with the rod raised, and each one-eye miss with the foot flush. To produce a miss, seat one port, leave its mate 13 mm out, and lower the rod. The shaft clears the missed nose by 2 mm. Pushing that nose back against the lowered shaft leaves its shoulder about 11 mm out, with 13 mm of outward travel still available. Repeat on the opposite side at the same foot height. A captured port has about 2 mm nominal outward play from its seat. Check for at least 8 mm of free travel under the calibration pull. If retained and loose travel ranges separate after reading uncertainty, place a broad cutoff mark midway between their uncertainty-adjusted bounds and check its readability on both sides in the stock 320 × 176 recordings.

## Environment and evidence

Use the calibrated dual-YAM PhysX environment with gravity, RGB and robot-joint observations, normal joint targets and normalized gripper-width actuation. The declarative trajectory specifies openings in metres. The limit is 600 control steps at 20 Hz, including the released hold. Rigid facets approximate the sleeve, bores and ring. Collision openings provide inspection windows in the rendered sleeve.

```sh
python -m pytest tests
python -m task_impl.run scripted --task . --out output/demo
python -m task_impl.run dream --task . --out output/model
```

A shoulder can recoil after the hand withdraws. A fingertip can pass beside the crown or catch a sloping ring facet. Excessive guide drag can stall the rod with both eyes aligned. Before treating the route as demonstrated, check the achieved fingertip position and jaw axis at approach, contact, stop and lift. Inspect contact on the crown through the stroke and the swept clearance to the bridge and opposite arm. The supplied cases exercise scorer logic only. Revised installed-arm residuals, reset stability, pull discrimination and safe completion within the limit remain unmeasured. Independent trials supply the authentic model video, real full-score demonstration as needed, scores and screenshot.

</details>


## Reproduce the measured trials

Install the pinned [YAM station](https://github.com/kairosiann/yam-isaac-lab-arena/tree/bd62771e60320433893bd1d2df0fc79b5ddcd6d1) on Linux with an RTX GPU, then source its yam.env. From this task directory run the following commands. Seed 0, 20 Hz.

```sh
uv pip install inspect-robots==1.0.0 inspect-robots-yam==0.38.0 dreamlabs-client==1.0.14 numpy scipy pillow "imageio[ffmpeg]" pytest
uv pip install "inspect-robots-dream[client] @ git+https://github.com/kairosiann/inspect-robots-dream@b5e936a201cec66c54c297e522c920d98445e339"
python -m pytest tests
python -m task_impl.run scripted --task . --out /tmp/robocurve-demo
python -m task_impl.run dream --task . --out /tmp/robocurve-model
```

The model run uses the official Dream-1 checkpoint with a configured Dream credential. Every output directory must be new. Set DREAM_API_KEY in your private shell environment. The tested endpoints were DREAM_GATEWAY_URL=https://api.dreamlabs.ai, DREAM_API_URL=https://platform.dreamlabs.ai, and DREAM_DATA_URL=https://data.dreamlabs.ai. Recording to the model account is disabled. Measured runtime on RTX 4090 including scene startup was 214.36 s for the scripted run and 220.51 s for the model trial.
