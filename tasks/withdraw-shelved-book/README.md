# Withdraw a tightly shelved book by exposing both covers

## Measured native simulation results

| Policy | Score | Control steps | Runtime including startup |
|---|---:|---:|---:|
| Dream-1 | 0 | 1200 | 387.08 s |
| Scripted demonstration | 1 | 1060 | 369.36 s |
| Independent scripted repeat | 1 | 1060 | 374.41 s |
| Hold initial pose | 0 | 1200 | 357.86 s |

The Canadian Conservation Institute describes how to remove a book when its fore-edge is inaccessible. Push back the neighbours, take hold of the exposed boards around the middle of the spine and ease the volume out. Pulling on the headcap can damage it. Use rigid expendable teaching volumes for this task. [CCI Note 11/7, Basic Care of Books](https://www.canada.ca/en/conservation-institute/services/conservation-preservation-publications/canadian-conservation-institute-notes/basic-care-books.html)



## Fixture



Use the single folder `withdraw-shelved-book`. Dimensions and coordinates below are in millimetres, in X, Y, Z order. X increases into the shelf, Y to the operator’s left and Z up from the tabletop. All bodies start unrotated. The YAM bases are at (0, +305, 2.5) and (0, −305, 2.5).



| Object | Construction and placement | Mass and friction |

|---|---|---|

| Shelf bay | Bench-mounted birch plywood, origin (435, 0, 0) at the lip. Plinth 173 × 110 × 65. The 173 × 8 × 188 walls have centres at (521.5, ±51, 159). The 8 × 94 × 188 back has centre (604, 0, 159). The interior ends at X = 600. | 1.10 kg, 0.32 |

| Roof | Screw-fixed polycarbonate, 173 × 110 × 8, centre (521.5, 0, 257). Underside Z = 253. | 0.18 kg, 0.35 |

| Tan books | Two free closed laminated board books, each 125 × 30 × 180. Centres (497.5, +31, 155) and (497.5, −31, 155). | Each 0.160 kg, 0.36 |

| Blue book | Free closed volume of the same size, centre (497.5, 0, 155). The partitioned casing, spine and page block fill the envelope around the three bonded pieces below. | Assembly 0.160 kg. Core 0.134 kg, 0.36 |

| Blue board regions | Two flush 121 × 5 × 60 solid board pieces at (499.5, ±12.5, 155). Each begins 4 mm behind the spine. | Each 0.012 kg, 0.60 |

| Blue headcap region | Solid 10 × 30 × 12 crown centred at (440, 0, 239). Three fixed joints bond the crown and the two board regions to the core. | 0.002 kg, 0.40 |

| Mat | Bench-fixed rubber-faced support, 170 × 220 × 50, centre (295, 0, 25). Its top spans X = 210–380 and Y = −110 to +110 at Z = 50. Its long axis runs across the bay. | 0.82 kg, 0.48 |



## Reset and manipulation



Stand the books on the shelf floor at Z = 65, blue in the middle. Align all spines to X = 435 within 0.5 mm with a removable straightedge. Use four removable 1 mm shims across the row. The nominal clearance is 8 mm overhead and 40 mm behind each book. Remove the gauges, check independent sliding, then restore the flush start. On the physical fixture check that a finger cannot acquire the protected grasp in the flush row or with only one side exposed. The simulator starts from its standard automatic robot reset. Its first scripted segment stages the left tips 60 mm ahead of the left tan spine at (375, 31, 155) and parks the right tips at (205, −320, 265). Both target a 4 mm opening in this segment. These are approach targets.



The reference motion pushes each tan book at mid-height and withdraws the fingers before traversing the shelf opening. The working jaws then enter the exposed space, close on the boards and draw the book clear. Incidental non-headcap body contact during a supported extraction is permitted.



The reference placement moves the gripped book over the mat and rolls its long dimension across the 220 mm width. It lowers the near-horizontal book, opens at the fingertip target (255.4, −8, 86.1), withdraws the lower finger toward X=190 and raises the hand. The actual score uses the observed release and stable bearing.

The fixed plywood plinth has its own rigid-body contact reporter named shelf_floor. Its mass and the wall assembly mass sum to the shelf mass listed above. The side and back walls belong to the separate bay body.

## Score a real trial

**Record the trial.** Use a fixed overhead view through the clear roof to see the shelf lip, the books and all four mat edges. Use synchronized front-oblique close-ups from opposite sides of the opening to see each jaw meet its cover, the top of the spine and the book meeting the mat. Place a millimetre scale parallel to X at the height of the spine midpoint. Before reset, check that a 3 mm displacement can be read in the close-ups. Keep the original timestamps and record continuously from the flush start. After release, continue filming as the hands withdraw. End the trial when every final-placement condition below has held for two uninterrupted seconds, or at 60 seconds from the start, whichever comes first.

**Mark the contact regions on the expendable blue book.** On each outer cover, outline the band 60–120 mm above the book's bottom edge and 4–125 mm behind its spine. Outline the headcap across the full 30 mm thickness, in the top 12 mm and first 10 mm behind the spine. These marks move with the book. At the grasp, check that the gripper's tool centre is within 50 mm of the spine midpoint using the scaled views and a visible reference mark on the gripper.

**Find the first pull.** Track the blue spine midpoint along X. Find the first frame at least 3 mm outward from its furthest inward position so far. Review back to the last frame at that inward position. A smaller movement that returns there allows a fresh setup. At departure, each tan book's inner spine edge must be behind the shelf lip and behind the 4 mm start of the adjacent blue band, exposing a visible strip on each side. Either gripper and either pad ordering qualify. In the two close-ups, one pad must already be seated against each exposed band with no visible gap. Review every frame from departure through the 3 mm crossing. Through the crossing frame, both pads must stay seated on opposite bands, the tool centre must remain within 50 mm of the spine midpoint, and the book must move with the jaws. The continuous opposed-grasp check ends at that frame.

**Assign one score.** Apply the whole-trial failure rules below, then award the highest row whose conditions the footage verifies. Record the exposure frame, departure and 3 mm crossing frames, clearance frame, or start and end of the final dwell, as applicable.

| Score | Observable milestone |
|---:|---|
| 0 | The footage shows a whole-trial failure, or the trial ends before any listed milestone is achieved. |
| 0.25 | Both tan books stand on the shelf floor and expose the two cover bands as defined above before the first pull. |
| 0.50 | Both sides are still exposed at departure and the opposed grasp passes the full first-pull check. |
| 0.75 | After the verified first pull, a frame shows the entire blue book in front of the lip (every corner at X < 435 mm) and either held by a gripper or resting on a support surface. |
| 1.0 | The exposure and opposed grasp pass the first-pull check. Throughout the final two uninterrupted seconds, one broad cover rests flat on the mat top at Z = 50 mm. The entire outline lies within X = 210–380 mm and Y = −110 to +110 mm. Both tan books rest on their bottom edges at Z = 65 mm, within 3° of vertical and inside the bay footprint at X = 435–600 mm and Y = −47 to +47 mm. All three books remain visibly stationary. Both hands are disengaged, with each tool centre at least 160 mm from the blue book centre and 180 mm from the row reference point (520, 0, 155) mm. |

Review the marked headcap from reset to the end of the trial. Score 0 if an arm or jaw visibly hooks, presses on or pulls against the headcap. Look for a finger catching the upper spine as it draws the book outward, or an arm bearing down against the crown. A passing brush alone is incidental contact. Also score 0 if the first 3 mm pull fails its exposure or opposed-grasp check, even if a later grasp succeeds. After a valid first pull, a change of grip is allowed. Judge the tan books’ final support and uprightness during the two-second dwell, including when a neighbour has been righted earlier. If every view obscures a required safety check, record the trial as unscorable and repeat with that interface visible.

## Recorded simulation scoring and measured contacts

At control sample 475, the tan books had moved inward by 33.11 mm and 32.79 mm. The left gripper's opposing pads already loaded the two cover regions at 2.385 N and 2.483 N. Outward travel first exceeded the numerical noise band at sample 476 and reached 3 mm at sample 490, where the two measured pad forces were 4.327 N and 5.128 N. Opposing contact above 0.10 N per pad and the mid-spine grasp-distance condition held at every intervening sample. The scorer accepts either gripper and either pad ordering. It checks both neighbouring setbacks and gaps before the first significant outward motion and monitors headcap loading throughout the episode. The left and right arm-to-headcap peaks in this demonstration were 0.000 N and 0.000 N. A reading of 0.05 N or more on either arm-to-headcap channel invalidates the simulated trial.

During the final two seconds, bearing_l measures tan_left against shelf_floor and bearing_r measures tan_right against shelf_floor. Wall reactions are excluded from these two force measurements. Both neighbours also have to stand upright with their bottom edges at the floor and their corners in the bay. The four reactions from the blue core, each cover and the crown against the mat are summed at each sample. The required total is at least 0.10 N. The blue book must lie flat, with all eight physical corners inside the mat, all three books quiet and both grippers disengaged and clear.

All 62 final measurement bounds and the two onset prerequisites held throughout samples 1021–1060. The ranges below cover all forty consecutive control samples.

| Measurement | Minimum | Maximum |
|---|---:|---:|
| Total book-to-mat reaction (N) | 1.5696 | 1.5696 |
| Left neighbour on floor (N) | 1.5696 | 1.5696 |
| Right neighbour on floor (N) | 1.5696 | 1.5696 |
| Book up-axis angle (degrees) | 90.0000 | 90.0000 |
| Book depth-axis angle (degrees) | 90.0000 | 90.0000 |
| Book centre height (mm) | 65.0000 | 65.0000 |
| Left hand to book centre (mm) | 160.9941 | 203.4299 |
| Right hand to book centre (mm) | 385.8837 | 385.8837 |

The full condition ranges, continuous onset samples, per-episode headcap peaks, sensor definitions and trace hashes are in evidence/lead-evidence.json. From the extracted archive, run `python evidence/recompute_lead.py .` to reproduce them. Contact-force thresholds and the noise floor are engineering settings for the rigid simulator. The recorded simulation scorer awards 1 for the complete finish and 0 otherwise. The real-trial scale above uses visible contact, motion and support.



## Reproduce the measured trials

Install the pinned [YAM station](https://github.com/kairosiann/yam-isaac-lab-arena/tree/bd62771e60320433893bd1d2df0fc79b5ddcd6d1) on Linux with an RTX GPU, then source its yam.env. From this task directory run the following commands. Seed 0, 20 Hz.

```sh
uv pip install inspect-robots==1.0.0 inspect-robots-yam==0.38.0 dreamlabs-client==1.0.14 numpy scipy pillow "imageio[ffmpeg]" pytest
uv pip install "inspect-robots-dream[client] @ git+https://github.com/kairosiann/inspect-robots-dream@b5e936a201cec66c54c297e522c920d98445e339"
python -m pytest tests
python -m task_impl.run scripted --task . --out /tmp/robocurve-demo
python -m task_impl.run dream --task . --out /tmp/robocurve-model
```

The model run uses the official Dream-1 checkpoint with a configured Dream credential. Every output directory must be new. Set DREAM_API_KEY in your private shell environment. The tested endpoints were DREAM_GATEWAY_URL=https://api.dreamlabs.ai, DREAM_API_URL=https://platform.dreamlabs.ai, and DREAM_DATA_URL=https://data.dreamlabs.ai. Recording to the model account is disabled. Measured runtime on RTX 4090 including scene startup was 369.36 s for the scripted run and 387.08 s for the model trial.
