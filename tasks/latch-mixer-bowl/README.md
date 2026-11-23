# Mount a bowl on the side pins of a bowl-lift mixer

Fit the empty bowl onto the lowered mixer arms. Seat both side tabs over their pins, press the back of the bowl into the spring latch, and let go. The two locating pins can be visibly seated while the rear stud remains perched on the catch. The mounting motion follows the ordinary bowl-lift procedure documented by KitchenAid. [Manufacturer's bowl-lift procedure](https://producthelp.kitchenaid.com/Countertop_Appliances/Stand_Mixers/Product_Info/Stand_Mixer_Assembly_and_Operation/Assemble_and_Adjust_a_Bowl-Lift_Mixer).

## Construct and reset the bench apparatus

Build the unpowered replica, its matching bowl and its actual sliding inspection catch from the [fabrication guide](fabrication/BUILD.md). The guide supplies seven fabrication-ready solids, a millimetre shop drawing, a passive spring rate and preload, fastening details, a body-weight check and a pre-run check for actual retention and release. The fixed motor silhouette, front drive shaft, lowered cradle and two open windows are retained. The permanent internal steel ring sets the replica bowl's total mass. The external ears and stud are integral with the rigid bowl.

Use the unpowered mixer replica and matched bowl from fabrication/BUILD.md. Bolt the lowered cradle to the bench. Leave the red fixed head and tool shaft fitted with the beater removed. The steel 6.4 mm pins are 246.4 mm apart and their shoulders are 128 mm above the bench. Set the 0.60 kg weighted empty bowl on its foot 176 mm ahead of the pin line, handle toward the operator. The orange catch rests at its forward stop. Support the bowl by its handle or rim, retract the guided spring catch, lift the rear stud clear and lift both ears off their pins. Release the catch and return the bowl to its foot mark.

Positive x points from the operator to the rear column. Positive y points left. The datum is the midpoint between the pin axes at shoulder height. The two yellow shanks are 6.4 mm in diameter. The 4.8 mm thick ears have 10.4 mm across-flats octagonal holes. The rear cross-stud is 12.8 mm in diameter with blue end faces. The nominal spring preload is 2.56 N, its rate is 0.20 N/mm and rear travel is 12.8 mm.

## Human and native scoring

Score the final uninterrupted second, or finish early when the 1.0 state lasts a second. Use synchronized views of both yellow pin tops, the left and right rear pocket windows, and both hands. Score 0.5 if each yellow pin passes through its own closed tab and projects 5–9 mm above the top face of the tab (7.6 mm with the tab resting on the shoulder). The tab top lies between the two black height rings on the pin. Use the 6.4 mm steel shank for scale. Both hands must be visibly clear of the bowl and spring catch throughout that second. Hands can finish above, in front of, beside, or between the tabs wherever they have let go. Judge actual contact with the bowl or catch. Score 1 if during the same second both blue stud ends sit entirely below and behind the orange lip in the respective pocket windows, and the collar stays within the 3 mm mark at its forward stop. The stud centre must be 5–15 mm above the pocket floor and 5–15 mm behind its front face at each window. A stud on the ramp or a retracted catch caps the score at 0.5. One missed tab or continued hand support earns zero. A pin touching the inside wall of a closed ear and a blue stud touching the underside of the lip both count when their visible positions meet these marks. A failed attempt may be retried.

The four synchronized close-ups show one ear or one pocket window each. The two black rings on each shank mark the acceptable tab-top heights. The two sill ticks and two jamb ticks are 10 mm apart. The collar has marks at its forward stop and 3 mm behind it. Inspect hand contact in the wide and wrist views. An observer may use another synchronized view to see an obscured feature. A trial whose required landmarks stay hidden is unverified.

The native scorer checks the finite bore at both tab faces, the yellow pin exposures, both blue stud endpoints and collar return. It tolerates 0.2 mm at pin contact and the lip underside to accommodate collision-solver boundaries. The four measured full-arm contacts against the bowl and moving catch must be zero throughout the 20-sample interval. Hand position and jaw opening do not contribute points. Earlier attempts can be retried.

## Archived recordings and revised scores

The archived demonstration and Dream-1 episode share the original scene, action interface and camera views. They remain available through media ID M-PCCEKE. In the demonstration both pins are seated while the blue ends remain perched, then the grippers press the rear rim. At 34.4 s the collar has moved back 7.72 mm. In the final second both blue ends are captured, both yellow pins remain exposed and both hands are clear. In the final 20 samples the side-pin exposures are 7.69 and 7.83 mm. The two stud centers are 10.41 and 10.40 mm above the floor and about 6.40 mm behind the marked front datum.

The [validation record](validation.json) retains the original native records and adds a replay of every archived measurement through the revised scoring rules. The demonstration scores 0.5 from sample 502 (25.10 s) on its first qualifying interval. It first scores 1 at sample 761 (38.05 s). The historical video continues to the original end at sample 805. A fresh run using the revised early-stop rule may end at sample 761. The complete original 830-step Dream-1 episode still scores zero. The unchanged video overlay describes pin seating, capture and the one-second release.

## Reproduce the simulation

Install the [pinned dual-YAM station](https://github.com/kairosiann/yam-isaac-lab-arena/tree/bd62771e60320433893bd1d2df0fc79b5ddcd6d1) on Linux, source `yam.env` and install `inspect-robots==1.0.0`, `inspect-robots-yam==0.38.0`, `dreamlabs-client==1.0.14`, `numpy`, `scipy`, `pillow`, `imageio[ffmpeg]` and `pytest`. Use the Inspect Robots Dream adapter at revision `b5e936a201cec66c54c297e522c920d98445e339`. Set the Dream credential privately. Run from this task directory with fresh output paths.

```sh
python -m pytest tests
python -m task_impl.run scripted --task . --out /tmp/robocurve-scripted
python -m task_impl.run dream --task . --out /tmp/robocurve-dream
```
