# Safety closure, counted dispensing and scoring

The robot starts with a closed bottle. The benchmark uses the dimensions of a retail stock bottle and cap.
A bayonet mechanism models the need to push and turn a child-resistant closure. Its travel, spring and
angular thresholds are simulation parameters. Retail packaging may use a two-shell clutch and a screw thread.

## Geometry and physical sequence

Two opposed lugs on the cap overlap retaining ledges on the bottle neck. With the cap seated,
a counterclockwise stop extends 1.7 mm below the lug's upper face. A deeper stop blocks clockwise motion.
An annular collar on a 4 mm downward-travel prismatic joint bears against the edge of the skirt.
Its axial spring has stiffness 110 N/m and damping 0.35 N·s/m. It stays on the bottle throughout the trial.
The cap and bottle interact through their convex colliders. Neither body is attached to a gripper.
There are two 41-degree windows in the retaining ledges, centred at 49.5 and 229.5 degrees.
Matching stripes on the cap and bottle provide references for monitoring relative twist. The cap skirt's modelled inner diameter is 40.6 mm to clear the ledges. Its nominal dispensing region is 36 mm across.

The right gripper steadies the bottle. The left gripper pinches the cap, compresses the collar, turns
the cap counterclockwise and lifts it free. The robot turns the empty cap open side up before dispensing.
During the pour the left gripper holds it clear of other objects and the right gripper tips the bottle.
Returning extras and tipping the cap into the cup are performed with one hand, the left, while the right arm waits.
All 30 tablets move through rigid-body dynamics, including gravity and contact with the containers.

## Verifying a trial

The opening scorer measures translation and rotation in the bottle frame at 20 Hz. Both fingertips on each
of two different grippers must press at least 1 N each on their respective object. From the closed alignment,
the cap must compress at least 1.5 mm and stay compressed by at least 1 mm through 20 degrees of turn.
Its orientation must reach the 34–68 degree release range, then it must rise 15 mm from its initial height
while still above the neck and pinched. Moving the bottle through space with its cap attached leaves these
relative measurements unchanged. The trace records the measured sequence, including failed opening attempts.

Any tablet escaping before the verified opening voids the run. A tablet entering the cup must previously
have lain inside the held cap for at least 0.25 s. With tablets in it, the cap must be at least 5 mm off the
table, with a valid two-fingertip grasp. The scorer treats a support force of 0.02 N or more from the bottle
or cup as external support. The card defines the dose tiers, release and stillness windows. For a hardware
trial, video from above and from the side can show the alignment stripe, axial travel, grip, tablet path
and final inventory. Two fingernail-sized witness marks and a ruler beside the station give distance and angle references.

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
