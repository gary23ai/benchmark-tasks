# Physical scope and grading

The stock calibrated YAM station, robot joint and gripper settings, three RGB policy cameras, 20 Hz actions and 240 Hz CPU physics remain unchanged. RTX rendering runs on the selected GPU. Coordinates below use metres above the 0.7475 m tabletop: +x forward, +y toward the robot's left and +z upward. The arm bases are (0,+0.305,0.0025) and (0,-0.305,0.0025). Each starts at [0,1,1,-0.6,0,0] rad with its gripper at 1 (open). Policy state and actions contain the left arm's six joints and jaw followed by the right arm's six joints and jaw. Policy cameras are top_cam, left_cam and right_cam at 320 x 176 RGB; the fourth side camera is review evidence only.

The wheel is 240 mm in diameter with a 38 mm wide tire, compound convex rim and six 26 x 75 x 20 mm spoke collision envelopes. Its estimated 580 g mass represents a hollow molded wheel and foam/rubber tire, not a homogeneous solid made from every collision envelope. Tire deformation and molded internal cavities are omitted. The 12 mm diameter steel spindle is 121 mm long and weighs an estimated 110 g, with its center of mass 60.5 mm along its axis. Its 8 mm root-diameter groove is 5 mm wide; its far end has a 5 mm explicit tapered chamfer. The spindle and wheel share an unrestricted native revolute bearing. These standalone body joints have no claimed Coulomb joint friction: the installed schema limits `physxJoint:jointFriction` to articulation joints. Only explicit body damping remains.

The wheel and spindle body origins both start at (0.360,0,0.150), with identity XYZW quaternion (0,0,0,1), axis +x and zero velocities. The wheel uses a rigid molded-rim proxy and rubber/foam tire contact material; the spindle and retaining tip use steel proxies. The spindle body origin is at the wheel plane, not its center of mass, which is 60.5 mm along +x. Two receiver lands create a real 12.6 mm bore; a separate slotted housing admits the lower steel retaining tip. The 25 g release slider starts at (0.445,0,0.134), identity orientation and zero velocity. Its tip is 3 mm axially by 5 mm transversely by 9 mm vertically, with its top 4.3 mm below the spindle axis at rest. The attached 18 x 18 x 12 mm pull tab is 52 mm laterally offset and 6 mm below the slider origin. A native prismatic guide permits 8 mm downward travel. Its fixed 400 N/m spring, 2 Ns/m damping and +1 mm target above the upper stop create a nominal 0.4 N preload. These estimates model a passive spring; no policy changes its target or parameters.

The aluminum frame leg is 40 x 40 x 170 mm, centered at (0.530,0,0.097), on a 110 x 140 x 12 mm bench-clamp base centered at (0.530,0,0.006). The receiver and frame are fixed parts of the already supported hub module. The rigid receiving rack is secured to the workbench at (0.510,-0.245). It has a 66 x 270 mm base, 4 mm floor, 43 mm internal axial clearance, 90 mm cheeks and two 24 mm wide, 10 mm tall saddles. The wheel's nominal resting center height is 131 mm. There is only one wheel and one initially empty rack; no replacement wheel or spare support is present. The hub module is safely held by a bench clamp, a realistic service setup that excludes handling a whole stroller or passenger.

All hardware dimensions, masses, inertias, friction, spring values and break-free fit are engineering reconstructions. The consumer operating sequence is supported by the manufacturer, but the geometry is not a dimensional replica. Wear, dirt, elastic deformation, production tolerances and whole-stroller safety are outside the simulation.

The read-only grader observes three native body poses and velocities and all robot-to-task contact counts at every 240 Hz step. A retainer retraction of more than 1.8 mm must precede withdrawal. The spindle's actual far tip must clear the receiver's front by 8 mm. Completion then requires the wheel upright in the rack, its center within 6/12/9 mm in x/y/z, actual axis x component above 0.995, wheel and spindle linear speeds below 10 mm/s, wheel angular speed below 0.25 rad/s, retainer returned within 0.3 mm and every robot-task contact absent. These conditions must hold for one elapsed second: the first observation contributes zero elapsed time, so at least 241 consecutive physical samples are required. Separating the bearing, departing the slider guide, forcing withdrawal before release, invalid states or dropping below the bench void the trial. Partial credit is 0.25 for release, 0.5 for complete withdrawal and 0.75 for storage. Only the full released stable endpoint earns 1.0. No scripted phase or policy-reported completion enters the scorer.

A real counterpart can be reviewed with the visible release lever, actual axle clearance, upright supported wheel, returned lever and an uncut view showing hands removed for one second. This is a bounded removal operation that exposes the hub and spindle for servicing. It does not assert that cleaning, repair or refitting has occurred.

# Novelty screen

The complete accepted, pending, revision and rejected RoboCurve snapshot dated 19 November 2026 at 20:08:48 UTC and all 1,156 EFE records were searched. The following nearest entries were read in full, including instructions, scoring and available review feedback:

- Accepted TK-LGTDUX rocks a complete pram through its handle; wheel service is different.
- Pending TK-S8Z33V handles a garden-hose twin-pin sleeve coupling and pull check. A coupling-release subskill is shared; this operation supports a freely rotating full-size wheel and withdraws a long cantilevered spindle from an actual bore before placing the assembly in a maintenance rack.
- Accepted TK-XDB5VZ removes a camera plate from a cam clamp and retaining lip; accepted TK-VCAYTT adjusts a bike saddle with an over-center cam.
- Pending TK-NE9FGT squeezes two drill-battery latches while sliding a pack on rails, rather than freeing a steel spindle from a spring-retained bore.
- Rejected TK-CDMS99 uses a custom quick-release blade holder with weak real-world motivation. Here the release mechanism and maintenance operation have a consumer manufacturer reference.
- Rejected TK-KQSSBV changes a grooming cartridge and was considered a duplicate of a toothbrush-head task. Changing the object name alone would not establish novelty; the actual bearing, bore withdrawal and supported wheel handling are disclosed as the relevant mechanics.
- Rejected TK-78P283 removes a miniaturized head restraint using relocated geometry. This task retains a 240 mm wheel and ordinary service positions; fixed remote frame pieces are omitted rather than scaling down the mechanism.

No direct stroller rear-wheel removal task was found in that snapshot. Shared release/coupling skills remain a review consideration, and this screen does not guarantee acceptance.

# Collision geometry and implementation

Tire, rim, receiver lands and slotted housing use compound convex annular sectors around real openings. The spindle end is a 12-sided tapered frustum. Custom vertices are authored in millimetres with a compensating 0.001 local scale, leaving world dimensions in metres. This conditions native hull cooking without enlarging the objects. Meshes and boxes use 0.15 mm contact offsets and zero rest offset; analytic cylinders retain the runtime defaults. Native cook inspection must use the final authored asset without temporary geometry overrides.

The scripted reference reads actual bodies and robot joint FK, closes a feedback loop through ordinary IK joint targets, and handles the physical wheel. This privileged development access does not alter the standard RGB policy interface. All body pose and velocity writes occur at reset only. The scorer never changes joints, moves bodies, applies forces or uses policy phases.


# Recorded model evaluation and evidence

The current [validation](validation.json) combines the recorded scripted reference with the later fixed-start Dream-1 beta evaluation. Scripted runs reached 1.0 in 604/700 actions, with 7,248 native samples each. The model run completed all 700 permitted actions, or 35 simulated seconds, and scored 0.0 without a task void. Every one of its 8,400 native samples confirms zero robot contact with the wheel, spindle and release slider. The release threshold was never reached, the spindle stayed in its receiver and the rack remained empty. This describes an unsuccessful removal attempt; it does not claim a failed replacement operation.

The model received the exact frozen instruction and the same scene, scorer, control rate, seed 0 and action limit as the scripted runs. It used the stock Dream policy, 32 preroll steps and 48-row chunks. There were 27 action attempts, 14 successful chunks and 13 service-session expirations. The standard SDK adopted each replacement session and retried with the preserved nine-frame RGB window; service memory continuity is therefore a limitation. Standard bounds clamps changed 144 of 700 targets, only on right_j2, with maximum adjustment 0.0330444188 rad. The delta limiter made no changes. These effects are part of the recorded integration conditions, not changes to the task scorer.

The [repository result summary](evidence/dream/RESULT-SUMMARY.json), [native input replay](evidence/AUDIT-INDEX.json), [model-input and action provenance](evidence/AUDIT-INDEX.json) and [visual review](evidence/AUDIT-INDEX.json) bind the recorded outcome to the frozen source. Replay covers every native grade, every actual joint observation and executed target, every uploaded JPEG and selected nine-frame window, the stock SDK filter and chunk queue, and the original guards.

The [full model video](../../uploads/remove-a-stroller-rear-wheel-for-servicing/model_run.mp4) retains the complete native chronology with a synchronized overhead inset on the side view. Its 351 frames at 10 Hz include the initial and terminal states, last PTS 35.0 s; the 35.1 s file duration includes the final displayed frame. There are no temporal cuts, speed changes or generated scene content. The [success demonstration](../../uploads/remove-a-stroller-rear-wheel-for-servicing/demo.mp4) and [initial screenshot](../../uploads/remove-a-stroller-rear-wheel-for-servicing/setup.png) accompany it. One model trial is not a success-rate estimate. No hardware validation or external submission is claimed.

# Reset and reproduction

Each validated run begins in a fresh process. At episode reset, restore the three task-body poses and zero velocities listed above, return both arms to READY and allow the unchanged runtime to settle. The spring target stays constant; the supported hub and empty rack remain fixed. There are no post-reset task-body transform writes. Installation, the full-budget scripted command, counterexample tests and asset-reproduction check are given in [README.md](README.md). Historical archive-reader checks supplement the source-bound records; they are not substitute robot trials or a required runtime step.

## Portable evidence scope

Descriptions of formal archives, prior prototypes or retention diagnostics above
refer to the original recording history, which is retained separately. This
portable folder includes current validation, original-result hashes, explicitly
marked result summaries and audit metadata in `evidence/`; it does not claim that
omitted raw native traces can be replayed from those summaries. Use the commands
in README.md to run and score a fresh trial. The exact recorded protocol and
service/guard limitations are in [EVALUATION.md](EVALUATION.md).
