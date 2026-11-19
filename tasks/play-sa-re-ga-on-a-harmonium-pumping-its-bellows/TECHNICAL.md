# Play Sa-Re-Ga on a hand-pumped harmonium

A standard Indian harmonium (39 keys, 3 1/4 octaves, double bellows) stands on the table with its keyboard toward
two YAM arms. The right hand plays Sa, Re and Ga (C4, D4, E4, marked with coloured dots), each held for at least
4 s and joined legato, the next key down before the last one comes up; the left hand pumps the bellows board at the back the whole time. Then both let go.

Why it needs both hands at the same time: a reed sounds only while air flows from the reservoir inside the case,
and a full reservoir keeps one held key sounding for under 3.1 s (2.96 s in the model). A held note needs 4 s,
so air must be pumped in while the key is down, and the pumping goes on while the other hand changes keys.
Every tier, even the lowest (Sa held 4 s), needs both hands working together. The kinematic dry run checks this
with three one-handed controls through the real scorer:

| Control (`tools/dryrun_demo.py`) | What happens | Score |
| --- | --- | --- |
| `no_pump`: the left hand never pushes the board | the reservoir stays empty; no reed speaks | 0 |
| `pump_only`: the right hand never presses a key | the reservoir is filled (2 strokes), nothing sounds | 0 |
| `prime_then_stop`: one stroke fills the reservoir, then the left hand stops | Sa sounds 2.8 s and dies away | 0 |
| nominal: both hands together | Sa, Re, Ga 5.0, 5.0, 4.45 s, each change overlapping 0.45 s (legato) | 1.0 |

Folder contents:

| Path | What it is |
| --- | --- |
| `task.json` | the task card |
| `rdb_harmonium/layout.py` | every dimension and air-model number, each marked sourced or estimate |
| `rdb_harmonium/harmonium.usda` | the scene asset, generated from `layout.py` by `tools/build_assets.py` |
| `rdb_harmonium/fixed_scene.py` | where the harmonium stands, the instruction, the run limit |
| `rdb_harmonium/air.py` | the air rule: board travel to reservoir, reservoir to reeds (NumPy) |
| `rdb_harmonium/grading.py` | the scorer (NumPy); the same code scores a real trial's audio frames |
| `rdb_harmonium/audio.py` | the real trial's calibrated pitch rule, and a reed-tone synthesis of a simulated trial |
| `rdb_harmonium/skin.py` | draws the bellows' leather folds from the board's angle (display only) |
| `rdb_harmonium/adapter.py` | adds the harmonium to the stock Inspect Robots YAM station, runs the air rule and the scorer |
| `rdb_harmonium/scripted.py` | the scripted demonstration |
| `rdb_harmonium/run.py` | runs one trial and writes the result, videos, synthesized sound and scorer trace |
| `tests/`, `tools/` | unit tests, the dry run, the asset generator, `judge_audio.py` (the real-trial rule on a recording) |

## The instrument, and where its numbers come from

| Quantity | Value used | Source | Status |
| --- | --- | --- | --- |
| Type | hand-pumped Indian harmonium, 39 keys, 3 1/4 octaves, 7 stops, double bellows, double reeds | retail listings of standard 39-key models (eBay, Reverb and Indian marketplaces, read 2026-11-18) | sourced |
| Range | C2 to D5 (front reed bank) | Banjira Standard Harmonium 7-Knob product page, musicfolk.com ("Front Reed Bank: C2-D5", 39 keys, 3 drones, about USD 475) | sourced |
| Case size | 559 x 305 x 254 mm with the lid; 205 mm tall with the lid off | one listing gives 55.88 x 30.48 x 25.4 cm and 8 kg; others 53 to 56 cm long, 23 to 25 cm tall | sourced; lid height (49 mm) estimate |
| Mass | 8.0 kg | same listing (others 7 to 10 kg) | sourced |
| How it is played | one hand pumps the bellows at the back, the other plays; an internal reservoir bellows, pressed by a spring, feeds the reeds | Wikipedia, "Indian harmonium" (left hand pumps; internal reservoir bellows gives a sustained sound); chandrakantha.com harmonium page ("The internal bellows push against a spring; it is this spring which forces the air over the reeds") | sourced |
| Bellows board | 530 x 189 mm, 12 mm thick, hinged along its bottom edge, top edge opening 56 mm | double (two-fold) rear bellows hinged at the bottom, as on standard models; sizes estimated from the case | estimate |
| Force to push the board shut | 11.6 N at its top edge (2.19 N m about the hinge) | the reservoir's working pressure times the board's area: 200 Pa (20 mm water gauge) x 9.5 L/rad, plus a 0.3 N m reopening spring. Parlour reed organs run at 40 to 130 mm water gauge (pumporganrestorations.com, "physics of the pump organ"; a suction instrument, so only an order of magnitude); a portable harmonium is pumped by one hand, so a lighter wind was taken | estimate; measure with a spring balance hooked on the board's top edge |
| Reservoir | 1.2 L between empty and its relief valve, 160 to 230 Pa | no published figure | estimate |
| One open reed | 0.4 L/s at 200 Pa (flow ~ square root of pressure) | no published figure | estimate |
| Full reservoir, one held key | 2.96 s | follows from the two rows above | measure: time one held key after pumping stops (the scoring rule raises the hold time if a real instrument holds longer) |
| Keys | 22 mm white pitch, 100 mm long, tops 180 mm above the table, 7 mm travel, 0.66 to 1.4 N, speaks from 2.5 mm down | estimated from the case size and from photos of standard models | estimate |

No product image, mesh or logo is included; the case carries no maker's name.

Station coordinates (mm; x forward from the arm-base line, y left, z up from the table): the case's front edge is
along x = 140, centred on y = 0, so its back is at x = 445 with the board shut. Key tops (playing point 22 mm
behind the front of the key) for Sa, Re and Ga: (168, -66, 180), (168, -88, 180), (168, -110, 180). The bellows
board's hinge runs along x = 433, z = 16; its outer top edge is at x = 500, z = 193 when open and at x = 445,
z = 205 when shut. The left hand works the edge at y = 225, near the board's left end, where a player's left hand
goes.

Placement and reach. The board's top edge is 0.19 to 0.21 m above the table, above the band where the YAM reaches
straight down. Every waypoint was checked with inverse kinematics from six seeds before building: a hand tipped
25 degrees (pump), or tipped 55 degrees, turned 35 degrees about the vertical and twisted up to 30 degrees
either way about its own axis (keys), from vertical along the line from its base reaches the whole stroke and every key position
with at least 19 degrees of joint margin, while straight-down hands reach neither. The harmonium stands at its
real size; only its place on the table (front edge 140 mm ahead of the arm bases) was chosen for reach.

## What the model sees

The policy gets the overhead camera, a camera on each wrist (all 320 x 176 RGB) and the joint and jaw positions.
In the overhead view one pixel is about 2 to 3 mm at the instrument: the wooden case (559 mm, about 200 pixels
wide), the white and black keys (a white key about 8 pixels wide), the three coloured dots (3 to 4 pixels), the
stop rail and the bellows board with its dark folds are all in view from the first frame. The wrist cameras see
the keys under the right hand and the board's edge under the left. Sound is not an input (the simulator has
none). A review-only camera (1280 x 704, never a policy input) looks across the keyboard at both hands and the
bellows from the robot's left.

## How the air works in the simulation

The board and the keys are real articulated bodies moved only by finger contact. The board's hinge carries a
velocity drive whose torque is capped at 2.19 N m: left alone, the board swings open to its 0.30 rad stop; pushed
shut at any speed it resists with that torque, 11.6 N at its top edge. Every control step `air.py` reads the
board's angle and the 39 key depths: each radian the board closes adds its swept volume (9.5 L/rad) to the
reservoir, capped at 1.2 L (the rest leaves by the relief valve); the reservoir's spring holds 160 to 230 Pa while
it holds air and the pressure falls to zero over its last 30 mL; a key at least 2.5 mm down opens its reed, which
sounds while the pressure is above 120 Pa and draws 0.4 L/s (orifice law); the valves leak 0.02 L/s. This rule
stands in for the air, a fluid response the rigid-body simulator does not have; the pushing and the key presses
are real contact work. The run writes `media/sound.wav`, a reed-tone synthesis of the sounding keys (louder with
pressure), and `media/calibration.wav`, so that the real-trial rule can be run on a simulated trial.

## Scoring

`grading.py` builds the score from one set of sounding keys per 50 ms frame.

- In simulation a key sounds while it is at least 2.5 mm down and the reservoir pressure is above 120 Pa.
- A note is held when it sounds for at least 4.0 s, silences of 0.1 s or less bridged.
- 0.25 Sa held; 0.5 then Re held (starting and ending after Sa); 0.75 then Ga held, in order.
- 0.85 in addition: each change of note leaves at most 0.5 s of silence and overlaps the previous note by at
  most 1.0 s (longer is a chord); no other key sounds for more than 0.2 s in all; the harmonium never moves more
  than 10 mm; and at the end both grippers are off the harmonium (no contact with keys, board or case) with
  every key up, for 1 s.
- 1.0: all that, and the sound never breaks: each next note starts before the previous one stops (legato, as
  players do it) or at most 0.1 s (two frames, the dropout a recording is allowed inside a note) after.
- 0 for the rest of the trial if the harmonium is pushed more than 30 mm or tipped more than 5 degrees.

In `RESULT.json` (`final_grade`) and the scorer trace: `score` is the final score; `played` is what the playing
earned (0, 0.25, 0.5, 0.75, 0.85 or 1.0) before the release rule (1.0 and 0.85 need both hands off with every key
up for 1 s, otherwise 0.75) and the latched zeros; `held_in_order` is the part of it that comes from holding and
order alone (0, 0.25, 0.5 or 0.75), before the joins, other keys and case motion are judged; `joined` is
"legato", "detached" or none, with the two joins in `joins_s` (negative: an overlap).

### Scoring a real trial

The real trial is judged from a microphone recording with a calibrated pitch rule (`audio.py`, command line
`tools/judge_audio.py`), so no sound-level meter and no absolute decibel threshold are needed:

1. Calibration, just before the trial, same instrument, microphone and room: a person pumps steadily and holds
   every key from C2 to D5 (all 39) for 1.5 s in turn, 0.5 s apart, then 3 s of the room with the robot powered
   and still (about 81 s in all), so that every key has its own reference level and the stray-key limit can be
   enforced anywhere on the keyboard.
   Also time once how long one held key sounds after pumping stops with the reservoir full; the hold time is
   4.0 s, or that time plus 1 s if longer, so that a held note always needs pumping during it.
2. Analysis: 4,096-sample Hann windows (0.256 s at 16 kHz) every 50 ms; a key's level is the power within 30
   cents of its fundamental. Its reference is its median level during its own calibration note; the noise floor
   is the 90th percentile of that band in the silent part.
3. A key sounds in a frame when its level is within 15 dB of its reference and at least 10 dB above the noise
   floor. A knock or a key pressed on empty bellows makes no reed tone and does not count. The long pitch
   windows blur note edges by about 0.1 s, so breaks are timed separately: a frame whose own 50 ms of sound has
   its 200 to 2,000 Hz band more than 20 dB below the median of the calibration notes is silent whatever the
   pitch windows say. A break between notes is then read to within 50 ms.
   Overtones and spill are not keys: the candidate keys are taken in order of how close each is to its own
   reference level, and a key counts only if it also stands 5 dB above what the keys already counted put into its
   band, as measured during their own calibration notes (C4's second harmonic falls on C5, D4's on D5, a low key's
   fourth on C4, and a low note spills onto its neighbour semitone). So a marked note never shows as a stray key,
   a stray low key never shows as Sa, Re or Ga, and any key really pressed, alone or with a marked note, is heard.
4. The frames go through the same `PhraseGrade` as the simulation. The harmonium's position comes from tape
   marks and the video; the release from the video.

Checks in `tests/test_audio.py`: the calibration covers all 39 keys; a stray C2, G#2, C3, F#3, B3, C5 or D5 is
heard in 18 or more of its 20 frames and never as a marked note; C5 or D5 played together with C4 or D4 is still
heard; the phrase alone shows no stray key; recordings with the earlier 11-key calibration (A3 to G4, written by
the simulated runs up to run 8) are still read. The rule also reads a legato phrase back from its synthesized
sound to within 0.15 s per note, reads breaks of 0.05, 0.15 and 0.3 s between notes to within 50 ms (so legato and detached are told
apart), tells C4 from C#4, still counts a note played at the reservoir's lowest pressure, and ignores a click.
On the dry-run demonstration the microphone rule and the simulator agree on 431 of 435 frames, and the rule also
scores it 1.0.

Tests in `tests/test_grading.py` cover the full phrase, each partial tier, a note 0.1 s short, the wrong order,
one-handed timelines, breaks of 0, 0.1, 0.15, 0.5 and 0.6 s, overlaps of 0.2, 0.8 and 0.9 s, bridged and unbridged dropouts, a stray
key, a moved, pushed or tipped case, a late release and a stuck key.

## The demonstration

`scripted.py` reads the joint readings and, as the review rules allow, simulator state (the board's angle, the
reservoir's air, key depths and sounding reeds). Both grippers are closed and used as paddles.

Left hand (pump), tipped 25 degrees forward along the line from its base: it comes down beside the board's left
end, behind it, and slides in along the top edge; it pumps by pulling toward the robot so that its lower finger's
outer face, 22 mm up from the tip, pushes the edge shut, servoed on the measured board angle. The command leads
the board by up to 70 mm while the compliant arm lags under the 12 N load. That lead is aimed along the finger
face's outward normal (toward the robot and 37 degrees down, at most 12 mm of it downward), because the face
leans 36 degrees from vertical and friction alone cannot hold the edge on it (run 1, below). A stroke closes the
board from 0.30 to 0.12 rad (33 mm at the top edge, 1.7 L) whenever the reservoir is less than half full, then
the hand backs off and the board reopens. If the measured hand rides 15 mm above where it would be with the edge on its
finger at the board's measured angle, passes 20 mm in front of the edge, or a stroke leaves the board open, the hand goes back behind the board the way it first came
in (up, out past the board's left end, back, down, along the edge) and meets the edge 8 mm further up the finger
(at most three times).

Right hand (keys), legato as players do it: tipped 55 degrees and turned 35 degrees about the vertical, so that its
lower pad tip's 15 mm end edge lies nearly along the keyboard. Twisting joint 6 alone (the hand's own axis) by
30 degrees either way tilts that edge so that one end is 6.3 mm lower than the other, and moves the low end 8.9 mm
along the keyboard. A note is held at a straddle: the hand placed so that the edge's lower end is 4.5 mm inside the
held key, 3.3 mm down (servoed on the measured depth; shallow so that the part of the edge just over the next key,
rising 0.41 mm per mm, stays under that key's 2.5 mm), and its raised end over the next key. The change of note
is a twist of joint 6 from -30 to +30 degrees in 0.6 s, nothing else moving: the next key goes down as the held one
comes up. Two legs of 1.2 s then carry the edge across the new key to the next straddle (to its middle with the
edge level, then on with the twist back), each step taken only when the measured hand has caught up with the
last (within 2.5 mm and 3 degrees of its standing error). The hand's horizontal place is servoed on the measured
hand while it is meant to be still (at most 12 mm). After Ga both hands back out and return to ready.

Kinematic dry run (`python tools/dryrun_demo.py --clearance <folder with clearance.py>`: arms as first-order lags,
the board pushed exactly as far as the finger goes and reopening otherwise, key depths from the pad tip, the real
air rule and scorer): 1.0 at step 439 (22 s), Sa, Re and Ga 4.6, 4.6 and 4.4 s, each change overlapping 0.15 s,
the reservoir never below 0.55 L while a note plays. The arm model was calibrated on Isaac: giving the shoulder
joints a 0.07 s lag and the soft wrist joints 0.15 s matches the key hand's measured path in runs 3 and 4 to 5 to
7 mm RMS, and run on run 4's demonstration it reproduced run 4's failure (0.75, F4 sounding 0.45 s, a 1.7 s
overlap). With that model the current demonstration scores 1.0 (step 436), also with the wrist lagging more
(0.05 and 0.25 s), with the key hand 3 mm off along the keyboard either way and uncorrected (4 mm fails), with
uniformly slow arms (0.25 s), with the pushing hand giving way 30 mm under load, and with the key hand riding 7,
10 or 13 mm above its plan (as it did in runs 4 and 5; the depth servo makes it up). Inverse kinematics error 0.07
mm, joint margins at least 19 degrees, the two arms at least 263 mm apart, arm links (not the fingers) at least
30 mm from the case and its cheeks, the right fingertips at least 8 mm from every unmarked key.
`tests/test_demo.py` also checks that the recovery path from where run 1's hand came to rest is reachable and
that a good stroke (run 2) is not taken for a slip.

## Results in Isaac

Both final trials ran in run 8 on the final package files (the scene, asset, layout, air rule and scorer hashes in
both `RESULT.json` files match this folder), on one L40S GPU, Isaac Lab-Arena, seed 0, 20 Hz, cameras 320 x 176.

| | Demonstration (scripted) | Dream-1 |
| --- | --- | --- |
| Score | **1.0** (success at step 464 of 2,400, 23.2 s) | **0.0** (all 2,400 steps) |
| Sa, Re, Ga held | 4.7, 4.7, 4.4 s | 0, 0, 0 s |
| Changes of note | legato: each next note began 0.35 s before the last one stopped | none |
| Other keys sounding | 0 s | 0 s |
| Air pumped in | 14.2 L in 8 strokes (7.1 L let out by the relief valve) | none |
| Harmonium moved | 0.81 mm | 0 mm |
| Both hands off at the end | yes | yes |
| Wall time | 134 s | 555 s (58 model requests) |

The demonstration in detail, from `RESULT.json`:
- Pump: each stroke closed the bellows board from 0.30 to 0.11 rad against up to 15.4 N, with the command leading
  the board by about 52 mm (the compliant arm's lag under that load); no recoveries were needed. The reservoir
  never fell below 0.42 L while a note played.
- Keys: each held key stayed between 2.6 and 4.7 mm down (it speaks from 2.5 mm); the depth servo ended at -7.5
  mm, making up the hand riding high under the key load; at most 2.3 N on the keys. The twist rock changed notes
  with the next key down before the last came up, and nothing else was pressed.
- The real-trial sound rule (`tools/judge_audio.py`) run on the trial's `media/sound.wav` and
  `media/calibration.wav` also scores it 1.0 (Sa, Re, Ga 4.75, 4.8, 4.45 s; joins -0.45 s).
- Dream-1 moved both grippers, mostly closed, over the instrument at 0.20 to 0.34 m above the table. Its left hand
  pressed on the top of the case at times (up to 24 N) and its right hand pushed two unmarked black keys part way
  down (F#3 4.2 mm, A#3 3.0 mm); it never touched the bellows board, so the reservoir stayed empty and no reed
  sounded.

### History (runs 1 to 7, earlier versions)

Run 7 (2026-11-18): the same demonstration and Dream-1 trial on the package as it was just before two result
fields were renamed for clarity (`tier`, now `held_in_order`; no rule or score changed). Demonstration 1.0 at step
464 (Sa, Re, Ga 4.7, 4.7, 4.4 s, legato joins -0.35 s, no other key, case moved 0.81 mm); Dream-1 0.0 over 2,400
steps (60 model requests; it hovered over the front of the instrument, touched the case top lightly and pressed
no key or the bellows board). Run 8 repeated both on the final files.

Run 1 (2026-11-18, one L40S, 292 s, 1,200 steps): 0.0. The setup, the reset and both
arms' placement worked (board open at 0.30 rad, trims under 2 mm, the right hand 3 mm from its hover point over
Sa), but the left finger never moved the board: at its first push it met the edge with 3 N, rode up 15 mm in a
quarter of a second and slid over the top edge onto the case (peak 7 N on the case, the board still at 0.30 rad),
and every later stroke did the same, so the reservoir stayed empty and Sa was never pressed. Cause: the closed
fingers' outer face widens 11.5 degrees up the finger, so in the 25-degree tipped hand the face the edge met
leans 36 degrees from vertical; a horizontal pull then needs friction above tan 36 = 0.73 to hold the edge, and
it slipped. Run 2's version aimed the push along that face's normal (needing no friction), met the edge
22 mm up the finger, gave the board a rubber-on-wood friction of 0.9, recovered and retried when the hand slipped,
logged the left hand's position relative to the edge on every step, and played legato (scoring change).

Run 2 (2026-11-18, 284 s, 1,200 steps): 0.0, with Sa sounding 2.85 s. The new push worked: the first stroke
closed the board from 0.30 to 0.19 rad against 12 to 14 N with the hand steady on the edge (8 mm behind it, under
1 mm up) and filled the reservoir to 1.2 L; the right hand then held Sa 4.5 to 4.7 mm down. But the slip check
compared the hand with its command, which leads the board and sinks 12 mm below it, so it flagged that good stroke
and every later one within a quarter of a second, before the finger even reached the edge; after three recoveries
the strokes kept being cut short, the reservoir ran dry 2.85 s into Sa, and the rest of the phrase was played on
empty bellows. During the first change of note the right hand also swung about 20 mm along the keyboard and pressed
E4, and at the end it brushed the case while its fingers opened on the way to ready. Changes: the slip check now
compares the hand with where it would be with the edge on its finger at the board's measured angle (a test checks
a good stroke is not flagged) and stops after three recoveries; the changes of note are slower (2.6 s instead of
1.8 s) with a servo on the key hand's measured horizontal position; the hands swing to ready with closed fingers;
the run limit is 2,400 steps (120 s), so that recoveries cannot use up the trial.

Run 3 (2026-11-18, 2,400 steps): 0.75. The phrase was played: Sa, Re and Ga 4.55, 4.55 and 4.65 s, legato
(joins 0.05 and -0.15 s), no pump recoveries, case moved 0.81 mm, both hands let go. Full score was missed only
because F4 sounded for 0.5 s (ticks 332 to 342, limit 0.2 s), so the trial ran to its limit (it ends early only on
1.0). F4 went down during the rock from Re to Ga: the key hand swung 15 mm past its planned place along the
keyboard (y -122 mm against -107 planned) and the edge's low end, centred only 3.6 mm from E4's far side, reached
F4. The swing came from the horizontal servo winding up while the hand lagged its moving command. Changes: the
servo integrates only while the hand is meant to be still; a held key is pressed 2.9 mm nearer its middle; the
roll back after a change first levels the edge without moving. The dry run now also checks the key hand 3 to 5 mm
off along the keyboard.

Run 4 (2026-11-18, 2,400 steps): 0.75. All three notes were held (4.55, 4.55, 4.45 s) but joined by 2.25 s of
overlap and F4 sounded for 0.45 s. The trace shows two things. During each rock (then a roll of the whole hand)
the edge swung 12 to 15 mm along the keyboard beyond its command: toward F4 at the second change, and, during the
roll back after the first change, onto C4, where its low end sank into C4 (pressed to its 7 mm stop) and was held
there by D4's side for 1.7 s (C4 and D4 sounding together). A first-order arm model cannot swing like that; one
in which the wrist joints lag the shoulder joints (0.15 s against 0.07 s) does, and fits both runs. Changes: the
rock is now a twist of joint 6 alone, so no shoulder motion has to keep pace with the wrist; the moves that remain
step on only when the measured hand has caught up; the held key is kept shallower (3.3 mm) and the depth servo may
press at most 6 mm deeper than planned.

Run 5 (2026-11-18, 2,400 steps): 0.0. The joint-6 rock did what it was meant to (no swing along the keyboard,
no unmarked key, Ga held 4.4 s), but Sa and Re never sounded: C4 and D4 stayed 1.9 to 2.0 mm down, under the
2.5 mm at which a reed speaks. The pressing hand rode about 7 mm above its plan (in run 4 about 6 mm), and the
depth servo, which had been capped at 6 mm in run 5, could not make that up. The dry run with the same 7 mm height
error and the 6 mm cap fails the same way (Sa and Re 0.2 s or less). The cap is now 16 mm, and the servo stops pressing deeper whenever any other key is more than 1.8 mm down (so
that a hand sitting too far toward the next key cannot press on into a chord); the dry run with the hand riding
7, 10 or 13 mm high scores 1.0, also combined with a 3 mm error along the keyboard.

Run 6 (2026-11-18): the demonstration scored 1.0 (success at step 464: Sa, Re and Ga 4.7, 4.7 and 4.4 s, legato
joins -0.35 and -0.35 s, no other key, both hands let go, case moved 0.81 mm). The Dream-1 trial that followed in
the same job stopped at import ("No module named 'yam_arena'") before any simulation started. `run.py` now imports
the adapter only inside `main()`, after making sure yam_arena can be imported (falling back to the station source
tree named by the job environment) and writing a RESULT.json with the Python path if it cannot. Nothing else
changed; run 7 repeated the demonstration and ran Dream-1.

## Running

Install the package next to the Inspect Robots YAM runtime and run one trial from the task folder:

    pip install -e .
    python -m rdb_harmonium.run scripted --out runs/scripted     # the demonstration
    python -m rdb_harmonium.run dream --out runs/dream           # Dream-1

Each trial is one fixed scene (seed 0, no jitter), at most 2,400 actions at 20 Hz. `RESULT.json` holds the score
and per-phase diagnostics; `media/` the overhead and review videos, the setup image, the last frames, the scorer's
trace, `sound.wav` and `calibration.wav`. Judge a recording with the real-trial rule:
`python tools/judge_audio.py media/sound.wav media/calibration.wav`. Wall time per trial on one L40S GPU, start-up
included: 134 s for the demonstration (464 steps) and 555 s for the full 2,400-step Dream-1 trial (run 8). Unit tests: `python -m pytest tests`.

## Licences

All code and the generated scene are under the MIT licence in `LICENSE`. No third-party assets are used: the
harmonium is built from boxes and generated meshes. The YAM arm model and the station come with the Inspect Robots
YAM runtime and are not bundled here.


## Packaging and recorded-source identity

The physical scene, asset, controller, air rule and simulation scorer match run 8. After that run, the real-trial audio helper was extended from 11 calibration notes to all 39 keys and harmonic rejection was added. It still reads the older recorded calibration. This is a post-processing update, not a new native run. The original audio helper is retained in evidence/audio-at-recording.py for comparison.
