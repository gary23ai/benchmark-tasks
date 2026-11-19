"""Every dimension of the harmonium and of its air model, in metres, kilograms, pascals and seconds, each marked by
where it comes from.

Instrument frame: origin on the table under the middle of the case's front edge (the keyboard side, toward the
robot); x back, away from the player (station +x), y to the player's left (station +y), z up. The harmonium faces
the robot squarely, so this frame is the station frame shifted by FRONT_X. NumPy only.

Sources (README has the links):
  [L] retail listings of standard 39-key, 3 1/4-octave, 7-stop, double-bellows Indian harmoniums: 55.88 x 30.48 x
      25.4 cm and 8 kg (one listing); length 53 to 56 cm, height 23 to 25 cm, 7 to 10 kg (others)
  [B] the Banjira Standard 7-knob harmonium page: 39 keys, front reed bank C2 to D5, back bank C3 to D6, 3 drones
  [W] reed organ construction notes: a spring on the reservoir sets a steady wind pressure; parlour reed organs run
      at 40 to 130 mm water gauge (one page; a suction instrument, so only an order of magnitude)
  [E] estimate (no published figure found; README says how to measure each on a real instrument)
"""

import numpy as np

# --- the case ------------------------------------------------------------------------------------------------
CASE_W = .559                     # [L] 22 in, side to side (y)
CASE_D = .305                     # [L] 12 in, front to back (x), the closed bellows board included
CASE_H = .205                     # [L]/[E] 254 mm listed with its lid; the lid (about 50 mm) is off for playing
MASS = 8.0                        # [L] 8 kg
FEET_H = .010                     # [E] four rubber feet
BACK_X = .293                     # back wall of the case; the bellows board (12 mm) closes against it
KEYBED_Z = .164                   # [E] the case under the keys
TOP_X = .135                      # [E] the top board behind the stop rail runs from here to the back
STOP_RAIL = (.108, .135, .205)    # [E] x from, x to, top: the raised rail behind the keys that holds the stop knobs
STOP_KNOB_D, STOP_KNOB_L = .018, .016          # [E] seven round stop knobs on the rail's front face
STOP_KNOB_Z = .192
STOPS = 7                         # [L] "7 stops"; the second from the left (the main stop) is pulled out 20 mm
MAIN_STOP_PULL = .020

# --- the keyboard (39 keys, C2 to D5: 23 white, 16 black) -------------------------------------------------------
LOWEST_MIDI = 36                  # [B] C2
KEYS = 39                         # [L][B]
WHITE_PITCH = .0220               # [E] 23 white keys over 506 mm of a 559 mm case
WHITE_W = .0210                   # 1 mm between white keys
WHITE_X = (.006, .106)            # white keys: front and back ends (100 mm showing)
WHITE_TOP = .180                  # [E] top of a white key at rest
WHITE_T = .014                    # collider thickness under the top
BLACK_W = .0110
BLACK_X = (.050, .106)
BLACK_TOP = .190
BLACK_T = .016
KEYBOARD_LEFT_Y = 23 * WHITE_PITCH / 2         # the C2 end, at the player's (and robot's) left
KEY_TRAVEL = .007                 # [E] a key goes 7 mm down to its stop
KEY_SPEAKS = .0025                # [E] its pallet opens, and the reed can speak, once it is 2.5 mm down
KEY_RATE = 110.                   # [E] N/m key spring
PLAY_FROM_FRONT = .022            # the demonstration presses a white key 22 mm behind its front end
DOT_FROM_FRONT = .014             # centre of a dot sticker, behind the key's front end
KEY_PRELOAD = .006                # the spring's rest is 6 mm above the top stop: 0.66 N to start, 1.4 N at the bottom
KEY_MASS_WHITE, KEY_MASS_BLACK = .015, .010    # [E]
WHITE_STEPS = (0, 2, 4, 5, 7, 9, 11)           # semitones of the white keys in an octave from C
NAMES = ("C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B")

# --- the phrase -------------------------------------------------------------------------------------------------
PHRASE = ("C4", "D4", "E4")       # Sa, Re, Ga with Sa on C4 (yellow, green and blue dot stickers)
DOT_COLOURS = {"C4": (.95, .82, .10), "D4": (.15, .65, .25), "E4": (.15, .35, .90)}

# --- the bellows ------------------------------------------------------------------------------------------------
FLAP_W = .530                     # [E] the bellows board at the back, side to side
FLAP_T = .012                     # [E] 12 mm wooden board (the draft's figure)
HINGE = (BACK_X, .016)            # its hinge runs along y at the bottom of the back wall (x, z)
FLAP_H = CASE_H - HINGE[1]        # 189 mm from the hinge to the board's top edge
FLAP_OPEN = .30                   # [E] rad: the top edge opens 56 mm; a spring holds it open when left alone
FLAP_MASS = .60                   # [E]
FEEDER_M3_PER_RAD = FLAP_W * FLAP_H ** 2 / 2   # air pushed into the reservoir per radian the board closes: 9.5 L/rad
# The board resists being pushed shut with the reservoir's working pressure over its area, and its spring reopens it.
WORK_PA = 200.                    # [W]/[E] 20 mm water gauge, a light wind for a small portable instrument
SPRING_NM = .30                   # [E] the reopening spring, N m about the hinge
FLAP_TORQUE_NM = WORK_PA * FEEDER_M3_PER_RAD + SPRING_NM          # 2.19 N m about the hinge
TOP_EDGE_PUSH_N = FLAP_TORQUE_NM / FLAP_H                         # 11.6 N pushed at the top edge
FLAP_OPEN_SPEED = .8              # rad/s: the spring reopens the board no faster than this
FLAP_DAMPING = 20.                # N m s/rad of the drive that carries the torque (saturates at FLAP_TORQUE_NM)

# --- the reservoir (inside the case) ---------------------------------------------------------------------------
RESERVOIR_M3 = 1.2e-3             # [E] air the spring-loaded reservoir holds between empty and its relief valve
EMPTY_PA, FULL_PA = 160., 230.    # [E] its pressure from just above empty to full (spring), 16 to 23 mm water
REED_FLOW_M3S = 4.0e-4            # [E] one open reed at 200 Pa: 0.4 L/s (orifice law, flow ~ sqrt(pressure))
REED_FLOW_PA = 200.
LEAK_M3S = 2.0e-5                 # [E] 0.02 L/s lost through the valves while the reservoir holds air
SAG_M3 = 3.0e-5                   # [E] the last 30 mL: the reservoir rests on its stops and its pressure falls to zero
SPEAK_PA = 120.                   # [E] a reed speaks above this pressure (below the last 23 mL it does not)


def white_index(midi):
    """Position of a white key among the 23 (0 = C2), or None for a black key."""
    octave, step = divmod(midi - LOWEST_MIDI, 12)
    return 7 * octave + WHITE_STEPS.index(step) if step in WHITE_STEPS else None


def key_name(midi):
    return f"{NAMES[midi % 12]}{midi // 12 - 1}"


KEY_MIDI = tuple(range(LOWEST_MIDI, LOWEST_MIDI + KEYS))
KEY_NAMES = tuple(key_name(m) for m in KEY_MIDI)


def prim_name(name):
    """A key name usable as a USD prim name ("C#4" -> "Cs4")."""
    return name.replace("#", "s")


def key_box(midi):
    """(centre x, centre y, top z, size x, size y, thickness, mass, is white) of a key at rest, instrument frame."""
    w = white_index(midi)
    if w is not None:
        y = KEYBOARD_LEFT_Y - WHITE_PITCH / 2 - w * WHITE_PITCH
        return ((WHITE_X[0] + WHITE_X[1]) / 2, y, WHITE_TOP, WHITE_X[1] - WHITE_X[0], WHITE_W, WHITE_T,
                KEY_MASS_WHITE, True)
    below = white_index(midi - 1)                 # a black key sits over the line between two white keys
    y = KEYBOARD_LEFT_Y - (below + 1) * WHITE_PITCH
    return ((BLACK_X[0] + BLACK_X[1]) / 2, y, BLACK_TOP, BLACK_X[1] - BLACK_X[0], BLACK_W, BLACK_T,
            KEY_MASS_BLACK, False)


def key_by_name(name):
    return KEY_MIDI[KEY_NAMES.index(name)]


def frequency(name):
    return 440. * 2 ** ((key_by_name(name) - 69) / 12)


def flap_point(theta, local_x, local_z):
    """A point of the bellows board (``local_x`` out from its inner face, ``local_z`` up from the hinge) in the
    instrument frame (x, z), with the board opened ``theta`` rad."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array((HINGE[0] + local_x * c + local_z * s, HINGE[1] - local_x * s + local_z * c))


def flap_corner(theta):
    """The board's outer top edge (x, z): where the pumping hand pushes."""
    return flap_point(theta, FLAP_T, FLAP_H)
