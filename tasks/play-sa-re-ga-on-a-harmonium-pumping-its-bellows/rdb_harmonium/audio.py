"""The real trial's sound rule, and a reed-tone synthesis of a simulated trial to check it against. NumPy only.

Real trial (README, "Scoring a real trial"): a microphone beside the harmonium records the calibration (every one
of the 39 keys, C2 to D5, held for 1.5 s in turn with steady pumping, 0.5 s apart, then 3 s of the room with the
robot powered and still: about 81 s) and then the trial. ``judge`` turns a recording into one set of sounding keys
per 50 ms frame:

- spectrum: 4,096-sample Hann windows (0.256 s at 16 kHz) every 50 ms;
- a key's level in a frame: the power within 30 cents of its fundamental (at least the nearest spectrum bin), in
  dB;
- its reference level: the median of that level over its own calibration note; the noise floor: the 90th
  percentile over the silent calibration segment;
- a key sounds in a frame when its level is within ``WITHIN_DB`` (15 dB) of its reference and at least
  ``ABOVE_NOISE_DB`` (10 dB) above the noise floor;
- overtones and spill are not keys: candidates are taken in order of how close they are to their own reference
  level, and each counts only if it also stands ``CROSS_MARGIN_DB`` (5 dB) above what the keys already counted in
  that frame put into its band (their harmonics and spectral spill: C2's fourth harmonic falls on C4, C4's second
  on C5, a low note spills onto its neighbour semitone), as measured during those keys' own calibration notes; so
  an overtone is never taken for a key (a stray C2 does not make C4 sound), while a key really played with it is;
- silences are timed sharply: the long pitch window blurs note edges by about 0.1 s, so a frame whose own 50 ms
  of sound has its reed band (200 to 2,000 Hz) more than ``SILENT_DB`` (20 dB) below the calibration notes' median
  counts as silent whatever the pitch windows say. A gap between notes is then read to within 50 ms.

The frames then go through the same ``grading.PhraseGrade`` as the simulation. Because every key is compared with
its own sound on the same instrument, microphone and room, no calibrated sound-level meter is needed, and a knock
or a key pressed on empty bellows (no reed tone) does not count.

In simulation the sounding keys come from the simulator; ``synthesize`` renders them as reed tones (a sawtooth-like
series of harmonics at each key's pitch, louder with reservoir pressure) into ``media/sound.wav`` so that the run
can be listened to and the real rule can be run on it (``tools/judge_audio.py``).
"""

import wave

import numpy as np

from . import layout as L

RATE = 16000
WINDOW = 4096
HOP = RATE // 20                     # 50 ms, one control step
CENTS = 30.
WITHIN_DB = 15.
ABOVE_NOISE_DB = 10.
CALIBRATION_KEYS = L.KEY_NAMES                     # all 39 keys, C2 to D5
CAL_NOTE_S, CAL_GAP_S, CAL_SILENCE_S = 1.5, .5, 3.
# Recordings written by the first eight simulated runs calibrated only A3 to G4, 2 s each; still readable.
LEGACY_KEYS, LEGACY_NOTE_S = ("A3", "A#3", "B3", "C4", "C#4", "D4", "D#4", "E4", "F4", "F#4", "G4"), 2.
CROSS_MARGIN_DB = 5.
SILENT_DB = 20.
REED_BAND = (200., 2000.)


def tone(freq, seconds, amplitude, rate=RATE, phase=0.):
    t = np.arange(int(round(seconds * rate))) / rate
    out = np.zeros_like(t)
    for h in range(1, 9):                                  # free-reed spectrum: strong upper harmonics
        if h * freq < rate / 2:
            out += np.sin(2 * np.pi * h * freq * t + h * phase) / h ** .8
    return amplitude * out / 3.


def synthesize(frames, rate=RATE, noise=1e-4, seed=0):
    """``frames``: one (sounding key names, reservoir pressure in Pa) per 50 ms. Returns float samples."""
    hop = rate // 20
    out = np.zeros(len(frames) * hop)
    phases = {}
    t = np.arange(hop) / rate
    for i, (names, p) in enumerate(frames):
        amp = .25 * min(1., p / L.FULL_PA)
        for name in names:
            f = L.frequency(name)
            ph = phases.get(name, 0.)
            seg = sum(np.sin(2 * np.pi * h * f * t + h * ph) / h ** .8 for h in range(1, 9) if h * f < rate / 2)
            out[i * hop:(i + 1) * hop] += amp * seg / 3.
            phases[name] = (ph + 2 * np.pi * f * hop / rate) % (2 * np.pi)
        if not names:
            for name in list(phases):
                phases.pop(name)
    return out + noise * np.random.default_rng(seed).standard_normal(len(out))


def calibration_recording(rate=RATE, noise=1e-4, seed=1):
    """A calibration recording as the README describes it, synthesized at the working pressure."""
    parts = []
    for name in CALIBRATION_KEYS:
        parts.append(tone(L.frequency(name), CAL_NOTE_S, .25 * L.WORK_PA / L.FULL_PA, rate))
        parts.append(np.zeros(int(CAL_GAP_S * rate)))
    parts.append(np.zeros(int(CAL_SILENCE_S * rate)))
    out = np.concatenate(parts)
    return out + noise * np.random.default_rng(seed).standard_normal(len(out))


def write_wav(path, samples, rate=RATE):
    data = np.clip(np.asarray(samples) * 32767, -32768, 32767).astype("<i2")
    with wave.open(str(path), "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(rate)
        w.writeframes(data.tobytes())


def read_wav(path):
    with wave.open(str(path), "rb") as w:
        rate, width, channels = w.getframerate(), w.getsampwidth(), w.getnchannels()
        raw = w.readframes(w.getnframes())
    data = np.frombuffer(raw, dtype={2: "<i2", 4: "<i4"}[width]).astype(float) / (2 ** (8 * width - 1))
    return data.reshape(-1, channels).mean(axis=1), rate


def levels(samples, rate, keys=CALIBRATION_KEYS):
    """dB level of each key's fundamental band in every 50 ms frame (frames centred every 50 ms from t = 0)."""
    hop = rate // 20
    window = np.hanning(WINDOW)
    padded = np.concatenate((np.zeros(WINDOW // 2), np.asarray(samples, dtype=float), np.zeros(WINDOW // 2)))
    count = len(samples) // hop
    freqs = np.fft.rfftfreq(WINDOW, 1 / rate)
    bands = {}
    half_bin = rate / WINDOW / 2
    for name in keys:
        f = L.frequency(name)
        width = max(f * (2 ** (CENTS / 1200) - 1), half_bin)
        bands[name] = (freqs >= f - width) & (freqs <= f + width)
    out = {name: np.zeros(count) for name in keys}
    for i in range(count):
        centre = i * hop + hop // 2 + WINDOW // 2
        chunk = padded[centre - WINDOW // 2:centre + WINDOW // 2] * window
        power = np.abs(np.fft.rfft(chunk)) ** 2
        for name, band in bands.items():
            out[name][i] = 10 * np.log10(power[band].sum() + 1e-20)
    return out


def broadband(samples, rate):
    """dB level of the reed band in each 50 ms frame, from that frame's own samples."""
    hop = rate // 20
    count = len(samples) // hop
    window = np.hanning(hop)
    freqs = np.fft.rfftfreq(hop, 1 / rate)
    band = (freqs >= REED_BAND[0]) & (freqs <= REED_BAND[1])
    out = np.zeros(count)
    for i in range(count):
        power = np.abs(np.fft.rfft(np.asarray(samples[i * hop:(i + 1) * hop], dtype=float) * window)) ** 2
        out[i] = 10 * np.log10(power[band].sum() + 1e-20)
    return out


def calibration_layout(samples, rate):
    """(keys, seconds per note) of a calibration recording, told apart by its length."""
    seconds = len(samples) / rate - CAL_SILENCE_S
    legacy = len(LEGACY_KEYS) * (LEGACY_NOTE_S + CAL_GAP_S)
    return (LEGACY_KEYS, LEGACY_NOTE_S) if abs(seconds - legacy) < 1. else (CALIBRATION_KEYS, CAL_NOTE_S)


def calibrate(samples, rate):
    """From a calibration recording: ({key: reference level dB}, {key: noise floor dB, "reed band": median reed-band
    level of the notes}, {key: {other key: its band level minus the key's own while that key alone sounds, dB}})."""
    keys, note_s = calibration_layout(samples, rate)
    lv = levels(samples, rate, keys)
    bb = broadband(samples, rate)
    ref, noise, cross = {}, {}, {}
    per = (note_s + CAL_GAP_S) * 20

    def held(i):                                            # each note without its first and last 0.4 s
        return slice(int(i * per + .4 * 20), int(i * per + (note_s - .4) * 20))

    for i, name in enumerate(keys):
        ref[name] = float(np.median(lv[name][held(i)]))
    for i, name in enumerate(keys):
        cross[name] = {k: float(np.median(lv[k][held(i)])) - ref[name] for k in keys if k != name}
    noise["reed band"] = float(np.median(np.concatenate([bb[held(i)] for i in range(len(keys))])))
    silent0 = int(len(keys) * per + .5 * 20)
    for name in keys:
        noise[name] = float(np.percentile(lv[name][silent0:], 90))
    return ref, noise, cross


def judge(samples, rate, ref, noise, cross=None):
    """One tuple of sounding key names per 50 ms frame."""
    lv = levels(samples, rate, tuple(ref))
    bb = broadband(samples, rate)
    count = len(next(iter(lv.values())))
    cross = cross or {}
    frames = []
    for i in range(count):
        if bb[i] < noise["reed band"] - SILENT_DB:
            frames.append(())
            continue
        on = [n for n in ref if lv[n][i] >= ref[n] - WITHIN_DB and lv[n][i] >= noise[n] + ABOVE_NOISE_DB]
        keep = []
        for n in sorted(on, key=lambda k: ref[k] - lv[k][i]):     # nearest its own reference first
            spill = max((lv[a][i] + cross[a][n] for a in keep if a in cross and n in cross[a]), default=-np.inf)
            if lv[n][i] >= spill + CROSS_MARGIN_DB:
                keep.append(n)
        frames.append(tuple(sorted(keep)))
    return frames
