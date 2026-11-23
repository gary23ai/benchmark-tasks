"""Judge equal portions from a top-down photograph. All lengths are in metres."""

from itertools import combinations

import numpy as np

# Every unordered split into six and six has one side containing the first carrot.
GROUP_A = np.array([(0, *indices) for indices in combinations(range(1, 12), 5)])
GROUP_B = np.array([sorted(set(range(12)) - set(group)) for group in GROUP_A])
ORANGE = (GROUP_A < 6).sum(axis=1)


def grade(
    positions,
    rotations,
    tool_clear,
    pan_stable,
    spilled,
    bare_hand,
    tool_used,
    one_layer,
):
    xy = positions[:, :2]
    distances = np.linalg.norm(xy[:, None] - xy[None, :], axis=2)
    neighbors = distances[:6, 6:]
    close_counts = [int((neighbors.min(axis=axis) <= 0.020).sum()) for axis in (1, 0)]

    # Try every assignment. Measure a straight clear strip perpendicular to the line
    # joining the two pile centres. Include the actual projected extent of each cube.
    a, b = xy[GROUP_A], xy[GROUP_B]
    direction = b.mean(axis=1) - a.mean(axis=1)
    direction /= np.maximum(np.linalg.norm(direction, axis=1, keepdims=True), 1e-12)
    projections = direction @ xy.T
    half_width = 0.005 * np.abs(
        np.einsum("si,nij->snj", direction, rotations[:, :2, :])
    ).sum(axis=2)
    near_b = np.take_along_axis(projections - half_width, GROUP_B, axis=1).min(axis=1)
    far_a = np.take_along_axis(projections + half_width, GROUP_A, axis=1).max(axis=1)
    gaps = near_b - far_a
    spans = [
        distances[group[:, :, None], group[:, None, :]].max(axis=(1, 2))
        for group in (GROUP_A, GROUP_B)
    ]
    compact = np.maximum(*spans) <= 0.040
    balanced = ORANGE == 3
    both_colors = (ORANGE >= 2) & (ORANGE <= 4)
    full = compact & balanced & (gaps >= 0.048)
    partial = compact & both_colors & (gaps >= 0.018)

    if spilled or bare_hand or not pan_stable or not tool_used:
        score = 0.0
    elif full.any() and tool_clear and one_layer:
        score = 1.0
    elif partial.any():
        score = 0.5
    elif min(close_counts) >= 4:
        score = 0.25
    else:
        score = 0.0
    eligible = np.flatnonzero(compact & balanced)
    best = eligible[np.argmax(gaps[eligible])] if eligible.size else None
    return dict(
        score=score,
        balanced_gap_mm=None if best is None else round(float(gaps[best]) * 1000, 2),
        portion_spans_mm=None
        if best is None
        else [round(float(s[best]) * 1000, 2) for s in spans],
        carrots_in_each=None if best is None else [3, 3],
        potatoes_in_each=None if best is None else [3, 3],
        unlike_neighbors=close_counts,
        spilled=spilled,
        hand_touched_food=bare_hand,
        tool_clear=bool(tool_clear),
        pan_stable=bool(pan_stable),
        single_layer=bool(one_layer),
    )
