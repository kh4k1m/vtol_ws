"""Color-classification helpers shared between plot types.

The visualization legend is consistent across every plot:

* GPS reference (truth)        -> blue
* prediction within tolerance   -> green
* prediction beyond tolerance   -> red
* marker invisible (no detect)  -> red on marker_visibility plot

This module owns the actual binning logic so plots stay declarative.
"""

from __future__ import annotations

from typing import List, Sequence, Tuple

import numpy as np


COLOR_REFERENCE = '#1f77b4'   # blue
COLOR_OK = '#2ca02c'          # green
COLOR_BAD = '#d62728'         # red


def nearest_match_indices(
    src_times_ns: Sequence[int],
    ref_times_ns: Sequence[int],
    max_dt_ns: int,
) -> List[int]:
    """For every src time, return index into ref of the nearest sample, or -1.

    Uses binary search; assumes both inputs are sorted.
    """
    if not src_times_ns or not ref_times_ns:
        return [-1] * len(src_times_ns)

    ref_arr = np.asarray(ref_times_ns, dtype=np.int64)
    out: List[int] = []
    for t in src_times_ns:
        idx = int(np.searchsorted(ref_arr, t))
        candidates = []
        if idx < len(ref_arr):
            candidates.append((abs(int(ref_arr[idx]) - int(t)), idx))
        if idx > 0:
            candidates.append((abs(int(ref_arr[idx - 1]) - int(t)), idx - 1))
        if not candidates:
            out.append(-1)
            continue
        dt, best = min(candidates)
        out.append(best if dt <= max_dt_ns else -1)
    return out


def colorize_by_tolerance(
    src_xyz: Sequence[Tuple[float, float, float]],
    ref_xyz: Sequence[Tuple[float, float, float]],
    nearest_idx: Sequence[int],
    tolerance_m: float,
    horizontal_only: bool = True,
) -> Tuple[List[str], List[float]]:
    """Return (colors[], errors[]) for each src point.

    ``nearest_idx[i] == -1`` means no reference within tolerance - we
    paint those gray; consumers can choose to drop them.
    """
    colors: List[str] = []
    errors: List[float] = []
    for i, ref_idx in enumerate(nearest_idx):
        if ref_idx < 0 or i >= len(src_xyz):
            colors.append('lightgray')
            errors.append(float('nan'))
            continue
        sx, sy, sz = src_xyz[i]
        rx, ry, rz = ref_xyz[ref_idx]
        if horizontal_only:
            err = float(np.hypot(sx - rx, sy - ry))
        else:
            err = float(np.linalg.norm(np.array([sx - rx, sy - ry, sz - rz])))
        errors.append(err)
        colors.append(COLOR_OK if err <= tolerance_m else COLOR_BAD)
    return colors, errors


def visibility_colors(
    src_times_ns: Sequence[int],
    detection_times_ns: Sequence[int],
    detection_counts: Sequence[int],
    visibility_gap_ns: int,
) -> List[str]:
    """For every src timestamp, return green if a tag was seen within
    visibility_gap_ns, otherwise red."""
    if not detection_times_ns:
        return [COLOR_BAD] * len(src_times_ns)
    det_arr = np.asarray(detection_times_ns, dtype=np.int64)
    counts = np.asarray(detection_counts, dtype=np.int64)
    out: List[str] = []
    for t in src_times_ns:
        idx = int(np.searchsorted(det_arr, t))
        seen = False
        for j in (idx - 1, idx):
            if 0 <= j < len(det_arr):
                if abs(int(det_arr[j]) - int(t)) <= visibility_gap_ns and counts[j] > 0:
                    seen = True
                    break
        out.append(COLOR_OK if seen else COLOR_BAD)
    return out
