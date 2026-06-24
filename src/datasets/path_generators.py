"""Parametric path geometry generators for the variant study.

The variant study defined in the Deliverable 1 proposal asks the
methods to be evaluated under three controlled axes of variation:
waypoint density, geometric character (sharp vs gradual), and time
spacing (uniform vs chord-length). To keep the comparison clean we
build *every* mobile-robot variant from a small set of analytic path
generators that take a desired number of waypoints as input.

Each generator returns an ``(N, 2)`` array of ``(x, y)`` coordinates
in metres. Time stamps are attached separately by the caller using the
spacing helpers in this module.
"""

from __future__ import annotations

from typing import Literal

import numpy as np


SpacingMode = Literal["uniform", "chord"]


# ---------------------------------------------------------------------
# Geometry generators
# ---------------------------------------------------------------------
def _baseline_aisle_polyline() -> np.ndarray:
    """Reference 12-vertex warehouse-aisle polyline.

    The same polygon underlies the original Deliverable 2 mobile-robot
    dataset; we expose it here so that the "mixed" variant family can
    resample it at arbitrary densities without duplicating the shape.
    """
    return np.array(
        [
            [0.0, 0.0],
            [2.0, 0.0],
            [4.0, 0.0],
            [4.0, -2.0],
            [4.0, -4.0],
            [6.5, -5.5],
            [9.0, -4.0],
            [9.0, -1.0],
            [8.0, 1.0],
            [7.0, 2.5],
            [5.0, 3.0],
            [3.0, 3.0],
        ]
    )


def mixed_aisle(n_waypoints: int) -> np.ndarray:
    """Resample the warehouse-aisle polyline at ``n_waypoints`` points.

    For the canonical count of 12 the original polyline is returned
    verbatim. For other counts the polyline is resampled at uniformly
    spaced arc-length positions so that the geometric character of the
    path (two sharp turns, one diagonal, one S-curve) is preserved.
    """
    poly = _baseline_aisle_polyline()
    if n_waypoints == poly.shape[0]:
        return poly.copy()
    return _arclength_resample(poly, n_waypoints)


def sharp_zigzag(n_waypoints: int, leg_length: float = 2.0) -> np.ndarray:
    """Rectangular zigzag path with 90-degree direction reversals.

    The path alternates between horizontal and vertical segments of
    fixed length, producing exactly ``n_waypoints - 1`` segments and
    ``n_waypoints - 2`` interior corners. Every interior corner is a
    sharp 90-degree turn, which stresses the methods' ability to
    handle abrupt direction changes.
    """
    if n_waypoints < 3:
        raise ValueError("zigzag requires at least 3 waypoints")
    pts = [(0.0, 0.0)]
    direction = "right"
    for i in range(n_waypoints - 1):
        x, y = pts[-1]
        if direction == "right":
            pts.append((x + leg_length, y))
            direction = "down" if i % 2 == 0 else "up"
        elif direction == "down":
            pts.append((x, y - leg_length))
            direction = "right"
        elif direction == "up":
            pts.append((x, y + leg_length))
            direction = "right"
    return np.asarray(pts, dtype=float)


def gradual_sinusoid(
    n_waypoints: int, x_span: float = 12.0, amplitude: float = 2.0, n_cycles: float = 1.5,
) -> np.ndarray:
    """Smooth sinusoidal path with no sharp corners.

    Generates ``n_waypoints`` points along ``y = A sin(2*pi*n_cycles*x/X)``.
    This is the gentle counterpart to :func:`sharp_zigzag` and is
    expected to be handled gracefully by every method.
    """
    if n_waypoints < 3:
        raise ValueError("sinusoid requires at least 3 waypoints")
    x = np.linspace(0.0, x_span, n_waypoints)
    y = amplitude * np.sin(2 * np.pi * n_cycles * x / x_span)
    return np.column_stack([x, y])


# ---------------------------------------------------------------------
# Resampling helpers
# ---------------------------------------------------------------------
def _arclength_resample(poly: np.ndarray, n_target: int) -> np.ndarray:
    """Sample ``n_target`` points equally spaced in arc length along ``poly``.

    The first and last vertices of the input polyline are preserved.
    """
    if n_target < 2:
        raise ValueError("n_target must be >= 2")
    seg = np.diff(poly, axis=0)
    seg_len = np.linalg.norm(seg, axis=1)
    cumulative = np.concatenate([[0.0], np.cumsum(seg_len)])
    total = cumulative[-1]
    targets = np.linspace(0.0, total, n_target)
    out = np.empty((n_target, poly.shape[1]))
    for i, s in enumerate(targets):
        # Find the polyline segment that contains arc length s.
        j = int(np.searchsorted(cumulative, s, side="right") - 1)
        j = min(max(j, 0), poly.shape[0] - 2)
        denom = cumulative[j + 1] - cumulative[j]
        alpha = 0.0 if denom <= 0 else (s - cumulative[j]) / denom
        out[i] = poly[j] + alpha * (poly[j + 1] - poly[j])
    return out


# ---------------------------------------------------------------------
# Time-stamp generators
# ---------------------------------------------------------------------
def time_stamps(
    waypoints: np.ndarray,
    spacing: SpacingMode = "chord",
    nominal_speed: float = 1.0,
    total_time: float | None = None,
) -> np.ndarray:
    """Attach time stamps to a set of waypoints.

    Parameters
    ----------
    waypoints : (n, d) array
    spacing : {"uniform", "chord"}
        ``"chord"`` makes ``dt_i`` proportional to the chord distance
        between consecutive waypoints, so that the implied nominal
        velocity ``chord_i / dt_i`` is constant. ``"uniform"`` makes
        every ``dt_i`` equal to ``total_time / (n - 1)``.
    nominal_speed : float
        Used only by ``"chord"`` to convert chord distances to seconds.
    total_time : float, optional
        Used only by ``"uniform"``. If omitted, defaults to the chord
        path length divided by ``nominal_speed`` so that the total
        traversal time matches what ``"chord"`` would produce.
    """
    n = waypoints.shape[0]
    if n < 2:
        raise ValueError("at least two waypoints are required")

    if spacing == "chord":
        chord = np.linalg.norm(np.diff(waypoints, axis=0), axis=1)
        dt = chord / nominal_speed
        return np.concatenate([[0.0], np.cumsum(dt)])

    if spacing == "uniform":
        if total_time is None:
            chord_total = float(np.sum(np.linalg.norm(np.diff(waypoints, axis=0), axis=1)))
            total_time = chord_total / nominal_speed
        return np.linspace(0.0, total_time, n)

    raise ValueError(f"unknown spacing mode {spacing!r}")
