"""Quantitative evaluation metrics for interpolated trajectories.

The metrics implemented here are the ones identified in the Deliverable
1 proposal: maximum velocity, maximum acceleration, RMS jerk, total
path length, waypoint deviation, and compute time. They are designed
to compare directly across methods on the same dataset, regardless of
whether the underlying interpolant produced the derivatives analytically
or whether they were extracted by post-hoc numerical differentiation.

Definitions
-----------

For a parameterised trajectory ``p(t)`` sampled densely on
``[t_0, t_N]``:

* **max_speed**       -- ``max ||v(t)||_2``
* **max_accel**       -- ``max ||a(t)||_2``
* **rms_jerk**        -- ``sqrt( mean( ||j(t)||_2^2 ) )``
* **path_length**     -- arc length ``int ||v(t)|| dt``, computed by
  trapezoidal quadrature on the dense sample
* **max_waypoint_err** -- maximum L2 distance between the requested
  waypoint and the trajectory evaluated at the corresponding waypoint
  time. Zero by construction for interpolating methods (linear, cubic,
  quintic). Non-zero for the B-spline approximation, which only passes
  through the endpoints of a clamped knot vector.
* **compute_time_ms** -- wall-clock time to construct *and* densely
  evaluate the interpolant.
"""

from __future__ import annotations

from dataclasses import dataclass, asdict
from typing import Callable, Iterable, Mapping

import numpy as np
import pandas as pd


@dataclass
class TrajectoryMetrics:
    method: str
    dataset: str
    n_waypoints: int
    n_samples: int
    max_speed: float
    max_accel: float
    rms_jerk: float
    path_length: float
    max_waypoint_err: float
    max_velocity_disc: float
    compute_time_ms: float

    def to_dict(self) -> dict:
        return asdict(self)


def _norm(vec: np.ndarray) -> np.ndarray:
    """Euclidean norm along the last axis. Works for 1-D and 2-D inputs."""
    if vec.ndim == 1:
        return np.abs(vec)
    return np.linalg.norm(vec, axis=-1)


def compute_metrics(
    method: str,
    dataset: str,
    t_knots: np.ndarray,
    p_knots: np.ndarray,
    t_dense: np.ndarray,
    p_dense: np.ndarray,
    v_dense: np.ndarray,
    a_dense: np.ndarray,
    j_dense: np.ndarray,
    compute_time_ms: float,
    p_at_knots: np.ndarray | None = None,
    max_velocity_disc: float = 0.0,
) -> TrajectoryMetrics:
    """Bundle the standard metrics for one method/dataset combination.

    Parameters
    ----------
    method, dataset : str
        Identifiers used for downstream tables and plots.
    t_knots, p_knots : ndarray
        The original waypoint times and values (used to compute
        ``max_waypoint_err`` against the interpolant evaluated at the
        waypoint times).
    t_dense, p_dense, v_dense, a_dense, j_dense : ndarray
        Dense samples of the interpolated trajectory and its
        derivatives.
    compute_time_ms : float
        Wall-clock cost of constructing and evaluating the interpolant.
    p_at_knots : ndarray, optional
        The interpolant evaluated at exactly ``t_knots``. If supplied
        this is used to compute ``max_waypoint_err``; otherwise the
        function falls back to nearest-sample lookup, which incurs an
        :math:`O(\\Delta t)` quantisation error proportional to the
        dense-sample spacing.
    """
    if p_at_knots is None and t_knots.size:
        p_at_knots = np.empty_like(p_knots, dtype=float)
        for i, tk in enumerate(t_knots):
            j = int(np.argmin(np.abs(t_dense - tk)))
            p_at_knots[i] = p_dense[j]
    if t_knots.size:
        waypoint_errs = _norm(p_at_knots - p_knots)
        max_waypoint_err = float(np.max(waypoint_errs))
    else:
        max_waypoint_err = 0.0

    speed = _norm(v_dense)
    accel = _norm(a_dense)
    jerk = _norm(j_dense)

    # Path length by trapezoidal quadrature of the speed.
    path_length = float(np.trapezoid(speed, t_dense))

    return TrajectoryMetrics(
        method=method,
        dataset=dataset,
        n_waypoints=int(t_knots.size),
        n_samples=int(t_dense.size),
        max_speed=float(np.max(speed)),
        max_accel=float(np.max(accel)),
        rms_jerk=float(np.sqrt(np.mean(jerk**2))),
        path_length=path_length,
        max_waypoint_err=max_waypoint_err,
        max_velocity_disc=float(max_velocity_disc),
        compute_time_ms=float(compute_time_ms),
    )


def metrics_to_dataframe(metrics: Iterable[TrajectoryMetrics]) -> pd.DataFrame:
    """Collect a list of :class:`TrajectoryMetrics` into a tidy DataFrame."""
    return pd.DataFrame([m.to_dict() for m in metrics])



# ----------------------------------------------------------------------
# Velocity discontinuity (C^1 violation) measurement
# ----------------------------------------------------------------------
def velocity_discontinuity(interp, t_knots: np.ndarray, eps: float | None = None) -> float:
    """Maximum velocity jump magnitude at internal knots.

    Evaluates the analytic first derivative slightly to the left and
    slightly to the right of every internal knot and returns the
    maximum Euclidean norm of the difference. For C^1 (or smoother)
    interpolants this should be zero up to the chosen ``eps`` and the
    interpolant's own evaluation noise. For piecewise-linear
    interpolation the result is the largest velocity jump along the
    path -- a directly meaningful measure of how violently a real
    actuator would have to react at each waypoint.

    Parameters
    ----------
    interp : object
        Any of the project interpolators implementing
        ``derivative(t_query, order=1)``.
    t_knots : (n,) ndarray
        Waypoint times. The boundary knots are ignored because
        evaluation outside the knot range is clamped.
    eps : float, optional
        Half-width of the one-sided sample around each knot. Defaults
        to ``1e-6 * (t_knots[-1] - t_knots[0])``, small enough that the
        result is dominated by genuine discontinuity, large enough to
        avoid floating-point noise in the analytic derivative.
    """
    if t_knots.size < 3:
        return 0.0
    if eps is None:
        eps = 1e-6 * float(t_knots[-1] - t_knots[0])

    interior = t_knots[1:-1]
    t_left = interior - eps
    t_right = interior + eps
    v_left = interp.derivative(t_left, order=1)
    v_right = interp.derivative(t_right, order=1)
    diff = v_right - v_left
    if diff.ndim == 1:
        magnitudes = np.abs(diff)
    else:
        magnitudes = np.linalg.norm(diff, axis=-1)
    return float(np.max(magnitudes))
