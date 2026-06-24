"""Experiment runner for the comparative study.

Provides a single high-level entry point :func:`run_experiment` that
takes a list of :class:`WaypointDataset` variants and returns a tidy
long-form pandas DataFrame containing every metric for every method
on every dataset. The same runner powers both the offline
``src/main.py`` orchestrator and the Jupyter notebook deliverable, so
that any change in metric definitions flows through to all artefacts.

The runner also exposes two diagnostic helpers used by the validation
sections of the notebook:

* :func:`analytic_vs_empirical_derivative` -- compares the analytic
  derivative provided by an interpolation method against the centered
  finite-difference estimate from :mod:`src.differentiation`. This is
  the validation step that the proposal calls for.
* :func:`quintic_bc_sensitivity` -- builds a quintic spline with two
  different boundary conditions (natural and clamped using the cubic
  spline's end velocities) and returns their kinematic profiles for
  side-by-side plotting.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Iterable, Mapping

import numpy as np
import pandas as pd

from .datasets import WaypointDataset
from .differentiation import differentiate
from .interpolation import (
    BSplineApproximation,
    CubicSpline,
    LinearInterpolator,
    QuinticSpline,
)
from .metrics import (
    TrajectoryMetrics,
    compute_metrics,
    metrics_to_dataframe,
    velocity_discontinuity,
)


METHOD_KEYS = ("linear", "cubic", "quintic", "bspline")
DEFAULT_N_DENSE = 1000


# ----------------------------------------------------------------------
# Method construction
# ----------------------------------------------------------------------
def build_method(name: str, t: np.ndarray, p: np.ndarray):
    """Build an interpolant by short name.

    Splines use ``natural`` boundary conditions (start/stop at rest)
    so all methods are compared under the same physical assumption.
    """
    if name == "linear":
        return LinearInterpolator(t, p)
    if name == "cubic":
        return CubicSpline(t, p, bc="natural")
    if name == "quintic":
        return QuinticSpline(t, p, bc="natural")
    if name == "bspline":
        return BSplineApproximation(t, p, degree=3)
    raise KeyError(f"unknown method {name!r}")


# ----------------------------------------------------------------------
# Per-dataset evaluation
# ----------------------------------------------------------------------
@dataclass
class MethodRun:
    """Bundle of dense samples and timings for one method on one dataset."""

    method: str
    interp: object
    p: np.ndarray
    v: np.ndarray
    a: np.ndarray
    j: np.ndarray
    compute_ms: float


def evaluate_method(name: str, dataset: WaypointDataset, t_query: np.ndarray) -> MethodRun:
    """Build, evaluate, and time one method on one dataset."""
    start = time.perf_counter()
    interp = build_method(name, dataset.t, dataset.p)
    p_d = interp(t_query)
    v_d = interp.derivative(t_query, order=1)
    a_d = interp.derivative(t_query, order=2)
    j_d = interp.derivative(t_query, order=3)
    elapsed_ms = (time.perf_counter() - start) * 1000.0
    return MethodRun(
        method=name,
        interp=interp,
        p=p_d,
        v=v_d,
        a=a_d,
        j=j_d,
        compute_ms=elapsed_ms,
    )


def evaluate_all_methods(
    dataset: WaypointDataset, n_dense: int = DEFAULT_N_DENSE
) -> dict[str, MethodRun]:
    """Run every method on a single dataset and return them keyed by name."""
    t_query = np.linspace(dataset.t[0], dataset.t[-1], n_dense)
    return {name: evaluate_method(name, dataset, t_query) for name in METHOD_KEYS}


# ----------------------------------------------------------------------
# Top-level experiment runner
# ----------------------------------------------------------------------
def run_experiment(
    datasets: Iterable[WaypointDataset],
    n_dense: int = DEFAULT_N_DENSE,
    methods: Iterable[str] = METHOD_KEYS,
) -> pd.DataFrame:
    """Evaluate every (dataset, method) pair and return a tidy DataFrame.

    Returned columns:
        method, dataset, n_waypoints, n_samples, max_speed, max_accel,
        rms_jerk, path_length, max_waypoint_err, max_velocity_disc,
        compute_time_ms, geometry, spacing, units.
    """
    rows: list[TrajectoryMetrics] = []
    extra_columns: list[dict] = []
    for ds in datasets:
        t_query = np.linspace(ds.t[0], ds.t[-1], n_dense)
        for method in methods:
            run = evaluate_method(method, ds, t_query)
            v_disc = velocity_discontinuity(run.interp, ds.t)
            m = compute_metrics(
                method=method,
                dataset=ds.name,
                t_knots=ds.t,
                p_knots=ds.p,
                t_dense=t_query,
                p_dense=run.p,
                v_dense=run.v,
                a_dense=run.a,
                j_dense=run.j,
                compute_time_ms=run.compute_ms,
                p_at_knots=run.interp(ds.t),
                max_velocity_disc=v_disc,
            )
            rows.append(m)
            extra_columns.append(
                {
                    "geometry": ds.geometry,
                    "spacing": ds.spacing,
                    "units": ds.units,
                }
            )

    df = metrics_to_dataframe(rows)
    df_extra = pd.DataFrame(extra_columns)
    return pd.concat([df, df_extra], axis=1)


# ----------------------------------------------------------------------
# Diagnostic 1: analytic vs empirical derivative validation
# ----------------------------------------------------------------------
def analytic_vs_empirical_derivative(
    dataset: WaypointDataset,
    method: str = "cubic",
    n_dense: int = DEFAULT_N_DENSE,
) -> dict:
    """Compare a method's analytic derivatives against finite differences.

    Returns a dictionary with the dense time grid, the analytic
    velocity / acceleration / jerk arrays, the corresponding finite-
    difference estimates, and per-derivative-order error norms. This
    is the empirical validation called for in step 4 of the
    Deliverable 1 methodology.
    """
    t_query = np.linspace(dataset.t[0], dataset.t[-1], n_dense)
    run = evaluate_method(method, dataset, t_query)
    v_fd = differentiate(t_query, run.p, order=1, scheme="auto")
    a_fd = differentiate(t_query, run.p, order=2, scheme="auto")
    j_fd = differentiate(t_query, run.p, order=3, scheme="auto")

    def _interior_diff(analytic, empirical, k):
        # Drop the boundary samples where the FD scheme falls back to
        # one-sided stencils so the comparison is apples-to-apples.
        skip = max(2, k + 1)
        if analytic.ndim == 1:
            return float(np.max(np.abs(analytic[skip:-skip] - empirical[skip:-skip])))
        return float(
            np.max(np.linalg.norm(analytic[skip:-skip] - empirical[skip:-skip], axis=-1))
        )

    return {
        "dataset": dataset.name,
        "method": method,
        "t": t_query,
        "v_analytic": run.v,
        "v_empirical": v_fd,
        "a_analytic": run.a,
        "a_empirical": a_fd,
        "j_analytic": run.j,
        "j_empirical": j_fd,
        "v_max_err": _interior_diff(run.v, v_fd, 1),
        "a_max_err": _interior_diff(run.a, a_fd, 2),
        "j_max_err": _interior_diff(run.j, j_fd, 3),
    }


# ----------------------------------------------------------------------
# Diagnostic 2: quintic boundary condition sensitivity
# ----------------------------------------------------------------------
def quintic_bc_sensitivity(
    dataset: WaypointDataset,
    n_dense: int = DEFAULT_N_DENSE,
) -> dict:
    """Compare natural vs cubic-derived clamped boundary conditions.

    The proposal flagged the quintic-spline result with natural
    boundary conditions (start and stop at rest in both velocity and
    acceleration) as potentially harsher than necessary. This helper
    builds a second quintic spline whose end velocities and end
    accelerations are read off the corresponding cubic spline, which
    is one of the most common practical choices, and reports the
    derivative-norm metrics for both variants.
    """
    t_query = np.linspace(dataset.t[0], dataset.t[-1], n_dense)
    cubic = CubicSpline(dataset.t, dataset.p, bc="natural")
    v0 = cubic.derivative(np.array([dataset.t[0]]), order=1)[0]
    vN = cubic.derivative(np.array([dataset.t[-1]]), order=1)[0]
    a0 = cubic.derivative(np.array([dataset.t[0]]), order=2)[0]
    aN = cubic.derivative(np.array([dataset.t[-1]]), order=2)[0]

    natural = QuinticSpline(dataset.t, dataset.p, bc="natural")
    clamped = QuinticSpline(
        dataset.t,
        dataset.p,
        bc="clamped",
        v0=v0,
        vN=vN,
        a0=a0,
        aN=aN,
    )

    def _profile(interp):
        return {
            "p": interp(t_query),
            "v": interp.derivative(t_query, order=1),
            "a": interp.derivative(t_query, order=2),
            "j": interp.derivative(t_query, order=3),
        }

    def _summarise(prof):
        speed = np.linalg.norm(prof["v"], axis=-1) if prof["v"].ndim > 1 else np.abs(prof["v"])
        accel = np.linalg.norm(prof["a"], axis=-1) if prof["a"].ndim > 1 else np.abs(prof["a"])
        jerk = np.linalg.norm(prof["j"], axis=-1) if prof["j"].ndim > 1 else np.abs(prof["j"])
        return dict(
            max_speed=float(np.max(speed)),
            max_accel=float(np.max(accel)),
            rms_jerk=float(np.sqrt(np.mean(jerk**2))),
        )

    natural_prof = _profile(natural)
    clamped_prof = _profile(clamped)
    return {
        "t": t_query,
        "natural": natural_prof,
        "clamped": clamped_prof,
        "natural_summary": _summarise(natural_prof),
        "clamped_summary": _summarise(clamped_prof),
        "boundary_velocities": {
            "v0": np.asarray(v0),
            "vN": np.asarray(vN),
            "a0": np.asarray(a0),
            "aN": np.asarray(aN),
        },
    }
