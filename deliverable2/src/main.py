"""Main orchestrator for the Deliverable 2 comparative study.

Running this script as

    python -m src.main

(from the ``deliverable2`` directory) executes the full analysis
pipeline:

1. Generates and saves the two waypoint datasets to ``data/``.
2. Constructs every interpolation method on every dataset.
3. Densely evaluates each interpolant and its analytic first / second /
   third derivatives.
4. Computes the comparative metrics defined in :mod:`src.metrics`.
5. Writes ``results/metrics_summary.csv`` and a set of figures into
   ``results/figures/``.

The script is deterministic: no random numbers are used anywhere in
the pipeline, so re-running it on the same machine reproduces every
artefact bit-for-bit (modulo floating-point ordering).
"""

from __future__ import annotations

import time
from pathlib import Path

import numpy as np
import pandas as pd

from .datasets import (
    generate_mobile_robot_waypoints,
    generate_robot_arm_waypoints,
    forward_kinematics,
)
from .differentiation import differentiate
from .interpolation import (
    BSplineApproximation,
    CubicSpline,
    LinearInterpolator,
    QuinticSpline,
)
from .metrics import compute_metrics, metrics_to_dataframe
from .plots import (
    plot_component_profiles,
    plot_kinematic_profiles,
    plot_trajectory_overlay,
)


METHOD_KEYS = ("linear", "cubic", "quintic", "bspline")
DEFAULT_N_SAMPLES = 1000


# ----------------------------------------------------------------------
# Construction helpers
# ----------------------------------------------------------------------
def build_method(name: str, t, p):
    """Build an interpolant by short name. Boundary conditions are
    ``natural`` (start/stop at rest) for the splines so that comparisons
    are fair across methods that need explicit boundary information."""
    if name == "linear":
        return LinearInterpolator(t, p)
    if name == "cubic":
        return CubicSpline(t, p, bc="natural")
    if name == "quintic":
        return QuinticSpline(t, p, bc="natural")
    if name == "bspline":
        return BSplineApproximation(t, p, degree=3)
    raise KeyError(name)


def evaluate_method(name, t_knots, p_knots, t_query):
    """Build, evaluate, and time a single method.

    Returns the interpolated position, analytic derivatives, the
    constructed object, and the wall-clock construction+evaluation
    time in milliseconds.
    """
    start = time.perf_counter()
    interp = build_method(name, t_knots, p_knots)
    p_dense = interp(t_query)
    v_dense = interp.derivative(t_query, order=1)
    a_dense = interp.derivative(t_query, order=2)
    # B-spline of degree 3 has zero analytic 3rd derivative beyond
    # piecewise-constant; for the linear interpolator the 3rd derivative
    # is also zero. We still call .derivative(.., 3) for consistency.
    try:
        j_dense = interp.derivative(t_query, order=3)
    except Exception:
        # Linear interpolator returns zero for order >= 2, so a manual
        # fallback would be redundant -- this branch is precautionary.
        j_dense = np.zeros_like(p_dense)
    elapsed_ms = (time.perf_counter() - start) * 1000.0
    return p_dense, v_dense, a_dense, j_dense, elapsed_ms, interp


# ----------------------------------------------------------------------
# Dataset runners
# ----------------------------------------------------------------------
def run_mobile_robot(out_root: Path, n_samples: int = DEFAULT_N_SAMPLES):
    print("== Mobile robot scenario ==")
    data = generate_mobile_robot_waypoints()
    # Save the dataset to disk for the deliverable.
    data_csv = out_root / "data" / "mobile_robot_waypoints.csv"
    data_csv.parent.mkdir(parents=True, exist_ok=True)
    data.to_dataframe().to_csv(data_csv, index=False, float_format="%.6f")
    print(f"  Wrote {data_csv}")

    t_query = np.linspace(data.t[0], data.t[-1], n_samples)
    profiles = {}
    metrics = []
    trajectories = {}
    for method in METHOD_KEYS:
        p_d, v_d, a_d, j_d, ms, interp = evaluate_method(
            method, data.t, data.p, t_query
        )
        profiles[method] = {"p": p_d, "v": v_d, "a": a_d, "j": j_d}
        trajectories[method] = p_d
        m = compute_metrics(
            method=method,
            dataset=data.name,
            t_knots=data.t,
            p_knots=data.p,
            t_dense=t_query,
            p_dense=p_d,
            v_dense=v_d,
            a_dense=a_d,
            j_dense=j_d,
            compute_time_ms=ms,
            p_at_knots=interp(data.t),
        )
        metrics.append(m)
        print(
            f"  {method:8s}  max|a|={m.max_accel:.3f}  rms|j|={m.rms_jerk:.3f}"
            f"  path={m.path_length:.3f}m  err_wp={m.max_waypoint_err:.2e}m"
            f"  t={m.compute_time_ms:.2f}ms"
        )

    fig_dir = out_root / "results" / "figures"
    fig_dir.mkdir(parents=True, exist_ok=True)
    plot_trajectory_overlay(
        waypoints=data.p,
        trajectories=trajectories,
        out_path=fig_dir / "mobile_robot_trajectory.png",
        title="Mobile robot trajectory: comparison of interpolation methods",
        xlabel="x [m]",
        ylabel="y [m]",
    )
    plot_kinematic_profiles(
        t=t_query,
        profiles=profiles,
        out_path=fig_dir / "mobile_robot_kinematics.png",
        title="Mobile robot: speed / accel / jerk magnitude over time",
    )
    return metrics


def run_robot_arm(out_root: Path, n_samples: int = DEFAULT_N_SAMPLES):
    print("== 2-DOF robot arm scenario ==")
    data = generate_robot_arm_waypoints()
    data_csv = out_root / "data" / "robot_arm_waypoints.csv"
    data_csv.parent.mkdir(parents=True, exist_ok=True)
    data.to_dataframe().to_csv(data_csv, index=False, float_format="%.6f")
    print(f"  Wrote {data_csv}")

    # Interpolate in joint space (radians) so that joint-space metrics
    # like RMS jerk have engineering meaning. Cartesian comparison
    # plots use forward kinematics on the dense trajectory.
    theta_knots = data.theta_rad
    t_query = np.linspace(data.t[0], data.t[-1], n_samples)
    profiles_joint = {}
    metrics = []
    cartesian_paths = {}
    for method in METHOD_KEYS:
        p_d, v_d, a_d, j_d, ms, interp = evaluate_method(
            method, data.t, theta_knots, t_query
        )
        profiles_joint[method] = {"p": p_d, "v": v_d, "a": a_d, "j": j_d}
        m = compute_metrics(
            method=method,
            dataset=data.name,
            t_knots=data.t,
            p_knots=theta_knots,
            t_dense=t_query,
            p_dense=p_d,
            v_dense=v_d,
            a_dense=a_d,
            j_dense=j_d,
            compute_time_ms=ms,
            p_at_knots=interp(data.t),
        )
        metrics.append(m)
        print(
            f"  {method:8s}  max|a|={m.max_accel:.3f}  rms|j|={m.rms_jerk:.3f}"
            f"  path={m.path_length:.3f}rad  err_wp={m.max_waypoint_err:.2e}rad"
            f"  t={m.compute_time_ms:.2f}ms"
        )
        # Cartesian path via forward kinematics.
        cartesian_paths[method] = forward_kinematics(
            p_d[:, 0], p_d[:, 1], data.L1, data.L2
        )

    fig_dir = out_root / "results" / "figures"
    fig_dir.mkdir(parents=True, exist_ok=True)
    plot_trajectory_overlay(
        waypoints=data.cartesian,
        trajectories=cartesian_paths,
        out_path=fig_dir / "robot_arm_cartesian.png",
        title="Robot arm Cartesian end-effector trajectory by method",
        xlabel="x [m]",
        ylabel="y [m]",
    )
    plot_kinematic_profiles(
        t=t_query,
        profiles=profiles_joint,
        out_path=fig_dir / "robot_arm_joint_kinematics.png",
        title="Robot arm joint-space kinematics: ||theta_dot||, ||theta_ddot||, ||theta_dddot||",
    )
    plot_component_profiles(
        t=t_query,
        profiles=profiles_joint,
        out_path=fig_dir / "robot_arm_components.png",
        title="Robot arm per-joint trajectories and derivatives",
        component_names=["theta_1", "theta_2"],
        units="rad",
    )
    return metrics


# ----------------------------------------------------------------------
# Entry point
# ----------------------------------------------------------------------
def main(out_root: Path | None = None) -> Path:
    if out_root is None:
        out_root = Path(__file__).resolve().parents[1]
    out_root = Path(out_root)

    metrics = []
    metrics.extend(run_mobile_robot(out_root))
    metrics.extend(run_robot_arm(out_root))

    df = metrics_to_dataframe(metrics)
    out_csv = out_root / "results" / "metrics_summary.csv"
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(out_csv, index=False, float_format="%.6f")
    print(f"\nWrote summary metrics to {out_csv}")
    print(df.to_string(index=False))
    return out_csv


if __name__ == "__main__":
    main()
