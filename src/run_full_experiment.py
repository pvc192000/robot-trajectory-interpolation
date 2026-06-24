"""Run the full experiment from the command line.

This script extends ``src/main.py`` with the variant sweep, the
analytic-vs-empirical derivative validation, and the quintic boundary-
condition sensitivity study. It writes:

* ``results/full_experiment.csv`` -- one row per (method, dataset)
  combination across all variants.
* ``results/figures/sweep_*.png`` -- aggregate plots showing how every
  metric changes with waypoint density, geometry, and spacing.
* ``results/figures/validation_*.png`` -- analytic vs empirical
  derivative comparison for the cubic spline on the baseline dataset.
* ``results/figures/quintic_bc_*.png`` -- natural vs clamped quintic
  boundary-condition comparison.

Run from the repository root:

    .venv/bin/python -m src.run_full_experiment
"""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from .datasets import (
    WaypointDataset,
    all_variants,
    forward_kinematics,
    make_arm_variant,
    make_mixed_variant,
    make_sharp_variant,
    make_gradual_variant,
)
from .experiment import (
    METHOD_KEYS,
    analytic_vs_empirical_derivative,
    evaluate_all_methods,
    quintic_bc_sensitivity,
    run_experiment,
)
from .plots import METHOD_COLOURS, METHOD_LABELS


HERE = Path(__file__).resolve().parents[1]
RESULTS_DIR = HERE / "results"
FIGURES_DIR = RESULTS_DIR / "figures"


# ----------------------------------------------------------------------
# Sweep figures
# ----------------------------------------------------------------------
def plot_density_sweep(df: pd.DataFrame, out_path: Path):
    """Metrics vs waypoint count on the mixed geometry, chord spacing."""
    sub = df[
        (df["geometry"] == "mixed")
        & (df["spacing"] == "chord")
    ].copy()
    fig, axes = plt.subplots(1, 3, figsize=(13, 4))
    metrics = [
        ("max_accel", "max ||a||"),
        ("rms_jerk", "RMS ||jerk||"),
        ("path_length", "path length [m]"),
    ]
    for ax, (col, label) in zip(axes, metrics):
        for method in METHOD_KEYS:
            data = sub[sub["method"] == method].sort_values("n_waypoints")
            ax.plot(
                data["n_waypoints"],
                data[col],
                "o-",
                lw=1.5,
                ms=7,
                color=METHOD_COLOURS[method],
                label=METHOD_LABELS[method],
            )
        ax.set_xlabel("number of waypoints")
        ax.set_ylabel(label)
        ax.set_title(col)
        ax.grid(True, alpha=0.3)
        ax.set_xticks(sorted(sub["n_waypoints"].unique()))
    axes[0].legend(loc="best", fontsize=8)
    fig.suptitle(
        "Density sweep: mobile robot mixed geometry, chord-length spacing",
        y=1.02,
    )
    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_geometry_comparison(df: pd.DataFrame, out_path: Path):
    """Sharp zigzag vs gradual sinusoid at n=12, chord spacing."""
    sub = df[
        df["dataset"].isin(
            ["mobile_sharp_n12_chord", "mobile_gradual_n12_chord"]
        )
    ].copy()
    fig, axes = plt.subplots(1, 3, figsize=(13, 4))
    metrics = [
        ("max_accel", "max ||a||"),
        ("rms_jerk", "RMS ||jerk||"),
        ("max_velocity_disc", "max velocity disc."),
    ]
    geom_labels = {"sharp": "sharp 90° corners", "gradual": "smooth sinusoid"}
    for ax, (col, label) in zip(axes, metrics):
        x_pos = np.arange(len(METHOD_KEYS))
        width = 0.35
        for offset, geom in zip([-width / 2, width / 2], ["sharp", "gradual"]):
            data = sub[sub["geometry"] == geom].set_index("method")
            vals = [data.loc[m, col] for m in METHOD_KEYS]
            ax.bar(x_pos + offset, vals, width, label=geom_labels[geom])
        ax.set_xticks(x_pos)
        ax.set_xticklabels(
            [METHOD_LABELS[m] for m in METHOD_KEYS], rotation=15, ha="right"
        )
        ax.set_ylabel(label)
        ax.set_title(col)
        ax.grid(True, alpha=0.3, axis="y")
        if col == "max_velocity_disc":
            ax.set_yscale("symlog", linthresh=1e-5)
    axes[0].legend(loc="best", fontsize=8)
    fig.suptitle("Geometry sweep: sharp vs gradual, n=12, chord-length spacing", y=1.02)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_spacing_comparison(df: pd.DataFrame, out_path: Path):
    """Chord-length vs uniform time spacing at n=12 mixed geometry."""
    sub = df[
        df["dataset"].isin(
            ["mobile_mixed_baseline_chord", "mobile_mixed_baseline_uniform"]
        )
    ].copy()
    fig, axes = plt.subplots(1, 3, figsize=(13, 4))
    metrics = [
        ("max_speed", "max ||v||"),
        ("max_accel", "max ||a||"),
        ("rms_jerk", "RMS ||jerk||"),
    ]
    spacing_labels = {"chord": "chord-length", "uniform": "uniform"}
    for ax, (col, label) in zip(axes, metrics):
        x_pos = np.arange(len(METHOD_KEYS))
        width = 0.35
        for offset, spacing in zip([-width / 2, width / 2], ["chord", "uniform"]):
            data = sub[sub["spacing"] == spacing].set_index("method")
            vals = [data.loc[m, col] for m in METHOD_KEYS]
            ax.bar(x_pos + offset, vals, width, label=spacing_labels[spacing])
        ax.set_xticks(x_pos)
        ax.set_xticklabels(
            [METHOD_LABELS[m] for m in METHOD_KEYS], rotation=15, ha="right"
        )
        ax.set_ylabel(label)
        ax.set_title(col)
        ax.grid(True, alpha=0.3, axis="y")
    axes[0].legend(loc="best", fontsize=8)
    fig.suptitle("Spacing sweep: chord-length vs uniform, n=12, mixed geometry", y=1.02)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_density_trajectories(out_path: Path):
    """Visualise the resampled waypoint sets for the density sweep."""
    fig, axes = plt.subplots(1, 3, figsize=(13, 5), sharex=True, sharey=True)
    for ax, n in zip(axes, (5, 12, 20)):
        ds = make_mixed_variant(n_waypoints=n, spacing="chord")
        runs = evaluate_all_methods(ds, n_dense=600)
        for method in METHOD_KEYS:
            traj = runs[method].p
            ax.plot(
                traj[:, 0],
                traj[:, 1],
                lw=1.4,
                color=METHOD_COLOURS[method],
                label=METHOD_LABELS[method],
            )
        ax.plot(
            ds.p[:, 0],
            ds.p[:, 1],
            "ko",
            markersize=5,
            markerfacecolor="white",
            markeredgewidth=1.2,
            label="waypoints",
            zorder=5,
        )
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3)
        ax.set_title(f"n = {n} waypoints")
        ax.set_xlabel("x [m]")
    axes[0].set_ylabel("y [m]")
    axes[0].legend(loc="best", fontsize=7, ncol=2)
    fig.suptitle("Mobile robot trajectories at varying waypoint density", y=1.02)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_geometry_trajectories(out_path: Path):
    """Side-by-side sharp vs gradual interpolation."""
    fig, axes = plt.subplots(1, 2, figsize=(11, 5))
    titles = ["Sharp zigzag (n=12)", "Gradual sinusoid (n=12)"]
    builders = [
        lambda: make_sharp_variant(n_waypoints=12, spacing="chord"),
        lambda: make_gradual_variant(n_waypoints=12, spacing="chord"),
    ]
    for ax, build, title in zip(axes, builders, titles):
        ds = build()
        runs = evaluate_all_methods(ds, n_dense=600)
        for method in METHOD_KEYS:
            traj = runs[method].p
            ax.plot(
                traj[:, 0], traj[:, 1], lw=1.5,
                color=METHOD_COLOURS[method], label=METHOD_LABELS[method],
            )
        ax.plot(
            ds.p[:, 0], ds.p[:, 1], "ko", markersize=5,
            markerfacecolor="white", markeredgewidth=1.2,
            label="waypoints", zorder=5,
        )
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3)
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_title(title)
    axes[0].legend(loc="best", fontsize=7, ncol=2)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


# ----------------------------------------------------------------------
# Validation figure: analytic vs empirical derivatives
# ----------------------------------------------------------------------
def plot_analytic_vs_empirical(diag: dict, out_path: Path):
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    t = diag["t"]
    pairs = [
        ("v", "velocity [m/s]"),
        ("a", "acceleration [m/s²]"),
        ("j", "jerk [m/s³]"),
    ]
    for ax, (key, ylabel) in zip(axes, pairs):
        analytic = diag[f"{key}_analytic"]
        empirical = diag[f"{key}_empirical"]
        ana_norm = (
            np.linalg.norm(analytic, axis=-1) if analytic.ndim > 1 else np.abs(analytic)
        )
        emp_norm = (
            np.linalg.norm(empirical, axis=-1) if empirical.ndim > 1 else np.abs(empirical)
        )
        ax.plot(t, ana_norm, "-", lw=1.5, color="#1f77b4", label="analytic (interp.derivative)")
        ax.plot(t, emp_norm, "--", lw=1.0, color="#d62728", label="empirical (centered FD)")
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)
    axes[0].legend(loc="best", fontsize=9)
    axes[0].set_title(
        f"Analytic vs empirical derivatives "
        f"({diag['method']} on {diag['dataset']})"
    )
    axes[2].set_xlabel("time [s]")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


# ----------------------------------------------------------------------
# Quintic boundary-condition sensitivity figure
# ----------------------------------------------------------------------
def plot_quintic_bc(sens: dict, out_path: Path):
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    t = sens["t"]
    pairs = [("v", "speed [m/s]"), ("a", "|accel| [m/s²]"), ("j", "|jerk| [m/s³]")]
    colours = {"natural": "#2ca02c", "clamped": "#9467bd"}
    for ax, (key, ylabel) in zip(axes, pairs):
        for variant in ("natural", "clamped"):
            arr = sens[variant][key]
            mag = np.linalg.norm(arr, axis=-1) if arr.ndim > 1 else np.abs(arr)
            ax.plot(
                t, mag, lw=1.5,
                color=colours[variant],
                label=f"quintic, {variant} BC",
            )
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)
    axes[0].set_title(
        "Quintic spline boundary-condition sensitivity "
        "(baseline 12-waypoint mobile robot)"
    )
    axes[2].set_xlabel("time [s]")
    axes[0].legend(loc="best")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


# ----------------------------------------------------------------------
# Entry point
# ----------------------------------------------------------------------
def main() -> Path:
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    FIGURES_DIR.mkdir(parents=True, exist_ok=True)

    print("Building all dataset variants ...")
    variants = all_variants()
    for v in variants:
        print(
            f"  {v.name:40s} n={v.n_waypoints:3d} "
            f"geom={v.geometry:8s} spacing={v.spacing:8s}"
        )

    print("\nRunning experiment ...")
    df = run_experiment(variants)
    out_csv = RESULTS_DIR / "full_experiment.csv"
    df.to_csv(out_csv, index=False, float_format="%.6f")
    print(f"  Wrote {out_csv} ({len(df)} rows)")

    print("\nGenerating sweep figures ...")
    plot_density_sweep(df, FIGURES_DIR / "sweep_density.png")
    plot_geometry_comparison(df, FIGURES_DIR / "sweep_geometry.png")
    plot_spacing_comparison(df, FIGURES_DIR / "sweep_spacing.png")
    plot_density_trajectories(FIGURES_DIR / "sweep_density_trajectories.png")
    plot_geometry_trajectories(FIGURES_DIR / "sweep_geometry_trajectories.png")
    print("  Wrote sweep_*.png")

    print("\nValidating analytic derivatives ...")
    baseline = next(v for v in variants if v.name == "mobile_mixed_baseline_chord")
    for method in ("cubic", "quintic", "bspline"):
        diag = analytic_vs_empirical_derivative(baseline, method=method)
        print(
            f"  {method:8s}: v_err={diag['v_max_err']:.2e} "
            f"a_err={diag['a_max_err']:.2e} j_err={diag['j_max_err']:.2e}"
        )
    diag_cubic = analytic_vs_empirical_derivative(baseline, method="cubic")
    plot_analytic_vs_empirical(
        diag_cubic, FIGURES_DIR / "validation_analytic_vs_empirical.png"
    )

    print("\nQuintic BC sensitivity ...")
    sens = quintic_bc_sensitivity(baseline)
    print(f"  natural: {sens['natural_summary']}")
    print(f"  clamped: {sens['clamped_summary']}")
    plot_quintic_bc(sens, FIGURES_DIR / "quintic_bc_sensitivity.png")

    print(f"\nAll outputs written under {RESULTS_DIR}")
    return out_csv


if __name__ == "__main__":
    main()
