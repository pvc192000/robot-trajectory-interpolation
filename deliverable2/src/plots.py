"""Plotting utilities for the comparative trajectory study.

Produces three families of figures, all written to ``results/figures``:

1. **Trajectory overlay** -- ``(x, y)`` paths produced by every method
   on the same axes, with the original waypoints highlighted.

2. **Kinematic profiles** -- speed, acceleration magnitude, and jerk
   magnitude versus time, one panel each, all methods overlaid. This
   exposes how each method handles the smoothness budget.

3. **Per-component profiles** -- for the robot arm dataset, separate
   plots per joint angle showing position/velocity/acceleration/jerk.

The styling is deliberately conservative -- plain colours, no fills, a
single legend per figure -- so the figures reproduce well in printed
papers and presentations.
"""

from __future__ import annotations

from pathlib import Path
from typing import Mapping

import matplotlib.pyplot as plt
import numpy as np


# Stable colour mapping per method so figures from different runs are
# visually consistent.
METHOD_COLOURS = {
    "linear": "#888888",
    "cubic": "#1f77b4",
    "quintic": "#2ca02c",
    "bspline": "#d62728",
}

METHOD_LABELS = {
    "linear": "Linear",
    "cubic": "Cubic spline",
    "quintic": "Quintic spline",
    "bspline": "B-spline (cubic)",
}


def _style_method(ax, method: str, **kwargs):
    """Return matplotlib kwargs with the canonical colour and label."""
    return dict(
        color=METHOD_COLOURS.get(method, "k"),
        label=METHOD_LABELS.get(method, method),
        **kwargs,
    )


def plot_trajectory_overlay(
    waypoints: np.ndarray,
    trajectories: Mapping[str, np.ndarray],
    out_path: Path,
    title: str,
    xlabel: str = "x [m]",
    ylabel: str = "y [m]",
):
    """Overlay 2-D trajectories from each method.

    ``waypoints`` has shape ``(n, 2)`` and ``trajectories`` maps a
    method key to an ``(N, 2)`` dense sample.
    """
    fig, ax = plt.subplots(figsize=(8, 7))
    for method, traj in trajectories.items():
        ax.plot(traj[:, 0], traj[:, 1], lw=1.6, **_style_method(ax, method))
    ax.plot(
        waypoints[:, 0],
        waypoints[:, 1],
        "ko",
        markersize=7,
        markerfacecolor="white",
        markeredgewidth=1.5,
        label="waypoints",
        zorder=5,
    )
    for i, (x, y) in enumerate(waypoints):
        ax.annotate(
            str(i),
            (x, y),
            xytext=(5, 5),
            textcoords="offset points",
            fontsize=8,
            color="0.3",
        )
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", framealpha=0.9)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_kinematic_profiles(
    t: np.ndarray,
    profiles: Mapping[str, dict],
    out_path: Path,
    title: str,
):
    """Plot speed / accel / jerk magnitudes over time for all methods.

    ``profiles[method]`` must contain keys ``"v"``, ``"a"``, ``"j"`` of
    shape ``(N,)`` (scalar input) or ``(N, d)`` (vector input). Norms
    are taken on the last axis for vector inputs.
    """
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)

    def _norm(x):
        if x.ndim == 1:
            return np.abs(x)
        return np.linalg.norm(x, axis=-1)

    for method, prof in profiles.items():
        axes[0].plot(t, _norm(prof["v"]), lw=1.5, **_style_method(axes[0], method))
        axes[1].plot(t, _norm(prof["a"]), lw=1.5, **_style_method(axes[1], method))
        axes[2].plot(t, _norm(prof["j"]), lw=1.5, **_style_method(axes[2], method))

    axes[0].set_ylabel("Speed [units/s]")
    axes[1].set_ylabel("|Accel| [units/s²]")
    axes[2].set_ylabel("|Jerk| [units/s³]")
    axes[2].set_xlabel("Time [s]")
    axes[0].set_title(title)
    for ax in axes:
        ax.grid(True, alpha=0.3)
    axes[0].legend(loc="upper right", framealpha=0.9, ncol=2)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_component_profiles(
    t: np.ndarray,
    profiles: Mapping[str, dict],
    out_path: Path,
    title: str,
    component_names: list,
    units: str = "rad",
):
    """Plot per-component profiles (position/vel/accel/jerk) as a grid.

    For a ``d``-component trajectory, produces a ``4 x d`` grid where
    rows are derivative orders 0..3 and columns are components.
    """
    sample_method = next(iter(profiles))
    p = profiles[sample_method]["p"]
    if p.ndim == 1:
        p = p[:, None]
    d = p.shape[1]

    fig, axes = plt.subplots(4, d, figsize=(4 * d + 1, 10), sharex=True)
    if d == 1:
        axes = axes[:, None]

    keys = ["p", "v", "a", "j"]
    ylabels = [
        f"position [{units}]",
        f"velocity [{units}/s]",
        f"accel [{units}/s²]",
        f"jerk [{units}/s³]",
    ]

    for col in range(d):
        for row, key in enumerate(keys):
            ax = axes[row, col]
            for method, prof in profiles.items():
                arr = prof[key]
                if arr.ndim == 1:
                    arr = arr[:, None]
                ax.plot(t, arr[:, col], lw=1.3, **_style_method(ax, method))
            if row == 0:
                ax.set_title(component_names[col])
            if col == 0:
                ax.set_ylabel(ylabels[row])
            if row == 3:
                ax.set_xlabel("Time [s]")
            ax.grid(True, alpha=0.3)
    axes[0, 0].legend(loc="best", framealpha=0.9, fontsize=8)
    fig.suptitle(title, y=1.0)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
