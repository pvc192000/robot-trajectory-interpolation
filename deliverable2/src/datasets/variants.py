"""Dataset variants used in the full experimental sweep.

Each entry returned by :func:`all_variants` is a fully self-contained
:class:`WaypointDataset` ready to be plugged into the experiment
runner. The variants exercise three independent axes from the
Deliverable 1 proposal:

* **Waypoint density** -- 5 (sparse), 12 (baseline), 20 (dense) on the
  mixed warehouse-aisle geometry.
* **Geometric character** -- sharp zigzag vs gradual sinusoid at a
  fixed 12 waypoints.
* **Time spacing** -- chord-length parameterisation vs uniform time
  spacing on the same 12-waypoint geometry.

All variants are 2-D mobile-robot scenarios so the metrics are
directly comparable. The 2-DOF arm dataset is treated separately
because its dimensional units are radians rather than metres.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .path_generators import (
    gradual_sinusoid,
    mixed_aisle,
    sharp_zigzag,
    time_stamps,
)


@dataclass(frozen=True)
class WaypointDataset:
    """Generic waypoint dataset used by the experiment runner."""

    name: str
    description: str
    t: np.ndarray
    p: np.ndarray
    units: str = "m"
    component_names: tuple = ("x", "y")
    geometry: str = "mixed"
    spacing: str = "chord"
    n_waypoints: int = field(init=False)

    def __post_init__(self):
        # n_waypoints is derived but useful as a column in result tables.
        object.__setattr__(self, "n_waypoints", int(self.t.shape[0]))


# ----------------------------------------------------------------------
# Mobile robot variants
# ----------------------------------------------------------------------
def make_mixed_variant(n_waypoints: int, spacing: str = "chord") -> WaypointDataset:
    """Mixed warehouse-aisle path, parameterised by waypoint count."""
    poly = mixed_aisle(n_waypoints)
    t = time_stamps(poly, spacing=spacing)
    label = {5: "sparse", 12: "baseline", 20: "dense"}.get(n_waypoints, f"n{n_waypoints}")
    return WaypointDataset(
        name=f"mobile_mixed_{label}_{spacing}",
        description=(
            f"Mobile robot warehouse aisle resampled at {n_waypoints} waypoints "
            f"with {spacing} time spacing."
        ),
        t=t,
        p=poly,
        units="m",
        component_names=("x", "y"),
        geometry="mixed",
        spacing=spacing,
    )


def make_sharp_variant(n_waypoints: int = 12, spacing: str = "chord") -> WaypointDataset:
    """Rectangular zigzag with 90-degree corners."""
    poly = sharp_zigzag(n_waypoints)
    t = time_stamps(poly, spacing=spacing)
    return WaypointDataset(
        name=f"mobile_sharp_n{n_waypoints}_{spacing}",
        description=(
            f"Sharp zigzag path with {n_waypoints} waypoints and 90-degree "
            f"interior corners; {spacing} time spacing."
        ),
        t=t,
        p=poly,
        units="m",
        component_names=("x", "y"),
        geometry="sharp",
        spacing=spacing,
    )


def make_gradual_variant(n_waypoints: int = 12, spacing: str = "chord") -> WaypointDataset:
    """Smooth sinusoidal path."""
    poly = gradual_sinusoid(n_waypoints)
    t = time_stamps(poly, spacing=spacing)
    return WaypointDataset(
        name=f"mobile_gradual_n{n_waypoints}_{spacing}",
        description=(
            f"Gradual sinusoidal path with {n_waypoints} waypoints; "
            f"{spacing} time spacing."
        ),
        t=t,
        p=poly,
        units="m",
        component_names=("x", "y"),
        geometry="gradual",
        spacing=spacing,
    )


# ----------------------------------------------------------------------
# Robot arm variant (kept simple: same dataset as Section 3 of D2)
# ----------------------------------------------------------------------
def make_arm_variant() -> WaypointDataset:
    """Wrap the canonical robot-arm dataset in the generic container."""
    from .robot_arm import generate_robot_arm_waypoints
    arm = generate_robot_arm_waypoints()
    return WaypointDataset(
        name="robot_arm_baseline",
        description=arm.description,
        t=arm.t,
        p=arm.theta_rad,
        units="rad",
        component_names=("theta_1", "theta_2"),
        geometry="arm",
        spacing="uniform",
    )


# ----------------------------------------------------------------------
# Top-level enumeration of variants
# ----------------------------------------------------------------------
def all_variants() -> list[WaypointDataset]:
    """Return every dataset evaluated in the full experiment.

    The order is significant for plotting routines that group by axis
    of variation: density sweep first, then geometry, then spacing,
    then the robot arm.
    """
    variants = []

    # Density sweep on the mixed geometry, chord-length parameterised.
    for n in (5, 12, 20):
        variants.append(make_mixed_variant(n_waypoints=n, spacing="chord"))

    # Geometry sweep at the baseline density (12) with chord spacing.
    variants.append(make_sharp_variant(n_waypoints=12, spacing="chord"))
    variants.append(make_gradual_variant(n_waypoints=12, spacing="chord"))

    # Spacing sweep at the baseline density (12) and mixed geometry.
    # The chord version is already covered by mobile_mixed_baseline_chord
    # above, so we add only the uniform counterpart here.
    variants.append(make_mixed_variant(n_waypoints=12, spacing="uniform"))

    # Robot arm baseline.
    variants.append(make_arm_variant())

    return variants
