"""Mobile robot 2-D waypoint dataset.

Scenario
--------

A differential-drive mobile robot must traverse a warehouse aisle that
contains:

1. A straight entry segment (loading dock to first turn).
2. A 90-degree right turn (rounding a shelving unit).
3. A diagonal traverse to a pick station.
4. A 90-degree left turn back to the main aisle.
5. A gentle S-curve to exit.

This combination produces three sharp direction changes (which stress
linear and cubic methods) and one smoothly varying section (where the
methods are expected to agree). The waypoints are spaced approximately
1 second apart with two intentionally short segments to test
non-uniform spacing.

Coordinates are in metres, times are in seconds.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd


@dataclass(frozen=True)
class MobileRobotWaypoints:
    t: np.ndarray   # (n,) waypoint times in seconds
    p: np.ndarray   # (n, 2) waypoint positions in metres
    name: str
    description: str

    def to_dataframe(self) -> pd.DataFrame:
        return pd.DataFrame(
            {
                "t_s": self.t,
                "x_m": self.p[:, 0],
                "y_m": self.p[:, 1],
            }
        )


def generate_mobile_robot_waypoints() -> MobileRobotWaypoints:
    """Return the canonical 12-waypoint mobile-robot dataset."""

    # Hand-designed waypoints. Coordinates chosen so that the robot
    # follows a recognisable aisle layout and includes both sharp turns
    # and a gentle curve.
    waypoints = np.array(
        [
            # x[m], y[m]
            [0.0, 0.0],     # loading dock
            [2.0, 0.0],     # straight entry
            [4.0, 0.0],     # before sharp right turn
            [4.0, -2.0],    # right turn (90 deg)
            [4.0, -4.0],    # short straight after turn
            [6.5, -5.5],    # diagonal traverse to pick station
            [9.0, -4.0],    # arrive at pick station
            [9.0, -1.0],    # 90-deg left turn back upward
            [8.0, 1.0],     # start of gentle S-curve
            [7.0, 2.5],     # mid S-curve
            [5.0, 3.0],     # finishing the S-curve
            [3.0, 3.0],     # exit point
        ]
    )

    # Times are spaced roughly proportional to chord distance between
    # waypoints so a constant nominal speed is implied. This is the
    # "chord-length parameterisation" recommended in the textbook.
    chord = np.linalg.norm(np.diff(waypoints, axis=0), axis=1)
    nominal_speed = 1.0  # m/s
    dt = chord / nominal_speed
    t = np.concatenate([[0.0], np.cumsum(dt)])

    return MobileRobotWaypoints(
        t=t,
        p=waypoints,
        name="mobile_robot",
        description=(
            "Mobile robot warehouse-aisle scenario with two 90-deg "
            "turns, one diagonal traverse, and a gentle S-curve. "
            "Chord-length parameterised at 1 m/s nominal speed."
        ),
    )


def save_mobile_robot_csv(out_path: Path) -> Path:
    """Generate the dataset and write it to CSV.

    The CSV has columns ``t_s,x_m,y_m`` and is suitable for inclusion
    as a deliverable artefact.
    """
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    data = generate_mobile_robot_waypoints()
    df = data.to_dataframe()
    df.to_csv(out_path, index=False, float_format="%.6f")
    return out_path


if __name__ == "__main__":
    out = Path(__file__).resolve().parents[2] / "data" / "mobile_robot_waypoints.csv"
    saved = save_mobile_robot_csv(out)
    print(f"Wrote {saved}")
    print(generate_mobile_robot_waypoints().to_dataframe())
