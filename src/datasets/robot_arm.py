"""2-DOF planar robot arm joint-angle waypoint dataset.

Scenario
--------

A 2-DOF planar arm with link lengths ``L1 = 0.5 m`` and ``L2 = 0.4 m``
performs a pick-and-place cycle:

    home -> approach -> grasp -> lift -> transfer -> place -> retract -> home

Joint angles are specified in degrees for readability and converted to
radians internally. The forward kinematics of the arm is

    x = L1 cos(theta1) + L2 cos(theta1 + theta2)
    y = L1 sin(theta1) + L2 sin(theta1 + theta2)

so that the dataset can be evaluated either in joint space (where the
interpolation methods operate) or in Cartesian space (which is what the
robot's end effector actually traces). The chosen waypoints respect a
reachable workspace with shoulder angles between -30 and +120 degrees
and elbow angles between -150 and +20 degrees.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd


L1_DEFAULT = 0.5  # m
L2_DEFAULT = 0.4  # m


def forward_kinematics(theta1: np.ndarray, theta2: np.ndarray, L1: float = L1_DEFAULT, L2: float = L2_DEFAULT) -> np.ndarray:
    """Forward kinematics for the 2-DOF planar arm.

    Parameters
    ----------
    theta1, theta2 : array_like, radians
    L1, L2 : float, link lengths in metres

    Returns
    -------
    p : (..., 2) ndarray with end-effector ``(x, y)`` coordinates.
    """
    theta1 = np.asarray(theta1, dtype=float)
    theta2 = np.asarray(theta2, dtype=float)
    x = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
    y = L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2)
    return np.stack([x, y], axis=-1)


@dataclass(frozen=True)
class RobotArmWaypoints:
    t: np.ndarray            # (n,) seconds
    theta_deg: np.ndarray    # (n, 2) joint angles in degrees
    L1: float
    L2: float
    name: str
    description: str

    @property
    def theta_rad(self) -> np.ndarray:
        return np.deg2rad(self.theta_deg)

    @property
    def cartesian(self) -> np.ndarray:
        return forward_kinematics(
            self.theta_rad[:, 0], self.theta_rad[:, 1], self.L1, self.L2
        )

    def to_dataframe(self) -> pd.DataFrame:
        cart = self.cartesian
        return pd.DataFrame(
            {
                "t_s": self.t,
                "theta1_deg": self.theta_deg[:, 0],
                "theta2_deg": self.theta_deg[:, 1],
                "x_m": cart[:, 0],
                "y_m": cart[:, 1],
            }
        )


def generate_robot_arm_waypoints() -> RobotArmWaypoints:
    """Canonical 8-waypoint pick-and-place dataset for the 2-DOF arm."""

    # Joint angles in degrees, chosen to visit:
    #  - home                (theta1=  0, theta2=  0): arm fully extended along +x
    #  - approach over part  (theta1= 30, theta2=-60): hover above pickup
    #  - grasp               (theta1= 30, theta2=-90): tip touches part
    #  - lift                (theta1= 30, theta2=-60): back to hover
    #  - transfer mid-point  (theta1= 75, theta2=-90): swing toward placement bay
    #  - over placement bay  (theta1=110, theta2=-60): hover above destination
    #  - place               (theta1=110, theta2=-90): lower for release
    #  - retract / home      (theta1= 50, theta2=  0)
    theta_deg = np.array(
        [
            [0.0, 0.0],
            [30.0, -60.0],
            [30.0, -90.0],
            [30.0, -60.0],
            [75.0, -90.0],
            [110.0, -60.0],
            [110.0, -90.0],
            [50.0, 0.0],
        ]
    )

    # Equal time spacing of 1 s per waypoint is intentional: it keeps the
    # comparison of methods uncluttered by non-uniformity. The mobile
    # robot dataset already exercises non-uniform spacing.
    t = np.arange(theta_deg.shape[0], dtype=float)

    return RobotArmWaypoints(
        t=t,
        theta_deg=theta_deg,
        L1=L1_DEFAULT,
        L2=L2_DEFAULT,
        name="robot_arm",
        description=(
            "2-DOF planar arm pick-and-place: 8 joint-space waypoints "
            "covering home, approach, grasp, lift, transfer, place, "
            "retract, home. Link lengths L1=0.5m, L2=0.4m."
        ),
    )


def save_robot_arm_csv(out_path: Path) -> Path:
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    data = generate_robot_arm_waypoints()
    df = data.to_dataframe()
    df.to_csv(out_path, index=False, float_format="%.6f")
    return out_path


if __name__ == "__main__":
    out = Path(__file__).resolve().parents[2] / "data" / "robot_arm_waypoints.csv"
    saved = save_robot_arm_csv(out)
    print(f"Wrote {saved}")
    print(generate_robot_arm_waypoints().to_dataframe())
