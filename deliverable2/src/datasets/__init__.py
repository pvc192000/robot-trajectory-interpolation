"""Synthetic waypoint datasets used in the comparative study.

Both scenarios are generated programmatically so that any reader can
reproduce them exactly. They are chosen to expose the strengths and
weaknesses of each interpolation method:

* :mod:`mobile_robot` -- a 2-D ``(x, y)`` waypoint sequence representing
  a mobile robot navigating an aisle with sharp 90-degree corners and
  one gentle curve. The sharp corners stress every interpolation
  method's ability to handle abrupt direction changes.

* :mod:`robot_arm` -- a 6-waypoint joint-angle sequence for a 2-DOF
  planar arm executing a pick-and-place cycle. The arm waypoints are
  designed so that the resulting Cartesian end-effector path is
  non-trivial even though each joint trajectory looks simple.
"""

from .mobile_robot import generate_mobile_robot_waypoints
from .robot_arm import generate_robot_arm_waypoints, forward_kinematics
from .variants import (
    WaypointDataset,
    all_variants,
    make_arm_variant,
    make_gradual_variant,
    make_mixed_variant,
    make_sharp_variant,
)
from .path_generators import (
    gradual_sinusoid,
    mixed_aisle,
    sharp_zigzag,
    time_stamps,
)

__all__ = [
    "generate_mobile_robot_waypoints",
    "generate_robot_arm_waypoints",
    "forward_kinematics",
    "WaypointDataset",
    "all_variants",
    "make_arm_variant",
    "make_gradual_variant",
    "make_mixed_variant",
    "make_sharp_variant",
    "gradual_sinusoid",
    "mixed_aisle",
    "sharp_zigzag",
    "time_stamps",
]
