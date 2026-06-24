"""Interpolation methods package.

Each interpolator follows a common interface:

    interp = Method(t_knots, p_knots)
    p = interp(t_query)      # position at query times
    v = interp.derivative(t_query, order=1)  # optional analytic derivative

Vector-valued positions are supported via shape (n_knots, dim).
"""

from .linear import LinearInterpolator
from .cubic_spline import CubicSpline
from .quintic_spline import QuinticSpline
from .bspline import BSplineApproximation

__all__ = [
    "LinearInterpolator",
    "CubicSpline",
    "QuinticSpline",
    "BSplineApproximation",
]
