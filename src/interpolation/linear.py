"""Linear (first-order) interpolation.

Reference: Chapra & Canale, "Numerical Methods for Engineers" (8th ed.),
Section 18.2 (Eq. 18.2):

    f1(x) = f(x0) + (f(x1) - f(x0)) / (x1 - x0) * (x - x0)

For trajectory planning the independent variable is time t and the
dependent variable is a (possibly vector-valued) configuration p.

Continuity:
    - C0: position is continuous at every knot.
    - C1 / higher: velocity is piecewise constant and discontinuous at
      every knot, which corresponds to instantaneous (impulsive)
      acceleration. This makes linear interpolation a useful baseline
      but unsuitable for direct execution on physical hardware.
"""

from __future__ import annotations

import numpy as np


class LinearInterpolator:
    """Piecewise-linear interpolator for vector-valued waypoints.

    Parameters
    ----------
    t : (n,) array_like
        Knot times, strictly increasing.
    p : (n,) or (n, d) array_like
        Knot values. If 2-D, each column is interpolated independently.

    Raises
    ------
    ValueError
        If `t` is not strictly increasing or shapes are inconsistent.
    """

    def __init__(self, t, p):
        t = np.asarray(t, dtype=float)
        p = np.asarray(p, dtype=float)
        if t.ndim != 1:
            raise ValueError("t must be 1-D")
        if p.shape[0] != t.shape[0]:
            raise ValueError(
                f"p has {p.shape[0]} rows but t has {t.shape[0]} entries"
            )
        if np.any(np.diff(t) <= 0):
            raise ValueError("t must be strictly increasing")
        if t.size < 2:
            raise ValueError("at least two knots are required")
        self.t = t
        self.p = p
        self._scalar_input = p.ndim == 1

    def __call__(self, t_query):
        """Evaluate the interpolant at one or more query times.

        Query times outside [t[0], t[-1]] are clamped to the nearest knot
        (no extrapolation). This matches typical trajectory-planning
        behaviour where the robot holds the start/end pose.
        """
        t_query = np.atleast_1d(np.asarray(t_query, dtype=float))
        # Clamp to [t0, tN] to avoid extrapolation artefacts.
        t_clipped = np.clip(t_query, self.t[0], self.t[-1])

        # searchsorted gives the first index i with t[i] >= q. We want the
        # left knot of the segment containing q, so subtract 1 and clamp.
        idx = np.searchsorted(self.t, t_clipped, side="right") - 1
        idx = np.clip(idx, 0, self.t.size - 2)

        t0 = self.t[idx]
        t1 = self.t[idx + 1]
        # Eq. 18.2 written as a normalised parameter alpha in [0, 1].
        alpha = (t_clipped - t0) / (t1 - t0)

        p0 = self.p[idx]
        p1 = self.p[idx + 1]
        if self._scalar_input:
            return p0 + alpha * (p1 - p0)
        return p0 + alpha[:, None] * (p1 - p0)

    def derivative(self, t_query, order=1):
        """Analytic derivative of the piecewise-linear interpolant.

        Returns a piecewise-constant first derivative for `order=1` and
        zeros for higher orders. The derivative is undefined at knot
        locations; we return the right-segment value there.
        """
        if order == 0:
            return self(t_query)
        t_query = np.atleast_1d(np.asarray(t_query, dtype=float))
        if order >= 2:
            shape = (t_query.size,) if self._scalar_input else (
                t_query.size,
                self.p.shape[1],
            )
            return np.zeros(shape)

        idx = np.searchsorted(self.t, t_query, side="right") - 1
        idx = np.clip(idx, 0, self.t.size - 2)
        dt = self.t[idx + 1] - self.t[idx]
        dp = self.p[idx + 1] - self.p[idx]
        if self._scalar_input:
            return dp / dt
        return dp / dt[:, None]
