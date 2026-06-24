"""B-spline approximation via the Cox--de Boor recursion.

Unlike the previous three methods, a B-spline does not in general pass
through its control points. Instead each control point pulls the curve
toward itself with a basis-function weight that is non-zero only over a
limited range of the parameter. This local-control property is the
defining feature of B-splines: moving one control point changes the
curve only in a bounded neighbourhood, which is useful for editable
trajectories and obstacle avoidance.

Numerical procedure
-------------------

For a B-spline of degree ``p`` with knot vector
``T = (t_0, t_1, ..., t_{m})``, the basis functions are defined by the
Cox--de Boor recursion:

    N_{i,0}(t) = 1   if t_i <= t < t_{i+1},   else 0
    N_{i,p}(t) = (t - t_i) / (t_{i+p} - t_i)         * N_{i,p-1}(t)
               + (t_{i+p+1} - t) / (t_{i+p+1} - t_{i+1}) * N_{i+1,p-1}(t)

with the standard convention ``0 / 0 = 0``. The B-spline curve is then

    C(t) = sum_{i=0}^{n-1} N_{i,p}(t) * P_i

where ``P_i`` are the control points.

For trajectory planning we generally want the curve to start at the
first waypoint and end at the last. A clamped (open uniform) knot
vector achieves this by repeating the first and last knots ``p + 1``
times. The interior knots are spaced uniformly in time. With this
choice ``C(t_0) = P_0`` and ``C(t_n) = P_{n-1}`` exactly, but interior
control points are only approximated.

Derivatives are obtained by lowering the degree once per derivative
order, with new control points

    Q_i = p / (t_{i+p+1} - t_{i+1}) * (P_{i+1} - P_i)

(see Piegl & Tiller, "The NURBS Book", Eq. 3.7).
"""

from __future__ import annotations

import numpy as np


class BSplineApproximation:
    """B-spline approximation curve over user waypoints.

    Parameters
    ----------
    t : (n,) array_like
        Parameter values associated with each waypoint. These set the
        time interval ``[t[0], t[-1]]`` over which the curve is
        evaluated and seed the interior knots.
    p : (n,) or (n, d) array_like
        Waypoints used as control points.
    degree : int, default 3
        Degree of the B-spline. Cubic (degree 3) is the standard choice
        and gives C^2 continuity over the parameter range.
    """

    def __init__(self, t, p, degree: int = 3):
        t = np.asarray(t, dtype=float)
        p = np.asarray(p, dtype=float)
        if t.ndim != 1:
            raise ValueError("t must be 1-D")
        if p.shape[0] != t.shape[0]:
            raise ValueError("p first axis must match t")
        if degree < 1:
            raise ValueError("degree must be >= 1")
        if t.size < degree + 1:
            raise ValueError(
                f"need at least degree+1={degree + 1} waypoints, got {t.size}"
            )
        if np.any(np.diff(t) <= 0):
            raise ValueError("t must be strictly increasing")

        self.t = t
        self._scalar_input = p.ndim == 1
        self.P = p[:, None] if self._scalar_input else p
        self.degree = degree
        self.knots = self._make_clamped_knots(t, degree)

    # ------------------------------------------------------------------
    # Knot construction
    # ------------------------------------------------------------------
    @staticmethod
    def _make_clamped_knots(t, p: int) -> np.ndarray:
        """Build a clamped (open uniform) knot vector.

        For ``n`` control points and degree ``p`` we need ``n + p + 1``
        knots. The first and last are repeated ``p + 1`` times so that
        the curve starts at the first waypoint and ends at the last.
        Interior knots are placed at the average of ``p`` consecutive
        parameter values, the standard "averaging" choice that mimics
        the input parameterisation.
        """
        n = t.size
        m = n + p + 1
        knots = np.empty(m)
        knots[: p + 1] = t[0]
        knots[m - p - 1 :] = t[-1]
        # Schoenberg-style averaging for interior knots.
        for j in range(1, n - p):
            knots[j + p] = np.mean(t[j : j + p])
        return knots

    # ------------------------------------------------------------------
    # Basis evaluation via Cox--de Boor recursion
    # ------------------------------------------------------------------
    @staticmethod
    def _basis_matrix(t_query: np.ndarray, knots: np.ndarray, p: int) -> np.ndarray:
        """Return matrix ``B`` of shape (n_query, n_ctrl) where
        ``B[k, i] = N_{i,p}(t_query[k])``.

        Implements the Cox--de Boor recursion directly. For numerical
        stability at the right endpoint we use a closed interval there:
        ``t == t[-1]`` is treated as belonging to the last non-empty
        knot span.
        """
        n_ctrl = knots.size - p - 1
        n_q = t_query.size

        # Allocate a (degree + 1) layer of basis values and recurse up.
        # We compute all N_{i,k} for i = 0..n_ctrl - 1 + (p - k).
        # For simplicity, allocate full (n_ctrl + p) at the bottom and
        # shrink as we recurse upward.

        N_low = np.zeros((n_q, n_ctrl + p))
        # Degree 0: indicator on [t_i, t_{i+1}) (and right-closed at end).
        for i in range(n_ctrl + p):
            left = knots[i]
            right = knots[i + 1]
            if right > left:
                in_span = (t_query >= left) & (t_query < right)
                N_low[in_span, i] = 1.0
        # Right endpoint: tag the rightmost non-degenerate span.
        last_idx = n_ctrl - 1
        N_low[t_query >= knots[-1] - 0.0, last_idx] = np.where(
            t_query[t_query >= knots[-1] - 0.0] >= knots[-1], 1.0,
            N_low[t_query >= knots[-1] - 0.0, last_idx],
        )

        for k in range(1, p + 1):
            n_basis_k = n_ctrl + p - k
            N_high = np.zeros((n_q, n_basis_k))
            for i in range(n_basis_k):
                denom_l = knots[i + k] - knots[i]
                denom_r = knots[i + k + 1] - knots[i + 1]
                term_l = 0.0
                term_r = 0.0
                if denom_l > 0:
                    term_l = (t_query - knots[i]) / denom_l * N_low[:, i]
                if denom_r > 0:
                    term_r = (knots[i + k + 1] - t_query) / denom_r * N_low[:, i + 1]
                N_high[:, i] = term_l + term_r
            N_low = N_high

        return N_low[:, :n_ctrl]

    def __call__(self, t_query):
        t_query = np.atleast_1d(np.asarray(t_query, dtype=float))
        t_clipped = np.clip(t_query, self.t[0], self.t[-1])
        B = self._basis_matrix(t_clipped, self.knots, self.degree)
        out = B @ self.P
        if self._scalar_input:
            return out[:, 0]
        return out

    def derivative(self, t_query, order: int = 1):
        """Analytic derivative via degree-lowering of the control polygon.

        Each derivative order is a B-spline of one lower degree with
        control points ``Q_i = p / (t_{i+p+1} - t_{i+1}) * (P_{i+1} -
        P_i)`` and knot vector formed by dropping the first and last
        knot of the original.
        """
        if order < 0:
            raise ValueError("order must be non-negative")
        if order == 0:
            return self(t_query)
        if order > self.degree:
            t_query = np.atleast_1d(np.asarray(t_query, dtype=float))
            shape = (t_query.size, self.P.shape[1])
            out = np.zeros(shape)
            return out[:, 0] if self._scalar_input else out

        P = self.P.copy()
        knots = self.knots.copy()
        p = self.degree
        for _ in range(order):
            n_ctrl = P.shape[0]
            denom = knots[p + 1 : p + n_ctrl] - knots[1 : n_ctrl]
            # New control points have shape (n_ctrl - 1, d).
            scaling = p / denom
            Q = scaling[:, None] * (P[1:] - P[:-1])
            P = Q
            knots = knots[1:-1]
            p -= 1

        t_query = np.atleast_1d(np.asarray(t_query, dtype=float))
        t_clipped = np.clip(t_query, self.t[0], self.t[-1])
        B = self._basis_matrix(t_clipped, knots, p)
        out = B @ P
        if self._scalar_input:
            return out[:, 0]
        return out
