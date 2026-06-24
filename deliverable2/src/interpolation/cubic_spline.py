"""Cubic spline interpolation (Chapra & Canale, Section 18.6).

This module derives cubic splines from the textbook second-derivative
formulation rather than calling ``scipy.interpolate.CubicSpline`` so the
numerical procedure is explicit and inspectable.

The textbook formulation
------------------------

For ``n + 1`` knots ``(t_0, p_0), ..., (t_n, p_n)`` we seek ``n`` cubic
polynomials ``S_i(t)`` on segment ``[t_i, t_{i+1}]`` that

* interpolate the data: ``S_i(t_i) = p_i`` and ``S_i(t_{i+1}) = p_{i+1}``;
* are C^2 across interior knots: ``S_{i-1}'`` and ``S_{i-1}''`` agree with
  ``S_i'`` and ``S_i''`` at every interior ``t_i``.

Following Chapra & Canale we let ``M_i = S''(t_i)`` be the unknowns and
write each segment in the closed form (Chapra eq. 18.36):

    S_i(t) = M_i / (6 h_i) (t_{i+1} - t)^3
           + M_{i+1} / (6 h_i) (t - t_i)^3
           + (p_i / h_i - M_i h_i / 6) (t_{i+1} - t)
           + (p_{i+1} / h_i - M_{i+1} h_i / 6) (t - t_i)

where ``h_i = t_{i+1} - t_i``. Imposing C^1 continuity at the interior
knots gives the tridiagonal system (Chapra eq. 18.37):

    h_{i-1} M_{i-1} + 2 (h_{i-1} + h_i) M_i + h_i M_{i+1}
        = 6/h_i (p_{i+1} - p_i) - 6/h_{i-1} (p_i - p_{i-1})

for ``i = 1, ..., n - 1``. The two missing equations come from the
boundary conditions; we implement the standard "natural" spline
(``M_0 = M_n = 0``) and "clamped" spline (specified end velocities), and
default to natural to match the textbook example.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

import numpy as np
from scipy.linalg import solve_banded


BoundaryType = Literal["natural", "clamped"]


@dataclass
class _SegmentLookup:
    """Cached results of locating query times within knot intervals."""

    idx: np.ndarray   # left-knot index for each query
    h: np.ndarray     # segment width h_i for each query
    a: np.ndarray     # (t_{i+1} - t)
    b: np.ndarray     # (t - t_i)


class CubicSpline:
    """Natural / clamped cubic spline implemented from textbook eq. 18.36.

    Parameters
    ----------
    t : (n,) array_like
        Knot times, strictly increasing. ``n >= 3`` is required for a
        non-trivial natural spline; ``n >= 2`` is allowed for clamped.
    p : (n,) or (n, d) array_like
        Knot values. Each column of a 2-D ``p`` is treated as an
        independent scalar function sharing the same ``t`` knots.
    bc : {"natural", "clamped"}, default "natural"
        Boundary condition. "natural" sets ``M_0 = M_n = 0``. "clamped"
        sets the first derivative at each end (``v0``, ``vN``).
    v0, vN : float or (d,) array_like, optional
        End-point first derivatives required when ``bc == "clamped"``.
    """

    def __init__(
        self,
        t,
        p,
        bc: BoundaryType = "natural",
        v0=None,
        vN=None,
    ):
        t = np.asarray(t, dtype=float)
        p = np.asarray(p, dtype=float)
        if t.ndim != 1:
            raise ValueError("t must be 1-D")
        if p.shape[0] != t.shape[0]:
            raise ValueError("p first axis must match length of t")
        if np.any(np.diff(t) <= 0):
            raise ValueError("t must be strictly increasing")
        if bc not in ("natural", "clamped"):
            raise ValueError("bc must be 'natural' or 'clamped'")

        self.t = t
        self._scalar_input = p.ndim == 1
        # Promote scalar to (n, 1) for uniform handling, demote on output.
        self.p = p[:, None] if self._scalar_input else p
        self.bc = bc

        n = t.size
        self.n_knots = n
        self.h = np.diff(t)  # (n - 1,)

        # Solve the tridiagonal system for the second-derivative vector
        # M of shape (n, d). We solve all dimensions simultaneously with
        # a matrix right-hand side.
        d = self.p.shape[1]
        self.M = self._solve_second_derivatives(
            self.h, self.p, bc, v0, vN, d
        )

    # ------------------------------------------------------------------
    # Construction
    # ------------------------------------------------------------------
    @staticmethod
    def _solve_second_derivatives(h, p, bc, v0, vN, d):
        n = p.shape[0]
        # Build the tridiagonal system A @ M = r. We use the banded form
        # (3, n) expected by scipy.linalg.solve_banded with l=u=1.
        ab = np.zeros((3, n))
        rhs = np.zeros((n, d))

        # Interior rows i = 1 .. n - 2 use Chapra eq. 18.37.
        i = np.arange(1, n - 1)
        # Upper diagonal (offset +1) sits in row 0 columns shifted by +1.
        ab[0, i + 1] = h[i]
        # Main diagonal at row 1.
        ab[1, i] = 2.0 * (h[i - 1] + h[i])
        # Lower diagonal (offset -1) at row 2 columns shifted by -1.
        ab[2, i - 1] = h[i - 1]
        rhs[i] = (
            6.0 / h[i, None] * (p[i + 1] - p[i])
            - 6.0 / h[i - 1, None] * (p[i] - p[i - 1])
        )

        # Boundary rows.
        if bc == "natural":
            # M_0 = 0 and M_{n-1} = 0.
            ab[1, 0] = 1.0
            ab[1, -1] = 1.0
            # rhs already zero on these rows.
        else:  # clamped
            if v0 is None or vN is None:
                raise ValueError("clamped spline requires v0 and vN")
            v0 = np.broadcast_to(np.asarray(v0, dtype=float), (d,))
            vN = np.broadcast_to(np.asarray(vN, dtype=float), (d,))
            # Derived from S'(t_0) = v0 and S'(t_{n-1}) = vN. Substituting
            # eq. 18.36 derivatives at the endpoints yields:
            #   2 h_0 M_0 + h_0 M_1 = 6/h_0 (p_1 - p_0) - 6 v0
            #   h_{n-2} M_{n-2} + 2 h_{n-2} M_{n-1}
            #       = -6/h_{n-2} (p_{n-1} - p_{n-2}) + 6 vN
            ab[1, 0] = 2.0 * h[0]
            ab[0, 1] = h[0]
            rhs[0] = 6.0 / h[0] * (p[1] - p[0]) - 6.0 * v0
            ab[2, -2] = h[-1]
            ab[1, -1] = 2.0 * h[-1]
            rhs[-1] = -6.0 / h[-1] * (p[-1] - p[-2]) + 6.0 * vN

        return solve_banded((1, 1), ab, rhs)

    # ------------------------------------------------------------------
    # Evaluation
    # ------------------------------------------------------------------
    def _locate(self, t_query) -> _SegmentLookup:
        t_query = np.atleast_1d(np.asarray(t_query, dtype=float))
        t_clipped = np.clip(t_query, self.t[0], self.t[-1])
        idx = np.searchsorted(self.t, t_clipped, side="right") - 1
        idx = np.clip(idx, 0, self.t.size - 2)
        h_i = self.h[idx]
        a = self.t[idx + 1] - t_clipped
        b = t_clipped - self.t[idx]
        return _SegmentLookup(idx=idx, h=h_i, a=a, b=b)

    def __call__(self, t_query):
        s = self._locate(t_query)
        Mi = self.M[s.idx]
        Mi1 = self.M[s.idx + 1]
        pi = self.p[s.idx]
        pi1 = self.p[s.idx + 1]

        h = s.h[:, None]
        a = s.a[:, None]
        b = s.b[:, None]
        # Eq. 18.36 evaluated point-wise.
        out = (
            Mi * a**3 / (6.0 * h)
            + Mi1 * b**3 / (6.0 * h)
            + (pi / h - Mi * h / 6.0) * a
            + (pi1 / h - Mi1 * h / 6.0) * b
        )
        if self._scalar_input:
            return out[:, 0]
        return out

    def derivative(self, t_query, order=1):
        """Analytic derivatives of the cubic spline."""
        if order == 0:
            return self(t_query)
        if order > 3:
            t_query = np.atleast_1d(np.asarray(t_query, dtype=float))
            shape = (t_query.size, self.p.shape[1])
            out = np.zeros(shape)
            return out[:, 0] if self._scalar_input else out

        s = self._locate(t_query)
        Mi = self.M[s.idx]
        Mi1 = self.M[s.idx + 1]
        pi = self.p[s.idx]
        pi1 = self.p[s.idx + 1]
        h = s.h[:, None]
        a = s.a[:, None]
        b = s.b[:, None]

        if order == 1:
            # d/dt of eq. 18.36; note d/dt(a) = -1 and d/dt(b) = +1.
            out = (
                -Mi * a**2 / (2.0 * h)
                + Mi1 * b**2 / (2.0 * h)
                - (pi / h - Mi * h / 6.0)
                + (pi1 / h - Mi1 * h / 6.0)
            )
        elif order == 2:
            out = Mi * a / h + Mi1 * b / h
        else:  # order == 3
            # 3rd derivative is constant within each segment.
            out = (Mi1 - Mi) / h
            # Broadcast to (n_query, d).
            out = np.broadcast_to(out, (s.idx.size, self.p.shape[1])).copy()

        if self._scalar_input:
            return out[:, 0]
        return out
