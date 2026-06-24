"""Quintic spline interpolation with C^4 continuity.

Where a cubic spline matches position, velocity, and acceleration at
each knot, a quintic spline goes further by also matching the third
derivative (jerk) and the fourth derivative (snap). For trajectory
planning this means jerk -- the quantity directly responsible for
mechanical vibration and payload disturbance -- is continuous, not just
piecewise constant as in cubic splines.

Formulation
-----------

We build each segment as a quintic Hermite polynomial parametrised by
the unit-interval variable ``s = (t - t_i) / h_i`` with ``h_i = t_{i+1}
- t_i``:

    p_i(s) = H0(s) p_i + H1(s) h_i v_i + H2(s) h_i^2 a_i
           + H3(s) p_{i+1} + H4(s) h_i v_{i+1} + H5(s) h_i^2 a_{i+1}

where ``v_i`` and ``a_i`` are the (unknown) first and second time
derivatives of the trajectory at knot ``i`` and the basis functions are

    H0(s) = 1 - 10 s^3 + 15 s^4 -  6 s^5
    H1(s) = s -  6 s^3 +  8 s^4 -  3 s^5
    H2(s) = s^2/2 - 3 s^3/2 + 3 s^4/2 - s^5/2
    H3(s) = 10 s^3 - 15 s^4 +  6 s^5
    H4(s) =       -  4 s^3 +  7 s^4 -  3 s^5
    H5(s) = s^3/2 - s^4 + s^5/2

These are the standard quintic Hermite basis functions: by construction
``p_i(0) = p_i``, ``p_i'(0) = v_i``, ``p_i''(0) = a_i`` and the
analogous identities at ``s = 1``.

The segments share position, velocity, and acceleration at every
internal knot by construction. To make the spline a true C^4 quintic,
we require continuity of the third and fourth time derivatives at each
internal knot, which gives ``2(n - 1)`` linear equations in the
``2(n + 1)`` unknowns ``{v_i, a_i}``. The remaining four degrees of
freedom are absorbed by user-specified boundary conditions: clamped
(both ``v`` and ``a`` set at each end) or natural (set everything to
zero, which corresponds to starting and ending at rest).
"""

from __future__ import annotations

from typing import Literal

import numpy as np
from scipy.linalg import solve as solve_dense

# Quintic Hermite basis coefficient tables, computed once.
# Row k of _BASIS gives the polynomial coefficients of H_k in ascending
# order [c0, c1, c2, c3, c4, c5] so that H_k(s) = sum c_j * s^j.
_BASIS = np.array(
    [
        [1.0, 0.0, 0.0, -10.0, 15.0, -6.0],   # H0
        [0.0, 1.0, 0.0, -6.0, 8.0, -3.0],     # H1
        [0.0, 0.0, 0.5, -1.5, 1.5, -0.5],     # H2
        [0.0, 0.0, 0.0, 10.0, -15.0, 6.0],    # H3
        [0.0, 0.0, 0.0, -4.0, 7.0, -3.0],     # H4
        [0.0, 0.0, 0.0, 0.5, -1.0, 0.5],      # H5
    ]
)


def _diff_coeffs(coeffs, k):
    """Return polynomial coefficients of the kth derivative."""
    c = coeffs.copy()
    for _ in range(k):
        n = c.shape[-1]
        if n == 0:
            return np.zeros_like(c[..., :1])
        powers = np.arange(1, n)
        c = c[..., 1:] * powers
    return c


def _eval_poly(coeffs, s):
    """Horner evaluation of polynomial(s) with coefficients in ascending order."""
    s = np.asarray(s, dtype=float)
    out = np.zeros(np.broadcast_shapes(coeffs.shape[:-1], s.shape))
    for c in np.moveaxis(coeffs, -1, 0)[::-1]:
        out = out * s + c
    return out


# Pre-compute basis derivatives at s = 0 and s = 1 for setting up the
# continuity equations. _BASIS_DERIV[k] is a (6, ?) array where row j is
# the kth derivative of H_j evaluated at the single-point coefficient
# polynomial.
_DERIV_COEFFS = [_diff_coeffs(_BASIS, k) for k in range(5)]


def _basis_deriv(order: int, s: float) -> np.ndarray:
    """All six basis derivatives of given order at parameter s."""
    coeffs = _DERIV_COEFFS[order]
    return _eval_poly(coeffs, np.asarray(s, dtype=float))


BoundaryType = Literal["natural", "clamped"]


class QuinticSpline:
    """C^4 quintic spline implemented from the Hermite formulation.

    Parameters
    ----------
    t : (n,) array_like
        Knot times, strictly increasing.
    p : (n,) or (n, d) array_like
        Knot values; vector-valued ``p`` is treated dimension-by-dimension.
    bc : {"natural", "clamped"}, default "natural"
        Boundary condition. "natural" sets ``v`` and ``a`` to zero at both
        endpoints (start and stop at rest). "clamped" lets the caller
        specify ``v0``, ``a0``, ``vN``, ``aN``.
    v0, vN, a0, aN : float or (d,), optional
        End-point first and second derivatives for "clamped" boundary.
    """

    def __init__(
        self,
        t,
        p,
        bc: BoundaryType = "natural",
        v0=None,
        vN=None,
        a0=None,
        aN=None,
    ):
        t = np.asarray(t, dtype=float)
        p = np.asarray(p, dtype=float)
        if t.ndim != 1:
            raise ValueError("t must be 1-D")
        if p.shape[0] != t.shape[0]:
            raise ValueError("p first axis must match t")
        if np.any(np.diff(t) <= 0):
            raise ValueError("t must be strictly increasing")
        if bc not in ("natural", "clamped"):
            raise ValueError("bc must be 'natural' or 'clamped'")

        self.t = t
        self._scalar_input = p.ndim == 1
        self.p = p[:, None] if self._scalar_input else p

        n = t.size
        d = self.p.shape[1]
        h = np.diff(t)
        self.h = h
        self.n_knots = n

        # Resolve boundary values into broadcasted (d,) arrays.
        def _coerce(name, value):
            if value is None:
                return np.zeros(d)
            arr = np.atleast_1d(np.asarray(value, dtype=float))
            return np.broadcast_to(arr, (d,)).copy()

        if bc == "natural":
            v0 = a0 = vN = aN = np.zeros(d)
        else:
            if any(x is None for x in (v0, a0, vN, aN)):
                raise ValueError("clamped quintic spline needs v0, a0, vN, aN")
            v0 = _coerce("v0", v0)
            a0 = _coerce("a0", a0)
            vN = _coerce("vN", vN)
            aN = _coerce("aN", aN)

        # Assemble and solve for v_i and a_i at every knot.
        self.v, self.a = self._solve_derivatives(h, self.p, v0, a0, vN, aN)

    # ------------------------------------------------------------------
    # System assembly
    # ------------------------------------------------------------------
    @staticmethod
    def _solve_derivatives(h, p, v0, a0, vN, aN):
        n = p.shape[0]
        d = p.shape[1]
        # Unknowns ordered as [v_0, a_0, v_1, a_1, ..., v_{n-1}, a_{n-1}].
        N = 2 * n
        A = np.zeros((N, N))
        rhs = np.zeros((N, d))

        # Pre-evaluate basis derivatives at endpoints.
        d3_at_1 = _basis_deriv(3, 1.0)  # shape (6,)
        d3_at_0 = _basis_deriv(3, 0.0)
        d4_at_1 = _basis_deriv(4, 1.0)
        d4_at_0 = _basis_deriv(4, 0.0)

        # Continuity equations at internal knots i = 1, ..., n - 2.
        #
        # The condition p^(k)(t_i^-) = p^(k)(t_i^+) is rearranged as
        #
        #     p^(k)(t_i^-) - p^(k)(t_i^+) = 0.
        #
        # Each side is evaluated through the Hermite basis:
        #
        #     p^(k)(t_i^-) = (1 / hL^k) * sum_j H_j^(k)(1) * w_left[j]
        #     p^(k)(t_i^+) = (1 / hR^k) * sum_j H_j^(k)(0) * w_right[j]
        #
        # where the per-knot weights are
        #     w_left  = (p_{i-1}, hL v_{i-1}, hL^2 a_{i-1}, p_i, hL v_i, hL^2 a_i)
        #     w_right = (p_i,     hR v_i,     hR^2 a_i,     p_{i+1}, hR v_{i+1}, hR^2 a_{i+1}).
        #
        # The (1/h^k) chain-rule factors combine with the h, h^2 scalings
        # to give variable coefficients of the form H_j^(k)(.) / h^q
        # where q in {k, k-1, k-2}.
        for i in range(1, n - 1):
            hL = h[i - 1]
            hR = h[i]
            cols = np.array(
                [
                    2 * (i - 1) + 0,  # v_{i-1}
                    2 * (i - 1) + 1,  # a_{i-1}
                    2 * i + 0,        # v_i
                    2 * i + 1,        # a_i
                    2 * (i + 1) + 0,  # v_{i+1}
                    2 * (i + 1) + 1,  # a_{i+1}
                ]
            )

            row3 = 2 * (i - 1)
            # Variable coefficients: left segment contributes with +sign,
            # right segment contributes with -sign because we subtracted.
            A[row3, cols[0]] = d3_at_1[1] / hL**2
            A[row3, cols[1]] = d3_at_1[2] / hL
            A[row3, cols[2]] = d3_at_1[4] / hL**2 - d3_at_0[1] / hR**2
            A[row3, cols[3]] = d3_at_1[5] / hL - d3_at_0[2] / hR
            A[row3, cols[4]] = -d3_at_0[4] / hR**2
            A[row3, cols[5]] = -d3_at_0[5] / hR
            # The p-terms are known data; move them to the RHS by negating.
            rhs[row3] = -(
                d3_at_1[0] * p[i - 1] / hL**3
                + d3_at_1[3] * p[i] / hL**3
                - d3_at_0[0] * p[i] / hR**3
                - d3_at_0[3] * p[i + 1] / hR**3
            )

            row4 = 2 * (i - 1) + 1
            A[row4, cols[0]] = d4_at_1[1] / hL**3
            A[row4, cols[1]] = d4_at_1[2] / hL**2
            A[row4, cols[2]] = d4_at_1[4] / hL**3 - d4_at_0[1] / hR**3
            A[row4, cols[3]] = d4_at_1[5] / hL**2 - d4_at_0[2] / hR**2
            A[row4, cols[4]] = -d4_at_0[4] / hR**3
            A[row4, cols[5]] = -d4_at_0[5] / hR**2
            rhs[row4] = -(
                d4_at_1[0] * p[i - 1] / hL**4
                + d4_at_1[3] * p[i] / hL**4
                - d4_at_0[0] * p[i] / hR**4
                - d4_at_0[3] * p[i + 1] / hR**4
            )

        # Boundary rows: place at the bottom of the system.
        # v_0 = v0, a_0 = a0, v_{n-1} = vN, a_{n-1} = aN.
        bnd_start = 2 * (n - 2) if n > 2 else 0
        A[bnd_start + 0, 0] = 1.0
        rhs[bnd_start + 0] = v0
        A[bnd_start + 1, 1] = 1.0
        rhs[bnd_start + 1] = a0
        A[bnd_start + 2, 2 * (n - 1) + 0] = 1.0
        rhs[bnd_start + 2] = vN
        A[bnd_start + 3, 2 * (n - 1) + 1] = 1.0
        rhs[bnd_start + 3] = aN

        x = solve_dense(A, rhs)
        # Reshape to (n, d) for v and a.
        v = x[0::2]
        a = x[1::2]
        return v, a

    # ------------------------------------------------------------------
    # Evaluation
    # ------------------------------------------------------------------
    def _locate(self, t_query):
        t_query = np.atleast_1d(np.asarray(t_query, dtype=float))
        t_clipped = np.clip(t_query, self.t[0], self.t[-1])
        idx = np.searchsorted(self.t, t_clipped, side="right") - 1
        idx = np.clip(idx, 0, self.t.size - 2)
        h_i = self.h[idx]
        s = (t_clipped - self.t[idx]) / h_i
        return idx, h_i, s

    def _eval_with_basis(self, t_query, deriv_order):
        idx, h_i, s = self._locate(t_query)
        # Coefficients of the requested derivative (with respect to s).
        coeffs = _DERIV_COEFFS[deriv_order]
        # Evaluate all 6 basis derivatives at every query s.
        # B has shape (6, n_query).
        B = np.array([_eval_poly(coeffs[k], s) for k in range(6)])
        # Variable contributions per knot:
        # H0 -> p_i (no scaling)
        # H1 -> h * v_i
        # H2 -> h^2 * a_i
        # H3 -> p_{i+1}
        # H4 -> h * v_{i+1}
        # H5 -> h^2 * a_{i+1}
        # The chain rule introduces a factor (1/h)^deriv_order when going
        # from d/ds to d/dt.
        h = h_i
        scale = 1.0 / (h ** deriv_order) if deriv_order > 0 else np.ones_like(h)
        h2 = h * h
        out = (
            B[0][:, None] * self.p[idx]
            + B[1][:, None] * (h[:, None] * self.v[idx])
            + B[2][:, None] * (h2[:, None] * self.a[idx])
            + B[3][:, None] * self.p[idx + 1]
            + B[4][:, None] * (h[:, None] * self.v[idx + 1])
            + B[5][:, None] * (h2[:, None] * self.a[idx + 1])
        )
        out = out * scale[:, None]
        if self._scalar_input:
            return out[:, 0]
        return out

    def __call__(self, t_query):
        return self._eval_with_basis(t_query, deriv_order=0)

    def derivative(self, t_query, order=1):
        if order < 0:
            raise ValueError("order must be non-negative")
        if order == 0:
            return self(t_query)
        if order > 5:
            t_query = np.atleast_1d(np.asarray(t_query, dtype=float))
            shape = (t_query.size, self.p.shape[1])
            out = np.zeros(shape)
            return out[:, 0] if self._scalar_input else out
        return self._eval_with_basis(t_query, deriv_order=order)
