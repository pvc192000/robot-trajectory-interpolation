"""Numerical differentiation formulas from Chapra & Canale, Chapter 28.

This module implements the finite-difference formulas listed in Tables
28.1--28.4 of Chapra & Canale, "Numerical Methods for Engineers"
(8th ed.). They are used in this project to compute velocity,
acceleration, and jerk profiles from sampled position data, providing
an *empirical* check against the analytic derivatives produced by each
interpolation method.

The supported schemes are:

* Forward difference, O(h)   -- Eq. 28.1
* Backward difference, O(h)  -- Eq. 28.2
* Centered difference, O(h^2) -- Eq. 28.3
* Higher-order forward / backward / centered formulas -- Tables 28.1--28.3

Each function takes a uniformly-spaced sample of a scalar or vector
function and returns the requested derivative on the same grid. End
points use one-sided formulas so the output has the same length as the
input, mirroring the textbook recommendation of swapping to forward or
backward differences near the boundary.
"""

from __future__ import annotations

from typing import Literal

import numpy as np

Scheme = Literal["forward", "backward", "centered", "centered4", "auto"]


def _ensure_uniform(t: np.ndarray) -> float:
    dt = np.diff(t)
    if not np.allclose(dt, dt[0], rtol=1e-9, atol=1e-12):
        raise ValueError("differentiate() expects uniformly spaced samples")
    return float(dt[0])


def differentiate(
    t: np.ndarray,
    f: np.ndarray,
    order: int = 1,
    scheme: Scheme = "auto",
) -> np.ndarray:
    """Numerically differentiate samples ``f`` taken at times ``t``.

    Parameters
    ----------
    t : (N,) array_like
        Uniformly spaced sample times.
    f : (N,) or (N, d) array_like
        Function samples. A 2-D ``f`` is differentiated column-wise.
    order : {1, 2, 3}
        Derivative order: 1 = velocity, 2 = acceleration, 3 = jerk.
    scheme : {"forward", "backward", "centered", "centered4", "auto"}
        Finite-difference scheme. ``"auto"`` picks centered O(h^2) for
        first/second derivatives and the second-order accurate centered
        formula for third derivative, matching Chapra & Canale's
        recommendation that centered formulas are preferred when both
        sides of the evaluation point are available.

    Returns
    -------
    df : ndarray
        Derivative samples on the same grid as ``f``.

    Notes
    -----
    Endpoint values use lower-order one-sided formulas. This is the
    standard approach (Chapra & Canale Sec. 28.2) and produces an array
    of the same length as the input, which is convenient when comparing
    against analytic derivatives sampled on the same grid.
    """
    t = np.asarray(t, dtype=float)
    f = np.asarray(f, dtype=float)
    h = _ensure_uniform(t)
    if order not in (1, 2, 3):
        raise ValueError("order must be 1, 2, or 3")

    if scheme == "auto":
        scheme = "centered4" if order == 1 else "centered"

    if order == 1:
        return _first_derivative(f, h, scheme)
    if order == 2:
        return _second_derivative(f, h, scheme)
    return _third_derivative(f, h)


# ----------------------------------------------------------------------
# First derivative
# ----------------------------------------------------------------------
def _first_derivative(f: np.ndarray, h: float, scheme: str) -> np.ndarray:
    n = f.shape[0]
    df = np.zeros_like(f)

    if scheme == "forward":
        # f'(x_i) = (f_{i+1} - f_i) / h, eq. 28.1.
        df[:-1] = (f[1:] - f[:-1]) / h
        df[-1] = (f[-1] - f[-2]) / h
    elif scheme == "backward":
        # f'(x_i) = (f_i - f_{i-1}) / h, eq. 28.2.
        df[1:] = (f[1:] - f[:-1]) / h
        df[0] = (f[1] - f[0]) / h
    elif scheme == "centered":
        # f'(x_i) = (f_{i+1} - f_{i-1}) / (2h), eq. 28.3.
        df[1:-1] = (f[2:] - f[:-2]) / (2 * h)
        df[0] = (-3 * f[0] + 4 * f[1] - f[2]) / (2 * h)   # forward 3-pt
        df[-1] = (3 * f[-1] - 4 * f[-2] + f[-3]) / (2 * h)  # backward 3-pt
    elif scheme == "centered4":
        # 5-point centered, O(h^4): table 28.3 row 1.
        if n < 5:
            return _first_derivative(f, h, "centered")
        df[2:-2] = (-f[4:] + 8 * f[3:-1] - 8 * f[1:-3] + f[:-4]) / (12 * h)
        # Near the boundary fall back to the O(h^2) centered or one-sided.
        df[1] = (f[2] - f[0]) / (2 * h)
        df[-2] = (f[-1] - f[-3]) / (2 * h)
        df[0] = (-3 * f[0] + 4 * f[1] - f[2]) / (2 * h)
        df[-1] = (3 * f[-1] - 4 * f[-2] + f[-3]) / (2 * h)
    else:
        raise ValueError(f"unknown scheme {scheme!r}")
    return df


# ----------------------------------------------------------------------
# Second derivative
# ----------------------------------------------------------------------
def _second_derivative(f: np.ndarray, h: float, scheme: str) -> np.ndarray:
    n = f.shape[0]
    df = np.zeros_like(f)

    if scheme in ("centered", "centered4"):
        # f''(x_i) = (f_{i+1} - 2 f_i + f_{i-1}) / h^2, eq. 28.4.
        df[1:-1] = (f[2:] - 2 * f[1:-1] + f[:-2]) / h**2
        # Endpoints: 3-point one-sided, O(h).
        if n >= 3:
            df[0] = (f[2] - 2 * f[1] + f[0]) / h**2
            df[-1] = (f[-1] - 2 * f[-2] + f[-3]) / h**2
    elif scheme == "forward":
        # 3-point forward, O(h): (f_{i+2} - 2 f_{i+1} + f_i)/h^2.
        df[: -2] = (f[2:] - 2 * f[1:-1] + f[:-2]) / h**2
        df[-2] = df[-3]
        df[-1] = df[-3]
    elif scheme == "backward":
        df[2:] = (f[2:] - 2 * f[1:-1] + f[:-2]) / h**2
        df[0] = df[2]
        df[1] = df[2]
    else:
        raise ValueError(f"unknown scheme {scheme!r}")
    return df


# ----------------------------------------------------------------------
# Third derivative
# ----------------------------------------------------------------------
def _third_derivative(f: np.ndarray, h: float) -> np.ndarray:
    """Centered third derivative, O(h^2):

        f'''(x_i) = (f_{i+2} - 2 f_{i+1} + 2 f_{i-1} - f_{i-2}) / (2 h^3)

    (Chapra & Canale Table 28.3.)
    """
    n = f.shape[0]
    df = np.zeros_like(f)
    if n < 5:
        return df
    df[2:-2] = (f[4:] - 2 * f[3:-1] + 2 * f[1:-3] - f[:-4]) / (2 * h**3)
    # Replicate boundary samples from interior; jerk near the boundary
    # is unreliable with a centered 5-point stencil.
    df[0] = df[2]
    df[1] = df[2]
    df[-1] = df[-3]
    df[-2] = df[-3]
    return df


# ----------------------------------------------------------------------
# Convenience: extract velocity / acceleration / jerk profiles
# ----------------------------------------------------------------------
def kinematic_profile(t: np.ndarray, p: np.ndarray) -> dict:
    """Return position, velocity, acceleration, and jerk on the input grid.

    All derivatives are computed with the centered, second/fourth-order
    accurate formulas selected by ``scheme="auto"``. This is the
    "empirical" derivative used to assess how an interpolated trajectory
    would look when post-processed by a controller that does not have
    access to the analytic derivative formulas.
    """
    return {
        "t": t,
        "p": p,
        "v": differentiate(t, p, order=1, scheme="auto"),
        "a": differentiate(t, p, order=2, scheme="auto"),
        "j": differentiate(t, p, order=3, scheme="auto"),
    }
