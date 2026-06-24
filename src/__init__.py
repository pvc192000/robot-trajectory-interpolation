"""Numerical interpolation methods for robot trajectory planning.

Implements four interpolation methods from Chapra & Canale,
"Numerical Methods for Engineers" (8th ed.), Chapters 18, 24:

- Linear interpolation (Ch 18.2)
- Cubic spline interpolation (Ch 18.6)
- Quintic spline interpolation (extension of Ch 18.6)
- B-spline approximation (de Boor recursion)

Plus numerical differentiation from Ch 28 for computing velocity,
acceleration, and jerk profiles from interpolated position data.
"""
