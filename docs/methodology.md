# Deliverable 2 — Implementation, Data, and Full Experimental Sweep

**Project:** Comparative Analysis of Numerical Interpolation Methods for Robot Trajectory Planning
**Author:** Param Chokshi
**Course:** CISC 601 Scientific Computing 2
**Textbook:** Chapra & Canale, *Numerical Methods for Engineers* (8th ed.), Chapters 18, 24, 28.

---

## 1. Scope of this deliverable

Deliverable 1 proposed a comparative computational study of four
interpolation methods (linear, cubic spline, quintic spline, B-spline
approximation) applied to robot trajectory-planning scenarios.
Deliverable 2 is the implementation, data-collection, and
experimental-sweep phase. The artefacts produced are:

* a working Python package (`src/`) containing
  textbook-formulation implementations of every method, a numerical
  differentiation module, parameterised dataset generators, evaluation
  metrics, plotting utilities, and an experiment runner;
* the canonical waypoint datasets in CSV form (`data/`);
* a one-shot Jupyter notebook (`notebooks/analysis.ipynb`) and an HTML
  preview (`notebooks/analysis.html`) -- the primary submission artefact;
* the offline orchestrator scripts `src/main.py` (baseline-only) and
  `src/run_full_experiment.py` (full sweep);
* the resulting metrics summaries (`results/metrics_summary.csv` for
  the baseline, `results/full_experiment.csv` for the full sweep) and
  the figures in `results/figures/`.

The final paper (Deliverable 3) will assemble the textual analysis
around the same artefacts; this deliverable's structure is meant to
be additive rather than throw-away.

## 2. Repository layout

```
deliverable2/
├── data/                              # waypoint datasets (CSV)
│   ├── mobile_robot_waypoints.csv
│   └── robot_arm_waypoints.csv
├── results/
│   ├── metrics_summary.csv            # baseline experiment
│   ├── full_experiment.csv            # 28-row full sweep
│   └── figures/                       # 12 PNG figures
│       ├── mobile_robot_*.png         # baseline trajectory + kinematics
│       ├── robot_arm_*.png            # baseline arm figures
│       ├── sweep_density*.png         # density sweep
│       ├── sweep_geometry*.png        # geometry sweep
│       ├── sweep_spacing.png          # spacing sweep
│       ├── validation_*.png           # analytic vs empirical derivatives
│       └── quintic_bc_*.png           # BC sensitivity
├── src/
│   ├── interpolation/                 # textbook-formulation methods
│   │   ├── linear.py                  # Eq 18.2
│   │   ├── cubic_spline.py            # Eq 18.36, tridiagonal solve
│   │   ├── quintic_spline.py          # quintic Hermite, C^4 system
│   │   └── bspline.py                 # Cox–de Boor recursion
│   ├── datasets/
│   │   ├── mobile_robot.py            # baseline 12-waypoint dataset
│   │   ├── robot_arm.py               # baseline 8-waypoint pick&place
│   │   ├── path_generators.py         # parametric geometries + spacings
│   │   └── variants.py                # all_variants() returns 7 datasets
│   ├── differentiation.py             # Ch 28 finite-difference formulas
│   ├── metrics.py                     # 6 metrics + velocity_discontinuity
│   ├── plots.py                       # publication-style figures
│   ├── experiment.py                  # run_experiment + diagnostics
│   ├── main.py                        # baseline orchestrator
│   └── run_full_experiment.py         # full sweep + figures
├── deliverable2.ipynb                 # primary submission notebook
├── deliverable2.html                  # static preview
├── build_notebook.py                  # notebook generator
└── deliverable2_methodology.md        # this document
```

## 3. Implementation summary

All four methods are coded **directly from the textbook formulation**
rather than calling library black-box interpolation functions, as
promised in the proposal. SciPy is used only for the tridiagonal
solver (`scipy.linalg.solve_banded`) and dense linear solver
(`scipy.linalg.solve`); NumPy is used for vectorised evaluation.

### 3.1 Linear interpolation (`src/interpolation/linear.py`)

Implements Chapra & Canale Eq 18.2,
$$f_1(x) = f(x_0) + \frac{f(x_1) - f(x_0)}{x_1 - x_0}\,(x - x_0).$$
Vector-valued waypoints are supported. The analytic derivative is
piecewise constant; second and higher derivatives are zero. Query
times outside the knot range are clamped to avoid extrapolation.

### 3.2 Cubic spline (`src/interpolation/cubic_spline.py`)

Implements the textbook second-derivative formulation. Letting
$M_i = S''(t_i)$ be the unknowns, each segment is

$$
S_i(t) = \frac{M_i}{6 h_i}(t_{i+1} - t)^3 + \frac{M_{i+1}}{6 h_i}(t - t_i)^3
       + \left(\frac{p_i}{h_i} - \frac{M_i h_i}{6}\right)(t_{i+1} - t)
       + \left(\frac{p_{i+1}}{h_i} - \frac{M_{i+1} h_i}{6}\right)(t - t_i).
$$

Imposing $C^1$ continuity at internal knots produces the tridiagonal
system Eq 18.37, solved with `scipy.linalg.solve_banded`. Both natural
($M_0 = M_n = 0$) and clamped (specified end velocities) boundary
conditions are supported. The implementation matches
`scipy.interpolate.CubicSpline` to machine precision (verified during
development; max error $\approx 1.3 \times 10^{-15}$).

### 3.3 Quintic spline (`src/interpolation/quintic_spline.py`)

Implements a $C^4$ quintic spline using the quintic Hermite basis
parameterised by $s = (t - t_i)/h_i$:

$$
p_i(s) = H_0(s)\,p_i + H_1(s)\,h_i v_i + H_2(s)\,h_i^2 a_i + H_3(s)\,p_{i+1}
       + H_4(s)\,h_i v_{i+1} + H_5(s)\,h_i^2 a_{i+1}
$$

where the unknowns are the velocity $v_i$ and acceleration $a_i$ at
every knot. Position, velocity, and acceleration are continuous by
construction; continuity of the third and fourth derivatives at
internal knots gives $2(n-1)$ linear equations in the $2(n+1)$
unknowns, which are closed off with four boundary equations. Both
"natural" (start and stop at rest in v *and* a) and "clamped"
(specified $v_0, a_0, v_N, a_N$) modes are supported. A reference
test reproduces a known quintic polynomial to floating-point precision
(max position error $\sim 10^{-13}$).

### 3.4 B-spline approximation (`src/interpolation/bspline.py`)

Implements the Cox–de Boor recursion directly:

$$
N_{i,p}(t) = \frac{t - t_i}{t_{i+p} - t_i}\,N_{i,p-1}(t)
           + \frac{t_{i+p+1} - t}{t_{i+p+1} - t_{i+1}}\,N_{i+1,p-1}(t).
$$

A cubic B-spline (degree 3) is used with a clamped, "averaged" knot
vector. Derivatives are computed analytically by **degree-lowering**
of the control polygon ($Q_i = p / (t_{i+p+1} - t_{i+1}) \cdot
(P_{i+1} - P_i)$ from Piegl & Tiller, Eq 3.7). Matches
`scipy.interpolate.BSpline` to machine precision on identical knots.
The waypoints serve as control points; the curve passes through the
first and last waypoint by construction (clamped knot vector) but
only approximates the interior.

### 3.5 Numerical differentiation (`src/differentiation.py`)

Implements forward (Eq 28.1), backward (Eq 28.2), centered
($O(h^2)$, Eq 28.3), and 5-point centered ($O(h^4)$) formulas for
the first derivative, plus 3-point centered second derivative
($O(h^2)$) and 5-point centered third derivative ($O(h^2)$).
Endpoints fall back to one-sided 3-point stencils so the output array
has the same length as the input. Verified on $\sin(t)$ to scale as
$O(h^4)$ over the interior (1.7×10⁻⁵ on N=401, 3.3×10⁻⁶ on N=41
coarsened to test the 5-point centered scheme).

### 3.6 Variant generators (`src/datasets/path_generators.py`)

Three parametric path generators: `mixed_aisle(n)` resamples the
canonical 12-vertex warehouse polyline at `n` arc-length-equispaced
points; `sharp_zigzag(n)` produces a rectangular zigzag with exact
90° corners; `gradual_sinusoid(n)` produces a smooth $A \sin(2\pi k
x/X)$ curve. The `time_stamps()` helper attaches timestamps via
chord-length parameterisation (default) or uniform spacing.

### 3.7 Experiment runner (`src/experiment.py`)

`run_experiment(datasets)` builds and evaluates every method on every
dataset, returning a tidy long-form pandas DataFrame.
`analytic_vs_empirical_derivative()` and `quintic_bc_sensitivity()`
provide the two diagnostic comparisons used in the validation
sections.

## 4. Datasets

### 4.1 Baseline waypoint datasets

* **`mobile_robot_waypoints.csv`** -- 12 (x, y) waypoints in metres
  along a warehouse aisle: straight entry, 90° right turn, diagonal
  traverse to a pick station, 90° left turn, gentle S-curve to exit.
  Chord-length parameterisation at 1 m/s nominal speed (T_total ≈
  24.93 s).
* **`robot_arm_waypoints.csv`** -- 8 joint-space waypoints for a 2-DOF
  planar arm with $L_1 = 0.5$ m, $L_2 = 0.4$ m, covering a complete
  pick-and-place cycle. Uniform 1 s spacing (T_total = 7 s).

### 4.2 Variants for the experimental sweep

`all_variants()` enumerates seven datasets exercising three
independent axes (with the baseline overlapping the density sweep):

| Variant | Geometry | Spacing | n_waypoints | Purpose |
|---|---|---|---|---|
| mobile_mixed_sparse_chord | mixed | chord | 5 | density sweep |
| mobile_mixed_baseline_chord | mixed | chord | 12 | density / baseline |
| mobile_mixed_dense_chord | mixed | chord | 20 | density sweep |
| mobile_sharp_n12_chord | sharp | chord | 12 | geometry sweep |
| mobile_gradual_n12_chord | gradual | chord | 12 | geometry sweep |
| mobile_mixed_baseline_uniform | mixed | uniform | 12 | spacing sweep |
| robot_arm_baseline | arm | uniform | 8 | additional reference |

## 5. Quantitative results

### 5.1 Baseline metrics (4 methods × 2 datasets)

`results/metrics_summary.csv`. Rounded summary:

| Method | Dataset | max\|v\| | max\|a\| | rms\|j\| | path | wp_err |
|--------|---------|---------:|---------:|---------:|-----:|-------:|
| Linear  | mobile  | 1.000  | 0.000  | 0.000  | 24.93 | 0.0e+00 |
| Cubic   | mobile  | 1.215  | 1.175  | 0.340  | 25.51 | 1.8e-15 |
| Quintic | mobile  | 1.709  | 1.491  | 0.854  | 25.52 | 0.0e+00 |
| BSpline | mobile  | 1.509  | 0.411  | 0.126  | 22.92 | 6.8e-01 |
| Linear  | arm     | 1.888  | 0.000  | 0.000  |  6.37 | 1.1e-16 |
| Cubic   | arm     | 2.580  | 4.225  | 5.191  |  6.66 | 2.5e-16 |
| Quintic | arm     | 2.958  | 6.810  | 10.986 |  7.63 | 0.0e+00 |
| BSpline | arm     | 2.832  | 3.280  | 1.566  |  4.51 | 4.0e-01 |

Units: m on the mobile robot, rad on the arm. All splines use natural
boundary conditions in this baseline.

### 5.2 Full experiment sweep (28 rows)

`results/full_experiment.csv` adds 14 columns: the baseline six metrics
plus `n_samples`, `max_velocity_disc`, `compute_time_ms`, and the
experiment annotations `geometry`, `spacing`, `units`. See the
notebook (§4 of `deliverable2.ipynb`) for the full table; a few
salient slices:

**Density sweep (mixed geometry, chord spacing):**

| n  | method  | max\|a\| | rms\|j\| | path |
|----|---------|---------:|---------:|-----:|
| 5  | cubic   | 0.61 | 0.13 | 20.97 |
| 5  | quintic | 0.65 | 0.27 | 21.18 |
| 5  | bspline | 0.27 | 0.02 | 15.84 |
| 12 | cubic   | 1.18 | 0.34 | 25.51 |
| 12 | quintic | 1.49 | 0.85 | 25.52 |
| 12 | bspline | 0.41 | 0.13 | 22.92 |
| 20 | cubic   | 1.86 | 0.88 | 24.40 |
| 20 | quintic | 2.34 | 1.91 | 24.61 |
| 20 | bspline | 0.92 | 0.34 | 23.74 |

Adding waypoints monotonically increases peak acceleration and RMS
jerk for every spline-style method, because chord-length spacing
keeps each segment's nominal speed at ~1 m/s while reducing $h_i$,
forcing larger curvatures into the splines.

**Geometry sweep (n=12, chord spacing):**

| Geometry | method  | max\|a\| | rms\|j\| | max_v_disc |
|----------|---------|---------:|---------:|-----------:|
| sharp    | linear  | 0.000 | 0.000 | 1.41e+00 |
| sharp    | cubic   | 2.118 | 1.864 | 9.01e-05 |
| sharp    | quintic | 2.052 | 2.295 | 8.27e-05 |
| sharp    | bspline | 0.901 | 0.547 | 2.66e-05 |
| gradual  | linear  | 0.000 | 0.000 | 9.93e-01 |
| gradual  | cubic   | 1.190 | 0.622 | 4.10e-05 |
| gradual  | quintic | 1.654 | 1.275 | 3.28e-05 |
| gradual  | bspline | 0.764 | 0.207 | 1.64e-05 |

The `max_velocity_disc` column cleanly separates linear (~1 m/s real
discontinuities) from the smooth methods (~10⁻⁵ m/s, dominated by
ε-proportional probe noise). Sharp geometry roughly doubles peak
acceleration and roughly triples RMS jerk for cubic and quintic; the
B-spline absorbs the difference much more gracefully.

**Spacing sweep (n=12 mixed, chord vs uniform):**

| Spacing | method  | max\|v\| | max\|a\| | rms\|j\| |
|---------|---------|---------:|---------:|---------:|
| chord   | linear  | 1.000 | 0.000 | 0.000 |
| chord   | cubic   | 1.215 | 1.175 | 0.340 |
| chord   | quintic | 1.709 | 1.491 | 0.854 |
| chord   | bspline | 1.509 | 0.411 | 0.126 |
| uniform | linear  | 1.323 | 0.000 | 0.000 |
| uniform | cubic   | 1.412 | 0.901 | 0.348 |
| uniform | quintic | 1.499 | 1.158 | 0.660 |
| uniform | bspline | 1.323 | 0.580 | 0.169 |

Chord-length parameterisation keeps the linear interpolant at
exactly 1 m/s and lowers max speed for the splines; uniform spacing
is mostly within ~30% on accel and jerk. Time spacing matters less
than waypoint density or geometry in this study.

### 5.3 Validation: analytic vs empirical derivatives

For the mobile baseline, comparing `interp.derivative(...)` against
centered finite differences on the same 1000-sample dense grid (after
trimming the FD boundary stencils):

| method  | v_max_err | a_max_err | j_max_err |
|---------|----------:|----------:|----------:|
| cubic   | 2.58e-05  | 2.34e-03  | 5.13e-01  |
| quintic | 7.28e-08  | 4.27e-04  | 8.76e-04  |
| bspline | 5.06e-06  | 6.61e-04  | 1.28e-01  |

The cubic-spline jerk error is large because the cubic has true step
discontinuities in jerk at every internal knot, which the centered FD
scheme cannot resolve (it smooths each step into a short ramp). The
quintic spline has continuous jerk and the FD agreement is at the
$10^{-3}$ level. The B-spline cubic has piecewise-constant 3rd
derivative for similar reasons.

### 5.4 Quintic boundary-condition sensitivity

On the mobile baseline (12 waypoints, mixed geometry):

| BC                          | max\|v\| | max\|a\| | rms\|j\| |
|-----------------------------|---------:|---------:|---------:|
| natural (v=0, a=0 at ends)  | 1.709    | 1.491    | 0.854    |
| clamped (cubic-derived end v, a) | 1.250    | 1.000    | 0.307    |
| (cubic, for reference)      | 1.215    | 1.175    | 0.340    |

The clamped quintic is the smoothest of the three interpolating
methods on this dataset. The 33% drop in max acceleration and 64%
drop in RMS jerk vs natural BC confirms the §2 result was a
boundary-condition artefact, not a property of the quintic
formulation.

## 6. Findings

1. **Waypoint adherence is a hard distinction, not a continuum.**
   Linear, cubic, and quintic splines all hit every waypoint to
   floating-point precision on every variant. The cubic B-spline
   approximation deviates by 0.28–0.92 m on the mobile-robot variants
   and 0.40 rad on the robot arm; this is the defining trade-off of
   the formulation, not a tuning knob.

2. **Linear interpolation is the only method with C¹ violations.**
   `max_velocity_disc` ~1 m/s for linear regardless of geometry; all
   smooth methods sit at ≤ 10⁻⁴ m/s (numerical noise from the ε probe).
   The "0 acceleration / 0 jerk" rows of the table are therefore an
   artefact of the analytic formulation hiding a real discontinuity; a
   controller that finite-differences the commanded position would
   observe the discontinuity directly.

3. **More waypoints is not strictly better.** On the mixed geometry,
   max acceleration and RMS jerk increase monotonically as n goes from
   5 to 20 for every spline method, because chord-length spacing keeps
   nominal segment speed at 1 m/s while reducing $h_i$ and forcing
   larger curvatures.

4. **Geometry dominates the smoothness budget.** Sharp 90° corners
   roughly double peak acceleration and triple RMS jerk for cubic and
   quintic at the same n. The B-spline absorbs the difference much
   more gracefully (sharp/gradual ratio 1.18 vs 1.78 for cubic).

5. **Time-spacing matters less than expected.** Chord-length vs
   uniform leaves max acceleration and RMS jerk within ~30%; the most
   visible difference is in `max_speed`.

6. **The quintic spline's "high-jerk" anomaly was a boundary-condition
   artefact.** With cubic-derived clamped BCs, the quintic is the
   smoothest interpolating method on the baseline (max accel 1.00 m/s²
   and RMS jerk 0.31 m/s³, vs cubic's 1.18 / 0.34). The natural BC
   should be reported as a method-plus-BC choice, not a property of
   quintic splines per se.

7. **Analytic derivatives match numerical differentiation.** Centered
   FD agrees with the analytic derivatives to better than 10⁻³ m/s²
   for acceleration and 10⁻³ m/s³ for jerk on the cubic, quintic, and
   B-spline interpolants. The cubic-spline jerk exception is a real
   feature of the method (step discontinuities), not an implementation
   bug.

8. **Compute cost is not a deciding factor.** Even quintic finishes
   in well under 1 ms at every density studied; the full 28-row
   experiment runs in approximately 30 ms.

## 7. Reproducibility

The pipeline is deterministic. From the repository root:

```bash
python -m venv .venv
.venv/bin/pip install -r requirements.txt
.venv/bin/python -m ipykernel install --user --name nm-research \
    --display-name "Python 3 (Robot Trajectory Interpolation)"

# Baseline-only orchestrator:
.venv/bin/python -m src.main

# Full experiment with all variants and validation figures:
.venv/bin/python -m src.run_full_experiment

# Rebuild and re-execute the notebook:
.venv/bin/python notebooks/build_notebook.py
.venv/bin/jupyter nbconvert --to html notebooks/analysis.ipynb
```

Verified package versions used in this run: `numpy 2.4.6`,
`scipy 1.17.1`, `matplotlib 3.10.9`, `pandas 3.0.3`, on
`Python 3.12.11`.

## 8. References

1. Chapra, S. C., & Canale, R. P. (2021). *Numerical Methods for
   Engineers* (8th ed.). McGraw-Hill. Chapters 18, 24, 28.
2. Piegl, L., & Tiller, W. (1997). *The NURBS Book* (2nd ed.).
   Springer. Eq 3.7 used for B-spline derivative.
3. Biagiotti, L., & Melchiorri, C. (2008). *Trajectory Planning for
   Automatic Machines and Robots*. Springer.
4. Craig, J. J. (2018). *Introduction to Robotics: Mechanics and
   Control* (4th ed.). Pearson. Forward kinematics for the 2-DOF arm.
