# Comparative Analysis of Numerical Interpolation Methods for Robot Trajectory Planning

A reproducible study comparing four numerical interpolation methods for generating
smooth robot trajectories from sparse waypoints, and quantifying the smoothness,
accuracy, and computational trade-offs between them.

The methods are implemented from their textbook formulations (Chapra & Canale,
*Numerical Methods for Engineers*, 8th ed.) and evaluated on two robotics
scenarios — a mobile robot navigating a warehouse aisle and a 2-DOF planar arm
performing pick-and-place — across sweeps over waypoint density, path geometry,
and time parameterisation.

## Methods compared

| Method | Continuity | Interpolates waypoints? |
|--------|-----------|-------------------------|
| Linear interpolation | C⁰ | yes |
| Cubic spline | C² | yes |
| Quintic spline | C⁴ | yes |
| B-spline approximation (cubic) | C² | no (approximates) |

Velocity, acceleration, and jerk profiles are computed both analytically and via
finite-difference numerical differentiation (forward / centered, Ch. 28) and
cross-validated against each other.

## Key findings

1. Interpolating methods (linear / cubic / quintic) hit every waypoint to
   floating-point precision; the B-spline approximation deviates by 0.28–0.92 m
   on the mobile-robot variants.
2. Linear interpolation is the only method with C¹ violations — a ~1 m/s velocity
   jump at every waypoint regardless of geometry; smooth methods stay at ≤ 10⁻⁴ m/s.
3. Path geometry dominates the smoothness budget: sharp 90° corners roughly double
   peak acceleration and triple RMS jerk versus a gradual sinusoid.
4. The quintic spline's apparent "high-jerk" behaviour was a boundary-condition
   artefact; with cubic-derived clamped BCs it becomes the smoothest interpolating
   method on the baseline.
5. Analytic derivatives match centered finite differences to ≤ 10⁻³, validating
   both the closed-form formulas and the numerical-differentiation implementation.

See [`docs/methodology.md`](docs/methodology.md) for the full write-up and the
[research paper](docs/paper/) for the formal treatment.

## Repository layout

```
.
├── src/                       # Python package — the core implementation
│   ├── interpolation/         # linear, cubic_spline, quintic_spline, bspline
│   ├── datasets/              # waypoint generators + parametric path variants
│   ├── differentiation.py     # finite-difference formulas (Ch. 28)
│   ├── metrics.py             # smoothness / accuracy metrics
│   ├── plots.py               # figure generation
│   ├── main.py                # baseline orchestrator
│   └── run_full_experiment.py # full sweep + validation + BC sensitivity
├── data/                      # baseline waypoint datasets (CSV)
├── results/                   # generated metrics (CSV) and figures (PNG)
├── notebooks/                 # executable analysis notebook + builder script
│   ├── analysis.ipynb         # primary narrative notebook (all figures inlined)
│   ├── analysis.html          # static HTML preview
│   └── build_notebook.py      # regenerates and executes the notebook
├── docs/
│   ├── methodology.md         # implementation + experiment write-up
│   ├── proposal.md / .pdf     # original project proposal
│   ├── paper/                 # IEEEtran LaTeX research paper
│   └── presentation/          # Beamer slide deck
└── requirements.txt
```

## Getting started

```bash
python -m venv .venv
.venv/bin/pip install -r requirements.txt
```

### Run the analysis

```bash
# Baseline study (two scenarios, four methods):
.venv/bin/python -m src.main

# Full experiment — density / geometry / spacing sweeps,
# derivative validation, and quintic BC sensitivity:
.venv/bin/python -m src.run_full_experiment
```

Both scripts are deterministic and write their outputs to `data/` and `results/`.
The full 28-row experiment runs in well under a second.

### Explore the notebook

```bash
.venv/bin/python -m ipykernel install --user --name nm-research \
    --display-name "Python 3 (Robot Trajectory Interpolation)"
.venv/bin/jupyter notebook notebooks/analysis.ipynb
```

Or open [`notebooks/analysis.html`](notebooks/analysis.html) in a browser for a
static view with all figures and tables.

### Build the paper / slides

The LaTeX sources compile with any standard TeX distribution (or on Overleaf).
See [`docs/paper/README.md`](docs/paper/README.md) and
[`docs/presentation/README.md`](docs/presentation/README.md) for instructions.

## Datasets

- `data/mobile_robot_waypoints.csv` — 12-waypoint warehouse-aisle scenario.
- `data/robot_arm_waypoints.csv` — 8-waypoint planar pick-and-place.

Parametric variants (waypoint density 5 / 12 / 20, sharp-zigzag vs gradual-sinusoid
geometry, chord-length vs uniform time spacing) are generated on the fly by
`src/datasets/`.

## Author

**Param Chokshi** — CISC 601 Scientific Computing 2

Textbook reference: S. C. Chapra and R. P. Canale, *Numerical Methods for
Engineers*, 8th ed., McGraw-Hill, 2021 (Chs. 18, 24, 28).
