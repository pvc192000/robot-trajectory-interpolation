# Numerical Methods Research Project

## Course Info
- Author: Param Chokshi
- Course: CISC 601 Scientific Computing 2
- Textbook: "Numerical Methods for Engineers, 8th edition", Steven C. Chapra & Raymond P. Canale (ISBN: 1260232077, 2021, McGraw-Hill)
- Relevant chapters: 18 (Interpolation), 24 (Curve Fitting/Splines), 28 (Numerical Differentiation)

## Project Topic
Comparative Analysis of Numerical Interpolation Methods for Robot Trajectory Planning

## Methods Being Compared
1. Linear Interpolation
2. Cubic Spline Interpolation
3. Quintic Spline Interpolation
4. B-Spline Approximation
5. Numerical differentiation (forward/centered difference) for velocity/acceleration/jerk

## Deliverables
1. **Deliverable 1**: Research Project Initial Proposal (Word/PDF) — title, problem statement, proposed solution, methodology
2. **Deliverable 2**: Dataset, Data Analysis, and Math Modeling
3. **Deliverable 3**: Research Project Paper Draft
4. **Deliverable 4**: Final Research Paper

## Status
- [x] Deliverable 1 — proposal complete (`deliverable1_proposal.md`, `deliverable1_proposal.pdf`)
- [x] Deliverable 2 — full experiment, datasets, notebook, methodology complete (see `deliverable2/`)
- [x] Deliverable 3 — research paper draft in LaTeX, formatted with IEEEtran (see `deliverable3/`)
- [x] Class presentation — Beamer slide deck (see `presentation/`)
- [ ] Deliverable 4 — final research paper
- [ ] **GitHub upload — in progress** (see "GitHub Upload" section below)

## Files

### Deliverable 1 (root)
- `deliverable1_proposal.md` — initial proposal (Markdown source)
- `deliverable1_proposal.pdf` — rendered PDF for submission

### Deliverable 2 (`deliverable2/`)
- `deliverable2.ipynb` — **primary submission notebook** (37 cells, executed, 1.7 MB with all figures and dataframes inlined; covers baseline + density / geometry / spacing sweeps + validation + BC sensitivity + 8-point findings)
- `deliverable2.html` — rendered HTML preview of the same notebook (2.0 MB)
- `build_notebook.py` — generator script that builds and executes the notebook from the source modules
- `deliverable2_methodology.md` — written companion: implementation summary, dataset description, sweep results tables, findings, references
- `data/mobile_robot_waypoints.csv` — baseline 12-waypoint warehouse-aisle scenario
- `data/robot_arm_waypoints.csv` — baseline 8-waypoint pick-and-place
- `src/interpolation/{linear,cubic_spline,quintic_spline,bspline}.py` — textbook-formulation implementations of each method
- `src/datasets/{mobile_robot,robot_arm}.py` — baseline dataset generators
- `src/datasets/path_generators.py` — parametric path generators (mixed_aisle, sharp_zigzag, gradual_sinusoid) + chord/uniform spacing
- `src/datasets/variants.py` — `all_variants()` returns the 7 datasets used in the sweep
- `src/differentiation.py` — finite-difference formulas from Ch 28
- `src/metrics.py` — six metrics + `velocity_discontinuity` for C¹ violations
- `src/plots.py` — figure generators
- `src/experiment.py` — `run_experiment` + `analytic_vs_empirical_derivative` + `quintic_bc_sensitivity`
- `src/main.py` — baseline orchestrator (`python -m src.main`)
- `src/run_full_experiment.py` — full sweep + validation + BC sensitivity (`python -m src.run_full_experiment`)
- `results/metrics_summary.csv` — baseline 4 methods × 2 datasets
- `results/full_experiment.csv` — 28 rows × 14 columns, 4 methods × 7 variants
- `results/figures/*.png` — 12 figures (baseline + 5 sweep + validation + BC sensitivity)
- `.venv/` — Python 3.12 virtualenv with numpy/scipy/matplotlib/pandas/jupyter
- Jupyter kernel registered as `nm-research-d2` (display name "Python 3 (Numerical Methods D2)")

### Deliverable 3 (`deliverable3/`)
- `main.tex` — IEEEtran journal-class preamble, IEEE author block with `\thanks{}` footnotes, and section `\input{}` directives
- `references.bib` — 11 BibTeX entries (Chapra-Canale, Piegl-Tiller, Biagiotti-Melchiorri, Craig, Siciliano, de Boor, Kyriakopoulos, Macfarlane, NumPy, SciPy)
- `sections/abstract.tex` — `\begin{abstract}` block plus `\begin{IEEEkeywords}`
- `sections/introduction.tex` — opens with `\IEEEPARstart` drop-cap; motivation, problem statement, contributions, paper outline
- `sections/background.tex` — continuity classes, textbook recap of all 4 methods, numerical differentiation, robotics context
- `sections/methodology.tex` — implementation, both datasets, variant table, metrics, validation, BC sensitivity setup
- `sections/results.tex` — baseline + density / geometry / spacing sweeps + derivative validation + BC sensitivity (12 figures, 3 tables; wide content uses `figure*`/`table*`)
- `sections/discussion.tex` — 8 expanded findings
- `sections/conclusion.tex` — summary, recommendations, limitations, future work, reproducibility
- `figures/*.png` — 12 PNGs copied from `deliverable2/results/figures/`
- `Makefile` — pdflatex+bibtex orchestration with latexmk fallback
- `README.md` — IEEEtran conformance table, compile instructions for local TeX, Overleaf, Docker
- Format: IEEEtran journal class (matches `IEEE_journal_template-1-1-1-1.docx` at the project root)

### Class Presentation (`presentation/`)
- `presentation.tex` — Beamer slide deck (~22 slides, Madrid theme, 16:9 aspect ratio)
- `figures/*.png` — figures reused from Deliverable 2
- `README.md` — slide outline and compile instructions
- Compiles standalone with `pdflatex` (no bibliography needed)
- ZIP archive: `/presentation.zip` ready to upload to Overleaf

## Key Findings (Deliverable 2)
1. Interpolating methods (linear/cubic/quintic) hit every waypoint to floating-point precision; B-spline approximation deviates by 0.28–0.92 m on mobile-robot variants and 0.40 rad on the robot arm.
2. Linear interpolation is the only method with C¹ violations: ~1 m/s velocity jump at every waypoint regardless of geometry; smooth methods sit at ≤ 10⁻⁴ m/s.
3. Adding waypoints (5 → 12 → 20) on the same chord-length-parameterised geometry monotonically *increases* peak acceleration and RMS jerk for every spline method — denser inputs trade smoothness for geometric fidelity.
4. Geometry dominates the smoothness budget: sharp 90° corners roughly double max accel and triple RMS jerk vs the gradual sinusoid for cubic and quintic; the B-spline absorbs the difference much more gracefully (sharp/gradual ratio 1.18 vs 1.78 for cubic).
5. Time spacing (chord-length vs uniform) affects max speed but leaves max accel and RMS jerk within ~30%.
6. **The quintic spline's "high-jerk" anomaly was a boundary-condition artefact.** With cubic-derived clamped BCs, the quintic is the smoothest interpolating method on the baseline (max accel 1.00 m/s², RMS jerk 0.31 m/s³, vs cubic's 1.18 / 0.34).
7. Analytic derivatives match centered finite differences to ≤10⁻³ m/s² and ≤10⁻³ m/s³ for cubic, quintic, and B-spline — the only meaningful gap is cubic-spline jerk, which has *real* step discontinuities that FD cannot resolve.
8. Compute cost is sub-millisecond per method per variant; the full 28-row experiment runs in ~30 ms.

## Reproducing Results

### Run the offline pipeline
From `deliverable2/`:
```bash
python -m venv .venv
.venv/bin/pip install numpy scipy matplotlib pandas jupyter ipykernel nbformat
.venv/bin/python -m src.main
```

### Open the notebook
```bash
.venv/bin/python -m ipykernel install --user --name nm-research-d2 \
    --display-name "Python 3 (Numerical Methods D2)"
.venv/bin/jupyter notebook deliverable2.ipynb
```
or open `deliverable2.html` in any browser for a static view.

### Rebuild the executed notebook
```bash
.venv/bin/python build_notebook.py
.venv/bin/jupyter nbconvert --to html deliverable2.ipynb
```

## References
- See `deliverable1_proposal.md` for the full reference list.
- `deliverable2_methodology.md` adds Piegl & Tiller (NURBS Book) for the B-spline derivative formula.

## GitHub Upload (in progress)

**Goal:** publish this completed project to GitHub under https://github.com/pvc192000

**Status:** project copied to the dev-desk (`dev-dsk-pvc-1e-60cc5c9d.us-east-1.amazon.com:~/w/dev-agent/numerical-methods-research`) to be pushed from there. Not yet a git repo; not yet pushed.

### Decisions already made
- **Repo name (suggested):** `numerical-methods-research`
- **`.gitignore` is already created** in the project root and excludes: `.venv/`, `*.zip` (incl. the 175 MB `deliverable2.zip` which exceeds GitHub's 100 MB file limit), LaTeX build artifacts, and `.DS_Store`.
- **Excluded from the dev-desk copy** (do NOT re-add): `.venv/` (macOS-only, recreatable), all `*.zip` archives, `.DS_Store`. The actual source/content is only ~6–10 MB.
- **`deliverable2 copy/`** is a stray duplicate of `deliverable2/` that differs in 4 files (`build_notebook.py`, `deliverable2.html`, `deliverable2.ipynb`, `deliverable2_methodology.md`). Decide whether to keep or delete it before committing; recommended to delete it to avoid confusion.

### Remaining steps (resume on the dev-desk)
1. `cd ~/w/dev-agent/numerical-methods-research`
2. Recreate the venv if you need to run code (see "Reproducing Results"); not needed just to push.
3. Decide on `deliverable2 copy/` (keep or `rm -rf`).
4. Create an **empty** repo at `https://github.com/pvc192000/numerical-methods-research` (no README/license so the first push is clean). The `gh` CLI was NOT installed on the Mac — check if it's available on the dev-desk (`gh repo create pvc192000/numerical-methods-research --private --source=. --remote=origin`), otherwise create it in the web UI.
5. `git init` → `git add -A` → review `git status` to confirm `.venv`/zips are ignored → `git commit -m "Initial commit: numerical interpolation methods research project"`.
6. Add remote and push:
   - SSH: `git remote add origin git@github.com:pvc192000/numerical-methods-research.git`
   - HTTPS: `git remote add origin https://github.com/pvc192000/numerical-methods-research.git` (needs a PAT/credential helper)
   - `git branch -M main` → `git push -u origin main`
7. Verify on github.com that no file exceeds 100 MB and the `.venv` is absent.
