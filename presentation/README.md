# Class Presentation — Comparative Analysis of Numerical Interpolation Methods for Robot Trajectory Planning

LaTeX Beamer slide deck for the in-class presentation of the research
project. Compiles to a 25-slide PDF reusing the figures from
Deliverable 2.

**Author:** Param Chokshi
**Course:** CISC 601 Scientific Computing 2

## Project structure

```
presentation/
├── presentation.tex      # Beamer source (~500 lines)
├── README.md             # this file
└── figures/              # PNGs copied from deliverable2/results/figures
```

## Slide outline (25 slides, ~15-20 minute talk)

| # | Section | Slide |
|---|---------|-------|
| 1 | Title | Title page |
| 2 | Agenda | 8-item outline |
| 3 | Motivation | Section divider |
| 4 | Motivation | Robots move through waypoints |
| 5 | Motivation | The smoothness budget |
| 6 | Methods | Section divider |
| 7 | Methods | Continuity-class table |
| 8 | Methodology | Methodology summary |
| 9 | Datasets | Section divider |
| 10 | Datasets | Mobile robot + 2-DOF arm side-by-side |
| 11 | Baseline | Section divider |
| 12 | Baseline | Mobile robot trajectory |
| 13 | Baseline | Mobile robot kinematic profiles |
| 14 | Baseline | Robot arm Cartesian path |
| 15 | Baseline | **Robot arm per-joint derivatives** (new) |
| 16 | Baseline | Quantitative summary table |
| 17 | Variant experiments | Section divider |
| 18 | Variant experiments | Three-axis design |
| 19 | Density sweep | Metrics figure |
| 20 | Density sweep | Trajectories figure |
| 21 | Density sweep | **Numerical table** (new) |
| 22 | Geometry sweep | Metrics figure |
| 23 | Geometry sweep | Trajectories figure |
| 24 | Spacing sweep | Metrics figure |
| 25 | Spacing sweep | **All variants at a glance** (new) |
| 26 | Validation | Section divider |
| 27 | Validation | Analytic vs centered finite differences |
| 28 | Surprise | Why quintic looked worse than cubic |
| 29 | Surprise | Quintic boundary-condition fix |
| 30 | Recommendations | Section divider |
| 31 | Recommendations | Decision rule |
| 32 | Recap | 8 findings recap |
| 33 | Limitations | Limitations + future work |
| 34 | Q&A | Thank you / contact |

(Section dividers are full-screen interstitials and are counted in
the slide tally.)

## Building the PDF

The Madrid theme and seahorse colour theme are part of the standard
Beamer distribution and ship with TeX Live, MacTeX, and MiKTeX by
default. No additional package install is needed.

### Local build

```bash
pdflatex presentation.tex
pdflatex presentation.tex   # second pass for cross-references
```

Or with latexmk:

```bash
latexmk -pdf presentation.tex
```

### Overleaf (recommended)

1. Create a new project, choose **Upload Project**, and upload the
   contents of `presentation/` as a zip.
2. Set the main document to `presentation.tex` (Menu → Settings).
3. Click Recompile.

### Docker

```bash
docker run --rm -v $(pwd):/data -w /data \
    texlive/texlive:latest pdflatex presentation.tex
```

## Customisation

Common things to tweak before presenting:

- **Theme**: change `\usetheme{Madrid}` near the top of
  `presentation.tex`. Other safe choices: `Frankfurt`, `Berlin`,
  `Boadilla`, `default`.
- **Colour theme**: `\usecolortheme{seahorse}` → try `crane`,
  `dolphin`, `lily`, etc.
- **Aspect ratio**: change `aspectratio=169` to `aspectratio=43` for
  4:3 projectors.
- **Date**: `\date{\today}` auto-fills; change to a fixed date for
  the actual class slot.

## Notes on figure sizing

A `\slidefigure` helper macro at the top of `presentation.tex`
applies `keepaspectratio` and a default height of
`0.75\textheight` so figures never overflow the slide boundary.
Override the height for individual slides if needed:

```latex
\slidefigure[height=0.6\textheight]{my_figure.png}
```
