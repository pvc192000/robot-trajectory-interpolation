# Deliverable 3 — Research Paper Draft (IEEE Transactions / Journals format)

LaTeX source for the research paper draft of the
**Comparative Analysis of Numerical Interpolation Methods for Robot
Trajectory Planning** project. The paper is formatted using the
**IEEEtran** LaTeX class in journal mode, which produces the same
two-column layout as the IEEE Transactions / Journals Microsoft Word
template (`IEEE_journal_template-1-1-1-1.docx`) at the project
root. The IEEE Transactions template explicitly endorses LaTeX as an
alternative to Word, and IEEEtran is the official IEEE class file.

## Project structure

```
deliverable3/
├── main.tex                  # IEEEtran preamble, title block, \input directives
├── references.bib            # bibliography entries
├── Makefile                  # pdflatex/bibtex orchestration
├── sections/
│   ├── abstract.tex          # \begin{abstract}...\end{abstract} + \begin{IEEEkeywords}
│   ├── introduction.tex      # \IEEEPARstart drop-cap opening
│   ├── background.tex
│   ├── methodology.tex
│   ├── results.tex           # uses figure*/table* for two-column-spanning content
│   ├── discussion.tex
│   └── conclusion.tex
└── figures/                  # PNGs copied from deliverable2/results/figures
```

## IEEE template conformance

The project follows the conventions of the IEEE Transactions /
Journals Word template:

| Template element        | LaTeX implementation            |
|-------------------------|---------------------------------|
| Two-column layout       | `\documentclass[journal]{IEEEtran}` |
| Title and author block  | `\title{}` / `\author{}` with `\thanks{}` footnotes |
| Abstract                | `\begin{abstract}...\end{abstract}` (italics by class) |
| Index Terms             | `\begin{IEEEkeywords}...\end{IEEEkeywords}` |
| Drop-cap on Section I   | `\IEEEPARstart{First letter}{est of word}` |
| Bibliography style      | `\bibliographystyle{IEEEtran}` (numbered IEEE format) |
| Two-column-wide figures | `\begin{figure*}[!t]...\end{figure*}` |
| Two-column-wide tables  | `\begin{table*}[!t]...\end{table*}` |
| Section header style    | Default IEEEtran (Roman numerals on sections) |

## Building the PDF

### Option 1: Local TeX installation

If you have a TeX distribution installed (MacTeX, TeX Live, MiKTeX,
etc.) the IEEEtran class and bibliography style are already
included. Just run:

```bash
make            # full build via latexmk if available, otherwise
                # pdflatex + bibtex + 2× pdflatex
make watch      # latexmk -pvc continuous rebuild (latexmk required)
make clean      # remove auxiliary files and the PDF
```

The simplest macOS option is BasicTeX:

```bash
brew install --cask basictex
sudo tlmgr update --self
sudo tlmgr install latexmk siunitx tabularx booktabs cleveref
              microtype subcaption titlesec ieeetran
```

### Option 2: Overleaf (recommended)

The project compiles unmodified on
[Overleaf](https://www.overleaf.com/) — IEEEtran is one of the
default templates available there:

1. Create a new project, choose "Upload Project", and upload the
   contents of `deliverable3/` as a zip.
2. Set the main document to `main.tex` (Menu → Settings).
3. Compile.

Alternatively, you can start a fresh Overleaf project from their
"IEEE Transactions" template and copy the `sections/`, `figures/`,
and `references.bib` files into it.

### Option 3: Docker

Any TeX Live Docker image works:

```bash
docker run --rm -v $(pwd):/data -w /data \
    texlive/texlive:latest make
```

## Document structure

The paper follows the IEEE Transactions structure:

1. **Abstract** — italic 200-word summary of methods, key findings,
   and contributions.
2. **Index Terms** — keywords in alphabetical order.
3. **Introduction** (§I) — motivation, problem statement,
   contributions, paper outline, with IEEEPARstart drop-cap.
4. **Background** (§II) — continuity classes, textbook recap of
   each method, robotics context.
5. **Methodology** (§III) — implementation, datasets, variants,
   metrics, validation procedure.
6. **Results** (§IV) — baseline scenarios, density sweep, geometry
   sweep, spacing sweep, derivative validation, BC sensitivity.
7. **Discussion** (§V) — eight findings expanded, with practical
   recommendations.
8. **Conclusion** (§VI) — summary, limitations, future work,
   reproducibility.
9. **References** — IEEEtran-style numbered bibliography.

## Companion materials

The paper draws on artefacts produced in Deliverable 2. The companion
notebook (`../deliverable2/deliverable2.ipynb`) regenerates every
figure and table in this paper from the raw data. The full metrics
table is saved at `../deliverable2/results/full_experiment.csv`.

## Author and course information

**Author:** Param Chokshi
**Course:** CISC 601 Scientific Computing 2

The IEEE author block is configured in `main.tex` with two
`\thanks{}` footnotes per the IEEE Transactions template:
- The first footnote contains the manuscript-receipt date and a
  reference to the deliverable / project.
- The second footnote contains the author's affiliation and email
  (`pchokshi@my.harrisburgu.edu`).
