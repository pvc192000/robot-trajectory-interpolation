# Makefile for Deliverable 3 LaTeX project (IEEEtran journal format)
#
# Builds main.pdf via pdflatex + bibtex (3 passes for citations and
# cross-references). The IEEEtran class and the IEEEtran bibliography
# style ship with TeX Live and MiKTeX by default; no extra package
# install is required on a standard distribution.
#
# Use `make` for a single full build, `make clean` to remove all
# build artefacts, or `make watch` for continuous rebuilds via latexmk.

MAIN     := main
SECTIONS := $(wildcard sections/*.tex)
BIB      := references.bib
FIGURES  := $(wildcard figures/*.png)

# Use latexmk if present (handles bib + multi-pass automatically),
# otherwise fall back to a direct pdflatex/bibtex sequence.
LATEXMK := $(shell command -v latexmk 2>/dev/null)

.PHONY: all clean watch help

all: $(MAIN).pdf

$(MAIN).pdf: $(MAIN).tex $(SECTIONS) $(BIB) $(FIGURES)
ifdef LATEXMK
	latexmk -pdf -interaction=nonstopmode $(MAIN).tex
else
	pdflatex -interaction=nonstopmode $(MAIN).tex
	bibtex $(MAIN)
	pdflatex -interaction=nonstopmode $(MAIN).tex
	pdflatex -interaction=nonstopmode $(MAIN).tex
endif

watch:
	@if [ -z "$(LATEXMK)" ]; then \
		echo "Error: latexmk is required for watch mode."; exit 1; \
	fi
	latexmk -pdf -pvc -interaction=nonstopmode $(MAIN).tex

clean:
ifdef LATEXMK
	latexmk -C
else
	rm -f $(MAIN).aux $(MAIN).bbl $(MAIN).blg $(MAIN).log $(MAIN).out
	rm -f $(MAIN).toc $(MAIN).fdb_latexmk $(MAIN).fls $(MAIN).synctex.gz
	rm -f sections/*.aux
	rm -f $(MAIN).pdf
endif

help:
	@echo "Available targets:"
	@echo "  make        Build main.pdf"
	@echo "  make watch  Continuously rebuild on file changes (needs latexmk)"
	@echo "  make clean  Remove all build artefacts"
