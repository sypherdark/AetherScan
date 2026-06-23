# AetherScan hardware — common tasks.
# CAD runs under uv (zero-setup build123d); preview uses the sim venv (trimesh+matplotlib).

UV      := $(HOME)/.local/bin/uvx
VENV_PY := $(abspath ..)/redwood_sim/.venv/bin/python
CADDIR  := cad

.PHONY: all check cad preview clean

all: check cad preview

## Verify the CAD contract still matches redwood_sim (mass, arm, inertia, ...).
check:
	cd $(CADDIR) && $(VENV_PY) check_against_software.py

## Build the airframe → cad/out/frame.{glb,step,stl}
cad:
	cd $(CADDIR) && $(UV) --from build123d python frame.py

## Render cad/out/frame.stl → cad/out/frame_preview.png (headless, no GPU)
preview:
	cd $(CADDIR) && $(VENV_PY) render_preview.py out/frame.stl

clean:
	rm -rf $(CADDIR)/out/*
