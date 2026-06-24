# AetherScan hardware — common tasks.
# CAD runs under uv (zero-setup build123d); preview uses the sim venv (trimesh+matplotlib).

UV      := $(HOME)/.local/bin/uvx
VENV_PY := $(abspath ..)/redwood_sim/.venv/bin/python
CADDIR  := cad
PCBDIR  := electrical/pcb

.PHONY: all check cad parts assembly inertia preview pcb pcb-fab pcb-bom clean

all: check cad parts assembly preview

## Verify the CAD contract still matches redwood_sim (mass, arm, inertia, ...).
check:
	cd $(CADDIR) && $(VENV_PY) check_against_software.py

## Build the airframe → cad/out/frame.{glb,step,stl}
cad:
	cd $(CADDIR) && $(UV) --from build123d python frame.py

## Build the sensor brackets + compute tray.
parts:
	cd $(CADDIR) && $(UV) --from build123d python sensor_mounts.py
	cd $(CADDIR) && $(UV) --from build123d python compute_tray.py

## Build the full assembly (frame + component proxies) → cad/out/assembly.*
assembly:
	cd $(CADDIR) && $(UV) --from build123d python assembly.py

## Mass + inertia tensor vs the physics.py targets (no build123d needed).
inertia:
	cd $(CADDIR) && $(VENV_PY) assembly.py --inertia-only

## Render an STL → PNG (headless). Usage: make preview STL=out/assembly.stl
STL ?= out/frame.stl
preview:
	cd $(CADDIR) && $(VENV_PY) render_preview.py $(STL)

## Build the PSDB from code + clean placement (fresh layout every time).
## (atopile preserves manual edits, so we wipe elec/layout to avoid stale edits.)
pcb:
	cd $(PCBDIR) && rm -rf elec/layout && $(UV) --from atopile ato build && python3 place_board.py

## KiCad fab outputs: 3D render + DRC + JLCPCB gerbers → electrical/pcb/fab/
pcb-fab:
	cd $(PCBDIR) && ./fab.sh all

## Show the picked-parts BOM (real LCSC part numbers) after a build.
pcb-bom:
	@cat $(PCBDIR)/build/builds/psdb/psdb.bom.csv

clean:
	rm -rf $(CADDIR)/out/*
	rm -rf $(PCBDIR)/build
