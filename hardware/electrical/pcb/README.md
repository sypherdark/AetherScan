# PSDB — code-defined PCB (atopile)

The Power & Sensor Distribution Board, authored **as code** with
[atopile](https://github.com/atopile/atopile) — the same workflow as the airframe
CAD: parametric source → a build → a verifiable artifact, all version-controlled
in-repo. No clicking around a schematic editor.

## Why this tool (chosen 2026-06-23)

We searched for a PCB tool that matches the project's code-first, verifiable
workflow (the way `cad/frame.py` builds the airframe). Two complementary pieces:

| Tool | Role | Analogue in our CAD flow |
|---|---|---|
| **atopile** (`ato`) | **Authors** the board in `.ato` code; compiler solves constraints, **picks real parts (LCSC)**, runs checks, emits a KiCad `.kicad_pcb` + BOM + power-tree | `build123d` / `frame.py` |
| **kicad-happy** (12 Claude skills) | **Reviews** the result: DFM, EMC pre-compliance, SPICE testbenches, BOM sourcing (DigiKey/Mouser/LCSC), JLCPCB/PCBWay export | `check_against_software.py` + `render_preview.py` |

kicad-happy is installed at `~/.claude/skills/{kicad,emc,spice,bom,jlcpcb,...}`;
atopile runs zero-setup via `uvx --from atopile ato …`.

## The loop (same as the CAD loop)

```bash
# author:   edit elec/src/psdb.ato   (the schematic, as code)
make pcb            # ato build → real parts picked, checks run, KiCad PCB emitted
make pcb-bom        # show the picked-parts BOM (real LCSC #s)
# review:  ask Claude to run the kicad-happy `kicad` / `emc` / `spice` skills
#          on build/builds/psdb/  → DFM / EMC / SPICE findings
```

Build output (`build/`, gitignored) includes `psdb.kicad_pcb`, `psdb.bom.csv`
(real manufacturer + LCSC part numbers), and `power_tree.md`.

## What `psdb.ato` currently builds

The structural power board from [../psdb-design.md](../psdb-design.md): VBAT input
with bulk decoupling, **two isolated 5 V rails** (8 A Jetson / 3 A avionics) each
modelled as its regulator network (input/output caps + inductor + output ferrite
bead), and the battery-sense divider. The compiler picks real passives (Murata,
UNI-ROYAL, …) with LCSC numbers and the build passes atopile's electrical checks.

### Next development iterations (same loop)
- Add the switching-controller ICs as components (`ato create part` / the
  kicad-happy `lcsc` skill: TI **TPS568230** 8 A, **LMR33630** 3 A) and the
  **INA226** telemetry IC + 2 mΩ shunt.
- Add the connectors (XT60, JST-GH breakouts) and the reverse-polarity P-FET + TVS.
- Place the 30.5×30.5 mounting + board outline (≤50×50 mm) so it drops into the
  CAD assembly as a STEP.
- Run kicad-happy `emc` + `kicad` DFM review; iterate to clean.
- Export JLCPCB Gerbers + assembly files (kicad-happy `jlcpcb` skill).

> KiCad itself is optional for the build (atopile updates the `.kicad_pcb`
> without it); install KiCad only to open/inspect the layout visually.
