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

## Layout, 3D render & fab (KiCad)

KiCad 10 is installed (`/Applications/KiCad.app`), giving `kicad-cli` for renders /
DRC / Gerbers and the GUI for routing.

```bash
make pcb                       # 1. ato build  -> psdb.kicad_pcb (parts + nets)
python3 place_board.py         # 2. clean 2-channel placement + 50x42 mm outline
./fab.sh all                   # 3. KiCad: 3D render + DRC + JLCPCB gerbers -> fab/
```

`fab.sh` produces (in `fab/`, gitignored — regenerate any time):
- `psdb_3d.png`, `psdb_top.png` — photoreal KiCad renders of the real board.
- `drc.json` — design-rule check.
- `gerbers/` + `psdb_gerbers.zip` — fab output.

### Current status (honest)
The board is **placed + outlined + 3D-rendered**, Gerbers export. DRC is clean of
electrical errors; the only items are cosmetic silkscreen overlaps (auto-placed
designators) and **15 unconnected nets — i.e. it is not routed yet**. Routing is
GUI work: open `elec/layout/psdb/psdb.kicad_pcb` in KiCad ▸ **PCB Editor** and
route (or use the autorouter), then commit the `.kicad_pcb`.

### Next iterations (same loop)
- Add the switching-controller ICs (`ato create part`: TI **TPS568230** 8 A,
  **LMR33630** 3 A) + **INA226** + 2 mΩ shunt, and the connectors / reverse-polarity
  P-FET + TVS.  *(The current board is the power-path skeleton: bulk cap, two buck
  channels of passives, sense divider.)*
- Route copper + a ground pour; clear the silkscreen overlaps.
- Run kicad-happy `emc` + `kicad` DFM review; iterate to clean.
- Export the assembled board as STEP so it drops into the CAD assembly.
