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
| **KiCad** (ERC/DRC/DFM) | **Reviews** the result: DFM, EMC pre-compliance, SPICE testbenches, BOM sourcing (DigiKey/Mouser/LCSC), JLCPCB/PCBWay export | `check_against_software.py` + `render_preview.py` |


atopile runs zero-setup via `uvx --from atopile ato …`.

## The loop (same as the CAD loop)

```bash
# author:   edit elec/src/psdb.ato   (the schematic, as code)
make pcb            # ato build → real parts picked, checks run, KiCad PCB emitted
make pcb-bom        # show the picked-parts BOM (real LCSC #s)
# review:  run the KiCad ERC/DRC + DFM checks
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
The schematic is **complete**: the real switching ICs (TI TPS568230 8 A, LMR33630
3 A) with their full application circuits — feedback dividers set for 5 V,
bootstrap caps, VCC decoupling, enable — plus the INA226 + 2 mΩ shunt for I²C
battery telemetry. 27 components, all picked from JLCPCB. The board is
**placed (auto-clustered by module) + outlined + 3D-rendered**, and Gerbers
export. **Still not routed** — copper traces are GUI work: open
`elec/layout/psdb/psdb.kicad_pcb` in KiCad ▸ **PCB Editor** and route (or use the
autorouter), then commit the `.kicad_pcb`.

### Next iterations (same loop)
- Add the switching-controller ICs (TI **TPS568230** 8 A, **LMR33630** 3 A) +
  **INA226** + 2 mΩ shunt, and the connectors / reverse-polarity P-FET + TVS.
  *(The current board is the power-path skeleton: bulk cap, two buck channels of
  passives, sense divider.)*
  **Note:** `ato create part --search` needs an **interactive terminal** (it
  drives a TTY picker — it errors when piped). Run it yourself in a real shell to
  fetch each IC from JLCPCB, or hand-author the component `.ato` + footprint.
- Route copper + a ground pour; clear the silkscreen overlaps.
- Run KiCad review `emc` + `kicad` DFM review; iterate to clean.
- Export the assembled board as STEP so it drops into the CAD assembly.
