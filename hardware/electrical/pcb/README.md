# PSDB — KiCad project (placeholder)

The Power & Sensor Distribution Board schematic + layout will be generated here
using the **kicad-happy** skill (https://github.com/aklofas/kicad-happy), to the
requirements in `../README.md` (E1–E8).

**Status:** awaiting the kicad-happy skill install. Once it's available:

1. Generate the schematic implementing E1–E8 (XT60 in, dual isolated 5 V bucks,
   sensor breakout, 30.5×30.5 mounting).
2. Lay out the board ≤ 50×50 mm, ≤ 25 g (matches `bom.csv` + frame stack pattern).
3. Export Gerbers + BOM + PnP for JLCPCB.
4. Add the board outline as a STEP so it drops into the CAD assembly and the
   mass/inertia model picks it up.

Files (when generated):
```
psdb.kicad_pro      project
psdb.kicad_sch      schematic
psdb.kicad_pcb      layout
psdb.step           3D body for the CAD assembly
gerbers/            fab output
bom.csv             board BOM (rolls up into ../../bom/bom.csv)
```
