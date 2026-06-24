"""
Assemble a 3D model of the PSDB from the REAL component STEP models atopile
fetched, on a representative board outline. Exports per-material STL groups to
/tmp/psdb_board/ for a coloured render (render_board_3d.py).

Run:  uvx --from build123d python build_board_3d.py
"""
from pathlib import Path
from build123d import Box, Cylinder, Pos, Rot, Compound, import_step, export_stl

PARTS = Path(__file__).parent / "elec/src/parts"
OUT = Path("/tmp/psdb_board"); OUT.mkdir(exist_ok=True)

STEP = {
    "c0805":  PARTS / "HRE_CGA0805X5R226M350MT/C0805_L2.0-W1.3-H1.3.step",      # 22µF cin
    "c1210":  PARTS / "Murata_Electronics_GRM32ER61C476KE15L/C1210_L3.2-W2.5-H2.5.step",  # 47µF cout
    "elec":   PARTS / "AISHI_ERJ1HM471G16C36T/CAP-TH_D10.0-H16.0-P5.00.step",   # 470µF bulk
    "l22":    PARTS / "cjiang_FTC252012S2R2MBCA/IND-SMD_L2.5-W2.0_MHCHL2520.step",       # 2.2µH
    "l10":    PARTS / "cjiang_FTC252012S1R0MBCA/IND-SMD_L2.5-W2.0-VLS252012CX-150M.step", # 1µH/ferrite
}
_cache = {k: import_step(str(v)) for k, v in STEP.items()}

BOARD_T = 1.6
TOP = BOARD_T

def place(key, x, y, rz=0):
    p = _cache[key]
    bb = p.bounding_box()
    p2 = Rot(0, 0, rz) * p
    bb = p2.bounding_box()
    return Pos(x, y, TOP - bb.min.Z) * p2

def box(x, y, w, d, h, rz=0):
    return Pos(x, y, TOP + h / 2) * Rot(0, 0, rz) * Box(w, d, h)

# ── Board (52 × 46 mm, 4× M3 mounts on the 30.5 stack pattern) ──────────────
board = Box(52, 46, BOARD_T)
for sx in (-1, 1):
    for sy in (-1, 1):
        board = board - (Pos(sx*23, sy*20, 0) * Cylinder(radius=1.6, height=BOARD_T+1))
        board = board - (Pos(sx*15.25, sy*15.25, 0) * Cylinder(radius=1.6, height=BOARD_T+1))

# ── Components (representative clean layout of the real parts) ───────────────
mlcc, ind, elec, conn, res = [], [], [], [], []

# Input: bulk electrolytic + XT60
elec.append(place("elec", -17, 9, 0))
conn.append(box(-24, -7, 16, 9, 8))               # XT60 input (left edge)

# Two buck channels (comp rail y=+8, avi rail y=-8): cin, L2.2, L1.0, cout
for yy in (8.5, -8.5):
    mlcc.append(place("c0805", -4, yy+4, 90))      # cin 22µF
    ind.append(place("l22",     3, yy, 0))         # 2.2µH switch inductor
    ind.append(place("l10",     9, yy, 0))         # 1µH output ferrite
    mlcc.append(place("c1210", 15.5, yy+4, 90))    # cout 47µF

# Battery-sense divider (right) + INA226 placeholder
res.append(box(20, 2, 1.6, 0.9, 0.5))             # R0603 10k
res.append(box(20, -2, 1.6, 0.9, 0.5))            # R0603 2k
conn.append(box(20.5, 9, 3, 3, 1.1))              # INA226 (small QFN)

# Output + sensor breakout headers (right edge)
conn.append(box(24, 14, 5, 10, 8))                # 5V_comp out
conn.append(box(24, -14, 5, 10, 8))               # 5V_avi out
for i, yy in enumerate((6, 0, -6)):
    conn.append(box(24, yy, 4, 5, 6))             # JST-GH sensor breakouts

groups = {"board": [board], "mlcc": mlcc, "ind": ind, "elec": elec, "conn": conn, "res": res}
for name, parts in groups.items():
    if not parts:
        continue
    comp = Compound(children=parts) if len(parts) > 1 else parts[0]
    export_stl(comp, str(OUT / f"{name}.stl"))
    print(f"  {name}: {len(parts)} part(s) -> {OUT/name}.stl")
print("done")
