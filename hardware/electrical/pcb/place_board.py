"""
Programmatic placement + board outline for the PSDB.
Edits elec/layout/psdb/psdb.kicad_pcb: positions each footprint into a clean
two-buck-channel layout and adds a 50x42 mm Edge.Cuts outline. atopile preserves
manual placement across rebuilds, so this is the right artifact to edit.

Run: python place_board.py   (then kicad-cli render / DRC / gerbers)
"""
import re, uuid
from pathlib import Path

PCB = Path(__file__).parent / "elec/layout/psdb/psdb.kicad_pcb"

# ref -> (x, y, rotation)  [mm; board 0..50 x 0..42]
POS = {
    "C5": (10, 21, 0),                                   # 470µF bulk (input)
    # buck_comp channel  (cin · L_sw · ferrite · cout)
    "C1": (17, 12, 0), "L2": (24, 12, 0), "L1": (31, 12, 0), "C2": (39, 12, 0),
    # buck_avi channel
    "C3": (17, 30, 0), "L4": (24, 30, 0), "L3": (31, 30, 0), "C4": (39, 30, 0),
    # battery-sense divider
    "R1": (45, 19, 90), "R2": (45, 24, 90),
}

txt = PCB.read_text()

# Idempotency guard: atopile PRESERVES manual layout edits across `ato build`, so
# running this twice would stack duplicate outlines (and any bad edit persists and
# can make the board unloadable). Always run on a FRESH layout:
#   rm -rf elec/layout && ato build && python3 place_board.py
if 'Edge.Cuts' in txt and 'gr_line' in txt:
    raise SystemExit(
        "Refusing to run: this layout already has an Edge.Cuts outline.\n"
        "Regenerate clean first:  rm -rf elec/layout && ato build && python3 place_board.py")

lines = txt.split("\n")
i = 0
placed = []
while i < len(lines):
    if lines[i].lstrip().startswith("(footprint "):
        ref = None
        for j in range(i, min(i + 45, len(lines))):
            m = re.search(r'\(property "Reference" "([^"]+)"', lines[j])
            if m:
                ref = m.group(1); break
        for j in range(i + 1, min(i + 14, len(lines))):
            if re.match(r"^\t\t\(at ", lines[j]):
                if ref in POS:
                    x, y, r = POS[ref]
                    lines[j] = f"\t\t(at {x} {y} {r})"
                    placed.append(ref)
                break
    i += 1

txt = "\n".join(lines)

# board outline (Edge.Cuts) — insert before final closing paren
def edge(x1, y1, x2, y2):
    return (f'\t(gr_line (start {x1} {y1}) (end {x2} {y2}) '
            f'(stroke (width 0.15) (type solid)) (layer "Edge.Cuts") '
            f'(uuid "{uuid.uuid4()}"))')
W, Hh = 50, 42
outline = "\n".join([edge(0,0,W,0), edge(W,0,W,Hh), edge(W,Hh,0,Hh), edge(0,Hh,0,0)])
# mounting holes as silk circles for reference (visual)
idx = txt.rstrip().rfind(")")
txt = txt[:idx] + outline + "\n" + txt[idx:]

PCB.write_text(txt)
print(f"placed {len(placed)} parts: {', '.join(placed)}")
print(f"added 50x42 mm Edge.Cuts outline -> {PCB}")
