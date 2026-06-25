"""
Auto-placement + board outline for the PSDB.
Groups footprints by their atopile module address (buck_comp.* / buck_avi.* /
top-level INA cluster) and lays each cluster out — IC on the left, its passives
in a grid to the right — then adds a board outline.

atopile PRESERVES manual layout edits across `ato build`, so always run on a FRESH
layout:  rm -rf elec/layout && ato build && python3 place_board.py
(`make pcb` does this for you.)
"""
import re, uuid
from pathlib import Path

PCB = Path(__file__).parent / "elec/layout/psdb/psdb.kicad_pcb"
W, Hh = 70, 54

# region: (ic_x, ic_y, grid_x0, grid_y0, cols, dx, dy)
# Tight clusters: each regulator's passives sit close to its IC so routes are
# short (the autorouter completes all nets only when the buck loops are compact).
REGIONS = {
    "buck_comp": (10, 14, 18, 7,  4, 6.5, 7),   # top band, 4-col tight grid
    "buck_avi":  (10, 41, 18, 34, 4, 6.5, 7),   # bottom band
    "ina":       (60, 13, 47, 24, 2, 7.5, 8),   # right column (INA + bulk + shunt)
}
def clamp(v, lo, hi): return max(lo, min(hi, v))
def region_of(addr: str) -> str:
    if addr.startswith("buck_comp"): return "buck_comp"
    if addr.startswith("buck_avi"):  return "buck_avi"
    return "ina"

txt = PCB.read_text()
if "Edge.Cuts" in txt and "gr_line" in txt:
    raise SystemExit("Already placed — regenerate clean: rm -rf elec/layout && ato build && python3 place_board.py")
lines = txt.split("\n")

# collect footprints: (line_index_of_top_at, reference, address)
fps = []
i = 0
while i < len(lines):
    if lines[i].lstrip().startswith("(footprint "):
        ref = addr = None
        at_idx = None
        j = i + 1
        while j < len(lines) and not lines[j].startswith("\t)"):   # until footprint close
            if at_idx is None and re.match(r"^\t\t\(at ", lines[j]):
                at_idx = j
            m = re.search(r'\(property "Reference" "([^"]+)"', lines[j])
            if m: ref = m.group(1)
            m = re.search(r'\(property "atopile_address" "([^"]+)"', lines[j])
            if m: addr = m.group(1)
            j += 1
        if at_idx and ref:
            fps.append((at_idx, ref, addr or ""))
        i = j
    else:
        i += 1

# assign positions per region (IC = reference starting with U at the anchor)
counters = {k: 0 for k in REGIONS}
placed = []
for at_idx, ref, addr in fps:
    reg = region_of(addr)
    icx, icy, gx, gy, cols, dx, dy = REGIONS[reg]
    if ref.startswith("U"):
        x, y, rot = icx, icy, 0
    else:
        n = counters[reg]; counters[reg] += 1
        x = gx + (n % cols) * dx
        y = gy + (n // cols) * dy
        rot = 0
    x = clamp(x, 4, W - 4); y = clamp(y, 4, Hh - 4)
    lines[at_idx] = f"\t\t(at {round(x,2)} {round(y,2)} {rot})"
    placed.append(ref)

txt = "\n".join(lines)

# board outline (Edge.Cuts)
def edge(x1, y1, x2, y2):
    return (f'\t(gr_line (start {x1} {y1}) (end {x2} {y2}) '
            f'(stroke (width 0.15) (type solid)) (layer "Edge.Cuts") (uuid "{uuid.uuid4()}"))')
outline = "\n".join([edge(0,0,W,0), edge(W,0,W,Hh), edge(W,Hh,0,Hh), edge(0,Hh,0,0)])
idx = txt.rstrip().rfind(")")
txt = txt[:idx] + outline + "\n" + txt[idx:]

PCB.write_text(txt)
print(f"placed {len(placed)} parts in 3 clusters; {W}x{Hh} mm outline -> {PCB}")
