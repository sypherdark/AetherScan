"""
Generate elec/layout/psdb/psdb.kicad_sch from the routed board's netlist
(/tmp/netlist.json), so the schematic is in accordance with the PCB by
construction: every component symbol carries a global net label on each pin with
the exact net name from the board. Connectivity = matching net-name labels.

Run with the redwood_sim venv (has kiutils):
    redwood_sim/.venv/bin/python build_schematic.py
"""
import json, glob, uuid, copy
from pathlib import Path
from kiutils.schematic import Schematic
from kiutils.symbol import SymbolLib
from kiutils.items.schitems import SchematicSymbol, GlobalLabel
from kiutils.items.common import Position, Property, Effects, Font

HERE = Path(__file__).parent
PARTS = HERE / "elec/src/parts"
DEV = "/Applications/KiCad.app/Contents/SharedSupport/symbols/Device.kicad_sym"
POWER = "/Applications/KiCad.app/Contents/SharedSupport/symbols/power.kicad_sym"
OUT = HERE / "elec/layout/psdb/psdb.kicad_sch"

comps = json.load(open("/tmp/netlist.json"))

# --- load symbol libraries we draw from -------------------------------------
device = SymbolLib.from_file(DEV)
dev = {s.libId: s for s in device.symbols}          # 'R','C','L','C_Polarized'
ic_libs = {}                                        # nickname -> Symbol
for f in glob.glob(str(PARTS / "Texas_Instruments_*/*.kicad_sym")):
    nick = Path(f).parent.name
    sym = SymbolLib.from_file(f).symbols[0]
    ic_libs[nick] = sym

def pin_geom(sym):
    g = {}
    for u in sym.units:
        for p in u.pins:
            g[p.number] = (p.position.X, p.position.Y)
    return g

def font(sz=1.27):
    return Effects(font=Font(width=sz, height=sz))

# --- pick the source symbol for each component ------------------------------
def source_symbol(c):
    ref, fp, val = c["ref"], c["footprint"], c["value"]
    if ref.startswith("R"):
        return "Device", "R", dev["R"]
    if ref.startswith("L"):
        return "Device", "L", dev["L"]
    if ref.startswith("C"):
        if "CAP-TH" in fp or "470" in val:           # the bulk electrolytic
            return "Device", "C_Polarized", dev["C_Polarized"]
        return "Device", "C", dev["C"]
    if ref.startswith("U"):
        nick = fp.split(":")[0]
        sym = ic_libs[nick]
        return nick, sym.libId, sym
    raise ValueError(ref)

# --- clustered grid placement (mm) ------------------------------------------
def slot(i, n, x0, y0, cols, dx, dy):
    return x0 + (i % cols) * dx, y0 + (i // cols) * dy

def keyf(c):
    return (c["ref"][0], int("".join(filter(str.isdigit, c["ref"])) or 0))
passives = sorted([c for c in comps if not c["ref"].startswith("U")], key=keyf)
ics = sorted([c for c in comps if c["ref"].startswith("U")], key=keyf)

sch = Schematic.create_new()
sch.paper.paperSize = "A2"          # room for 24 passives + 3 ICs on one sheet
sch.libSymbols = []
seen_lib = {}

PWR_NETS = {}  # net -> a position to drop a PWR_FLAG (first power pin seen)

def add_lib_symbol(lib_id, src):
    if lib_id in seen_lib:
        return
    s = copy.deepcopy(src)
    s.libId = lib_id
    sch.libSymbols.append(s)
    seen_lib[lib_id] = True

# passives in a grid (top); ICs on a wide row (bottom)
def positions():
    cols = 6
    for i, c in enumerate(passives):
        yield c, 40 + (i % cols) * 76, 50 + (i // cols) * 60
    for j, c in enumerate(ics):
        yield c, 90 + j * 150, 50 + ((len(passives) + cols - 1) // cols) * 60 + 50

for c, gx, gy in positions():
    nick, entry, src = source_symbol(c)
    lib_id = f"{nick}:{entry}"
    add_lib_symbol(lib_id, src)

    su = str(uuid.uuid4())
    sy = SchematicSymbol(
        libraryNickname=nick, entryName=entry,
        position=Position(gx, gy, 0), unit=1, inBom=True, onBoard=True, uuid=su,
        properties=[
            Property(key="Reference", value=c["ref"], position=Position(gx + 5, gy - 2, 0), effects=font()),
            Property(key="Value", value=c["value"][:24], position=Position(gx + 5, gy + 2, 0), effects=font()),
        ],
    )
    sch.schematicSymbols.append(sy)

    # global label at each pin endpoint (schematic Y is inverted vs symbol lib Y)
    geom = pin_geom(src)
    for padnum, net in c["pads"].items():
        if not net:
            continue
        if padnum not in geom:
            continue
        px, py = geom[padnum]
        ax, ay = gx + px, gy - py
        sch.globalLabels.append(GlobalLabel(
            text=net, position=Position(ax, ay, 0), effects=font(1.0), uuid=str(uuid.uuid4())))

sch.to_file(str(OUT))
print(f"wrote {OUT}")
print(f"  {len(sch.schematicSymbols)} symbols, {len(sch.globalLabels)} net labels, {len(sch.libSymbols)} lib symbols")
