#!/bin/bash
# Full autoroute pipeline for the PSDB:
#   (placed board) → unify GND + DRC rules → DSN → Freerouting → import SES → GND pour.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
PCB="$HERE/elec/layout/psdb/psdb.kicad_pcb"
KPY="/Applications/KiCad.app/Contents/Frameworks/Python.framework/Versions/3.9/bin/python3"
JAVA="/opt/homebrew/opt/openjdk/bin/java"
FR="/tmp/fr/freerouting.jar"
W=/tmp/fr; mkdir -p "$W"

# 1. unify ground, set fine-pitch design rules, export Specctra DSN
"$KPY" - "$PCB" "$W/psdb.dsn" <<'PY' 2>&1 | grep -vE "Debug:|assert|wxApp|Pgm|stdpbase|Translocat" | tail -4
import sys, wx; wx.App()
import pcbnew
pcb, dsn = sys.argv[1], sys.argv[2]
b = pcbnew.LoadBoard(pcb)
# --- ground unification: atopile fragments some module grounds into tiny nets.
# Fold every net that is electrically ground (named "lv" or "*-power-lv", but NOT
# the bootstrap "cbst-*") into the largest ground net so the board has one GND. ---
nets = {}
for fp in b.GetFootprints():
    for p in fp.Pads():
        nets.setdefault(p.GetNetname(), []).append(p)
gnd_like = [n for n in nets if (n == 'lv' or n.endswith('-power-lv')) and 'cbst' not in n]
main = max(gnd_like, key=lambda n: len(nets[n]))
mi = b.FindNet(main)
moved = 0
for n in gnd_like:
    if n != main:
        for p in nets[n]:
            p.SetNet(mi); moved += 1
print(f'GND unified into "{main}" (+{moved} pads merged)')
# --- fine-pitch-capable rules ---
nc = b.GetAllNetClasses()['Default']
nc.SetClearance(pcbnew.FromMM(0.15))
nc.SetTrackWidth(pcbnew.FromMM(0.25))
nc.SetViaDiameter(pcbnew.FromMM(0.5)); nc.SetViaDrill(pcbnew.FromMM(0.25))
pcbnew.SaveBoard(pcb, b)
print('DSN export:', pcbnew.ExportSpecctraDSN(b, dsn))
PY

# 2. autoroute
"$JAVA" -Djava.awt.headless=true -jar "$FR" -de "$W/psdb.dsn" -do "$W/psdb.ses" -mp 100 2>&1 | grep -E "session completed" | tail -1

# 3. import session, add a GND pour on both layers, fill
"$KPY" - "$PCB" "$W/psdb.ses" <<'PY' 2>&1 | grep -vE "Debug:|assert|wxApp|Pgm|stdpbase|Translocat" | tail -3
import sys, wx; wx.App()
import pcbnew
pcb, ses = sys.argv[1], sys.argv[2]
b = pcbnew.LoadBoard(pcb)
print('import SES:', pcbnew.ImportSpecctraSES(b, ses))
# ground pour on B.Cu over the whole board outline
gnet = max((n for n in {p.GetNetname() for fp in b.GetFootprints() for p in fp.Pads()}
            if n.endswith('-power-lv') or n == 'lv'),
           key=lambda n: sum(1 for fp in b.GetFootprints() for p in fp.Pads() if p.GetNetname()==n))
box = b.GetBoardEdgesBoundingBox()
for layer in (pcbnew.B_Cu,):
    z = pcbnew.ZONE(b); z.SetLayer(layer); z.SetNet(b.FindNet(gnet))
    z.SetIsFilled(True); z.SetLocalClearance(pcbnew.FromMM(0.25))
    pts = pcbnew.VECTOR_VECTOR2I()
    x0,y0,x1,y1 = box.GetLeft(),box.GetTop(),box.GetRight(),box.GetBottom()
    for x,y in [(x0,y0),(x1,y0),(x1,y1),(x0,y1)]:
        pts.append(pcbnew.VECTOR2I(x,y))
    z.AddPolygon(pts)
    b.Add(z)
pcbnew.ZONE_FILLER(b).Fill(b.Zones())
pcbnew.SaveBoard(pcb, b)
print('GND pour added on B.Cu; tracks:', len(list(b.GetTracks())))
PY
echo "routed -> $PCB"
