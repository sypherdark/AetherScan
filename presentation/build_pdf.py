#!/usr/bin/env python3
"""Build the AetherScan 2-page investor brief (dark, modern, transparent)."""
from pathlib import Path
from reportlab.lib.pagesizes import A4
from reportlab.pdfgen import canvas
from reportlab.lib.utils import ImageReader
from PIL import Image

HERE = Path(__file__).parent
A = HERE / "assets"
OUT = HERE / "AetherScan_Investor_Brief.pdf"
W, H = A4  # 595 x 842 pt

# palette
BG     = (0.027, 0.035, 0.047)
PANEL  = (0.055, 0.075, 0.094)
PANEL2 = (0.078, 0.105, 0.130)
LINE   = (0.13, 0.18, 0.23)
CYAN   = (0.13, 0.83, 0.93)
MINT   = (0.20, 0.83, 0.60)
TXT    = (0.902, 0.929, 0.953)
MUT    = (0.55, 0.62, 0.70)
WARN   = (0.98, 0.75, 0.45)

c = canvas.Canvas(str(OUT), pagesize=A4)

def rect(x, y, w, h, fill=None, stroke=None, r=8, lw=0.8):
    if fill: c.setFillColorRGB(*fill)
    if stroke: c.setStrokeColorRGB(*stroke); c.setLineWidth(lw)
    c.roundRect(x, y, w, h, r, fill=1 if fill else 0, stroke=1 if stroke else 0)

def text(x, y, s, size=10, color=TXT, font="Helvetica", spacing=None):
    c.setFillColorRGB(*color); c.setFont(font, size)
    if spacing:
        c.drawString(x, y, ""); tx = x
        for ch in s:
            c.drawString(tx, y, ch); tx += c.stringWidth(ch, font, size) + spacing
    else:
        c.drawString(x, y, s)

def rtext(x, y, s, size=10, color=TXT, font="Helvetica"):
    c.setFillColorRGB(*color); c.setFont(font, size); c.drawRightString(x, y, s)

def ctext(x, y, s, size=10, color=TXT, font="Helvetica"):
    c.setFillColorRGB(*color); c.setFont(font, size); c.drawCentredString(x, y, s)

def wrap(x, y, s, size, color, font, maxw, leading):
    c.setFillColorRGB(*color); c.setFont(font, size)
    words = s.split(); line = ""
    for w_ in words:
        t = (line + " " + w_).strip()
        if c.stringWidth(t, font, size) > maxw and line:
            c.drawString(x, y, line); y -= leading; line = w_
        else:
            line = t
    if line: c.drawString(x, y, line); y -= leading
    return y

def img_fit(path, x, y, bw, bh, border=LINE):
    im = Image.open(path); iw, ih = im.size; ar = iw/ih
    if bw/bh > ar: dw = bh*ar; dh = bh
    else: dw = bw; dh = bw/ar
    ix = x + (bw-dw)/2; iy = y + (bh-dh)/2
    c.drawImage(ImageReader(path), ix, iy, dw, dh, mask='auto')
    if border:
        c.setStrokeColorRGB(*border); c.setLineWidth(1); c.roundRect(ix, iy, dw, dh, 4)
    return ix, iy, dw, dh

def logo(x, y, s=20):
    rect(x, y, s, s, fill=CYAN, r=5)
    c.setStrokeColorRGB(*BG); c.setLineWidth(1.6); c.setLineJoin(1)
    p = c.beginPath(); cx=x; b=y;
    pts=[(0.18,0.5),(0.36,0.5),(0.46,0.74),(0.6,0.26),(0.7,0.5),(0.84,0.5)]
    p.moveTo(x+pts[0][0]*s, y+pts[0][1]*s)
    for px,py in pts[1:]: p.lineTo(x+px*s, y+py*s)
    c.drawPath(p)

def header(small=False):
    top = H-44
    logo(40, top-4, 22)
    text(70, top, "Aether", 19, TXT, "Helvetica-Bold")
    wsc = c.stringWidth("Aether", "Helvetica-Bold", 19)
    text(70+wsc, top, "Scan", 19, CYAN, "Helvetica-Bold")
    rtext(W-40, top+4, "Autonomous Indoor 3D-Scanning Drone", 9.5, MUT, "Helvetica")
    rtext(W-40, top-8, "Investor Brief  ·  June 2026  ·  v1.0", 8, (0.4,0.46,0.54))
    c.setStrokeColorRGB(*LINE); c.setLineWidth(0.8); c.line(40, top-16, W-40, top-16)

# ───────────────────────── PAGE 1 ─────────────────────────
c.setFillColorRGB(*BG); c.rect(0, 0, W, H, fill=1, stroke=0)
header()

# headline
y = H-90
wrap(40, y, "A drone that flies itself through a building —", 17, TXT, "Helvetica-Bold", W-80, 21)
text(40, y-21, "and hands you the 3D model.", 17, CYAN, "Helvetica-Bold")
y -= 34
y = wrap(40, y, "AetherScan autonomously explores unknown indoor spaces with no GPS, maps them "
         "in real time, and exports a measured 3D reconstruction. The autonomy — perception, "
         "SLAM, exploration and flight control — is built from scratch and validated end to end.",
         9.5, MUT, "Helvetica", W-80, 13)

# hero image
y -= 6
hero_h = 250
img_fit(A/"dashboard_scan_apartment1.png", 40, y-hero_h, W-80, hero_h)
y -= hero_h
text(44, y-12, "Live capture — autonomous scan of a real indoor space (Meta Replica 'apartment_1'), single ~5-minute run.",
     7.6, MUT, "Helvetica-Oblique")
y -= 26

# stat strip
stats = [("87.8%","area covered"),("353K","points captured"),("58.8 m²","floor mapped"),("0","GPS satellites")]
sw = (W-80-3*10)/4
for i,(v,l) in enumerate(stats):
    sx = 40 + i*(sw+10)
    rect(sx, y-46, sw, 46, fill=PANEL, stroke=LINE, r=7)
    ctext(sx+sw/2, y-22, v, 17, CYAN if i<3 else MINT, "Helvetica-Bold")
    ctext(sx+sw/2, y-37, l, 7.5, MUT, "Helvetica")
y -= 64

# what works
text(40, y, "WHAT WORKS TODAY", 9, CYAN, "Helvetica-Bold")
y -= 16
bullets = [
 ("Fully autonomous, GPS-denied", "Frontier + coverage exploration over a log-odds occupancy grid the drone builds itself — it decides where to fly to finish the map."),
 ("Flies on its own estimate", "On-board SLAM (correlative scan-matching) corrects pose drift; flying on the estimated pose reaches coverage parity with ground truth."),
 ("Real-time reconstruction + export", "Dense semantic point cloud, exportable as coloured PLY, Poisson GLB mesh and an SVG floor plan with dimensions."),
 ("Validated flight stack", "6-DoF rigid-body physics at 500 Hz, cascaded controller: ~10-14° peak tilt, ±8 mm altitude hold across scenes."),
]
for tt, dd in bullets:
    c.setFillColorRGB(*MINT); c.circle(44, y-3, 2, fill=1, stroke=0)
    text(52, y, tt, 9.5, TXT, "Helvetica-Bold")
    y = wrap(52, y-12, dd, 8.6, MUT, "Helvetica", W-100, 11) - 6

# transparency band
by = 60
rect(40, by, W-80, 40, fill=PANEL2, stroke=LINE, r=8)
c.setFillColorRGB(*WARN); c.rect(40, by, 3, 40, fill=1, stroke=0)
text(52, by+26, "Honest status", 9, WARN, "Helvetica-Bold")
wrap(52, by+14, "Everything above is real output from the system running now. It is validated in a "
     "high-fidelity simulator with realistic sensor noise and pose drift. The aircraft is fully "
     "designed (below) but not yet physically built — that is what we are raising for.",
     8, MUT, "Helvetica", W-110, 10.5)
ctext(W/2, 30, "AetherScan  ·  confidential investor brief  ·  page 1 of 2", 7.5, (0.36,0.42,0.5))

c.showPage()

# ───────────────────────── PAGE 2 ─────────────────────────
c.setFillColorRGB(*BG); c.rect(0, 0, W, H, fill=1, stroke=0)
header(small=True)

y = H-86
text(40, y, "HOW IT WORKS", 9, CYAN, "Helvetica-Bold")
y -= 18
stages = ["Sense\n360° LiDAR + depth","Map\nlog-odds occupancy","Decide\nfrontier + coverage","Localize\nscan-match SLAM","Fly\n500 Hz controller"]
pw = (W-80-4*8)/5
for i,s in enumerate(stages):
    px = 40 + i*(pw+8)
    rect(px, y-40, pw, 40, fill=PANEL, stroke=LINE, r=7)
    a,b = s.split("\n")
    ctext(px+pw/2, y-17, a, 9.5, TXT, "Helvetica-Bold")
    ctext(px+pw/2, y-30, b, 6.7, MUT, "Helvetica")
    if i<4:
        c.setFillColorRGB(*CYAN); c.setFont("Helvetica-Bold", 11)
        c.drawCentredString(px+pw+4, y-24, "›")
y -= 58

# two columns (sized to content)
colw = (W-80-16)/2
lx = 40; rx = 40+colw+16
coltop = y
PANEL_H = 250
pbot = coltop - PANEL_H

# left: aircraft (dark brand render)
rect(lx, pbot, colw, PANEL_H, fill=PANEL, stroke=LINE, r=8)
text(lx+12, coltop-16, "THE AIRCRAFT", 9, CYAN, "Helvetica-Bold")
ih = 108
img_fit(A/"airframe_dark.png", lx+8, coltop-24-ih, colw-16, ih, border=None)
yy = coltop-24-ih-12
specs = [("Class","360 mm quad-X · 7\" guarded"),
         ("Mass / endurance","1.45 kg · ~14-17 min"),
         ("Sensors","RPLIDAR A2 · D435i · flow/ToF · IMU"),
         ("Compute","Jetson Orin Nano + Pixhawk 6C / PX4"),
         ("BOM","~$1,659 · CAD + PCB defined in code")]
for k,v in specs:
    text(lx+12, yy, k.upper()+"  ", 6.6, CYAN, "Helvetica-Bold")
    text(lx+12+c.stringWidth(k.upper()+"  ","Helvetica-Bold",6.6), yy, v, 7.8, TXT, "Helvetica")
    yy -= 12.5

# right: why it wins + market
rect(rx, pbot, colw, PANEL_H, fill=PANEL, stroke=LINE, r=8)
text(rx+12, coltop-16, "WHY IT WINS", 9, CYAN, "Helvetica-Bold")
yy = coltop-30
yy = wrap(rx+12, yy, "The moat is GPS-denied indoor autonomy. Most drones need GPS or a "
          "pilot; AetherScan needs neither.", 8.6, TXT, "Helvetica", colw-24, 11) - 4
for k,v in [("vs Skydio / DJI","outdoor, GPS-reliant"),
            ("vs Flyability","caged, pilot-flown"),
            ("vs Matterport / NavVis","tripod, manual")]:
    c.setFillColorRGB(*MINT); c.circle(rx+15, yy-3, 1.8, fill=1, stroke=0)
    text(rx+22, yy, k, 8.3, TXT, "Helvetica-Bold")
    rtext(rx+colw-12, yy, v, 7.6, MUT, "Helvetica")
    yy -= 15
yy -= 3
text(rx+12, yy, "FOCUS MARKET", 7.6, CYAN, "Helvetica-Bold"); yy -= 11
wrap(rx+12, yy, "Construction-progress capture & facility survey — daily, repeatable, "
     "GPS-denied interior 3D where speed and autonomy beat tripods.", 8, MUT, "Helvetica", colw-24, 10.5)

# proven-today badges
by = pbot - 22
text(40, by, "PROVEN IN SIMULATION TODAY", 9, CYAN, "Helvetica-Bold")
by -= 16
badges = ["Autonomous GPS-denied flight","On-board SLAM, drift bounded","500 Hz validated controller","PLY / GLB / floor-plan export"]
bw = (W-80-3*8)/4
for i,b in enumerate(badges):
    bx = 40+i*(bw+8)
    rect(bx, by-30, bw, 30, fill=PANEL2, stroke=LINE, r=6)
    c.setFillColorRGB(*MINT); c.setFont("Helvetica-Bold", 9); c.drawString(bx+9, by-19, "✓")
    wrap(bx+20, by-12, b, 7.0, TXT, "Helvetica", bw-26, 8.4)
by -= 46

# roadmap timeline
text(40, by, "ROADMAP TO FIRST FLIGHT", 9, CYAN, "Helvetica-Bold")
by -= 8
ty = by - 20
c.setStrokeColorRGB(*LINE); c.setLineWidth(1.4); c.line(58, ty, W-58, ty)
miles = [("Now","Sim validated\nend-to-end", MINT),
         ("Step 1","Compute + sensors\non the bench  (~$530)", CYAN),
         ("Step 2","Airframe + first\nautonomous scan  (~$530)", CYAN),
         ("Scale","Pilot deployments\n& v2", MUT)]
nx = (W-116)/(len(miles)-1)
for i,(t,d,col) in enumerate(miles):
    cx = 58 + i*nx
    c.setFillColorRGB(*col); c.circle(cx, ty, 4.2, fill=1, stroke=0)
    c.setFillColorRGB(*BG); c.circle(cx, ty, 1.7, fill=1, stroke=0)
    align = ctext
    if i==0: align = lambda X,Y,*a,**k: text(cx-6,Y,*a,**k)
    if i==len(miles)-1: align = lambda X,Y,*a,**k: rtext(cx+6,Y,*a,**k)
    align(cx, ty+10, t, 8.2, col, "Helvetica-Bold")
    for j,ln in enumerate(d.split("\n")):
        align(cx, ty-14-j*9, ln, 6.8, MUT, "Helvetica")

# THE ASK band
rect(40, 44, W-80, 40, fill=PANEL2, stroke=CYAN, r=8, lw=1.0)
text(52, 70, "THE ASK", 9, CYAN, "Helvetica-Bold")
wrap(52, 58, "Seed capital funds the first physical build. Bench-first staging reaches a fully "
     "autonomous indoor scan for ~$1,060 of parts. Still ahead and scoped: real-airframe "
     "bring-up, global loop closure, EMC / thermal bench tests.",
     7.9, MUT, "Helvetica", W-230, 10.3)
rtext(W-52, 70, "github.com/sypherdark/AetherScan", 8, CYAN, "Helvetica-Bold")
rtext(W-52, 58, "oaf01022006@gmail.com", 7.8, MUT, "Helvetica")
ctext(W/2, 30, "AetherScan  ·  confidential investor brief  ·  page 2 of 2", 7.5, (0.36,0.42,0.5))

c.showPage()
c.save()
print("wrote", OUT)
