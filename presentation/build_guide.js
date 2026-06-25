// AetherScan — Founder's Master Guide. Generates a comprehensive study .docx.
const fs = require("fs");
const { Document, Packer, Paragraph, TextRun, Table, TableRow, TableCell, ImageRun,
  AlignmentType, LevelFormat, HeadingLevel, BorderStyle, WidthType, ShadingType,
  TableOfContents, PageBreak, PageNumber, Header, Footer } = require("docx");

const FONT = "Calibri";
const NAVY = "1F3B57", BLUE = "2E6CA4", CYAN = "0E7C86", GREY = "5A6470", MINT = "1B7A4B", AMBER = "9C6A12";

// ---------- helpers ----------
const H1 = (t) => new Paragraph({ heading: HeadingLevel.HEADING_1, children: [new TextRun(t)] });
const H2 = (t) => new Paragraph({ heading: HeadingLevel.HEADING_2, children: [new TextRun(t)] });
const H3 = (t) => new Paragraph({ heading: HeadingLevel.HEADING_3, children: [new TextRun(t)] });
const P = (runs, opts = {}) => new Paragraph({ spacing: { after: 120, line: 276 }, children: Array.isArray(runs) ? runs : [new TextRun(runs)], ...opts });
const T = (t, o = {}) => new TextRun({ text: t, ...o });
const B = (t) => new TextRun({ text: t, bold: true });
const lead = (label, t) => P([new TextRun({ text: label + " ", bold: true, color: BLUE }), new TextRun(t)]);
const bullet = (runs) => new Paragraph({ numbering: { reference: "b", level: 0 }, spacing: { after: 70, line: 270 }, children: Array.isArray(runs) ? runs : [new TextRun(runs)] });
const num = (runs) => new Paragraph({ numbering: { reference: "n", level: 0 }, spacing: { after: 70, line: 270 }, children: Array.isArray(runs) ? runs : [new TextRun(runs)] });
const br = () => new Paragraph({ children: [new PageBreak()] });
const space = (h = 80) => new Paragraph({ spacing: { after: h }, children: [new TextRun("")] });

function callout(title, body, color = CYAN) {
  return new Table({
    width: { size: 9360, type: WidthType.DXA }, columnWidths: [9360],
    borders: { top: { style: BorderStyle.NONE }, bottom: { style: BorderStyle.NONE }, left: { style: BorderStyle.SINGLE, size: 24, color }, right: { style: BorderStyle.NONE }, insideHorizontal: { style: BorderStyle.NONE }, insideVertical: { style: BorderStyle.NONE } },
    rows: [new TableRow({ children: [new TableCell({
      width: { size: 9360, type: WidthType.DXA }, shading: { fill: "F2F6F8", type: ShadingType.CLEAR },
      margins: { top: 120, bottom: 120, left: 200, right: 160 },
      children: [
        new Paragraph({ spacing: { after: 60 }, children: [new TextRun({ text: title, bold: true, color })] }),
        ...(Array.isArray(body) ? body : [new Paragraph({ children: [new TextRun(body)] })]),
      ] })] })],
  });
}

function table(headers, rows, widths) {
  const tw = widths.reduce((a, b) => a + b, 0);
  const hd = { style: BorderStyle.SINGLE, size: 1, color: "BBBBBB" };
  const bd = { top: hd, bottom: hd, left: hd, right: hd };
  const headRow = new TableRow({ tableHeader: true, children: headers.map((h, i) => new TableCell({
    borders: bd, width: { size: widths[i], type: WidthType.DXA }, shading: { fill: NAVY, type: ShadingType.CLEAR },
    margins: { top: 70, bottom: 70, left: 110, right: 110 },
    children: [new Paragraph({ children: [new TextRun({ text: h, bold: true, color: "FFFFFF", size: 20 })] })] })) });
  const dataRows = rows.map((r, ri) => new TableRow({ children: r.map((c, i) => new TableCell({
    borders: bd, width: { size: widths[i], type: WidthType.DXA }, shading: { fill: ri % 2 ? "F4F6F8" : "FFFFFF", type: ShadingType.CLEAR },
    margins: { top: 60, bottom: 60, left: 110, right: 110 },
    children: [new Paragraph({ children: [new TextRun({ text: String(c), size: 20 })] })] })) }));
  return new Table({ width: { size: tw, type: WidthType.DXA }, columnWidths: widths, rows: [headRow, ...dataRows] });
}

function qa(q, a) {
  return [
    new Paragraph({ spacing: { before: 140, after: 50 }, children: [new TextRun({ text: "Q.  ", bold: true, color: AMBER }), new TextRun({ text: q, bold: true })] }),
    ...(Array.isArray(a) ? a : [new Paragraph({ spacing: { after: 80, line: 276 }, children: [new TextRun({ text: "A.  ", bold: true, color: MINT }), new TextRun(a)] })]),
  ];
}
function ansP(t) { return new Paragraph({ spacing: { after: 80, line: 276 }, children: [new TextRun({ text: "A.  ", bold: true, color: MINT }), new TextRun(t)] }); }
function plainP(t) { return new Paragraph({ spacing: { after: 80, line: 276 }, indent: { left: 300 }, children: Array.isArray(t) ? t : [new TextRun(t)] }); }

function img(path, w, h, caption) {
  const out = [new Paragraph({ alignment: AlignmentType.CENTER, spacing: { before: 80, after: 40 }, children: [new ImageRun({ type: "png", data: fs.readFileSync(path), transformation: { width: w, height: h }, altText: { title: caption, description: caption, name: caption } })] })];
  if (caption) out.push(new Paragraph({ alignment: AlignmentType.CENTER, spacing: { after: 140 }, children: [new TextRun({ text: caption, italics: true, size: 18, color: GREY })] }));
  return out;
}

const A = "assets/";
const children = [];

// ===================== COVER =====================
children.push(
  new Paragraph({ spacing: { before: 1600, after: 0 }, alignment: AlignmentType.CENTER, children: [new TextRun({ text: "AetherScan", bold: true, size: 72, color: NAVY })] }),
  new Paragraph({ alignment: AlignmentType.CENTER, spacing: { after: 80 }, children: [new TextRun({ text: "Autonomous Indoor 3D-Scanning Drone", size: 30, color: BLUE })] }),
  new Paragraph({ alignment: AlignmentType.CENTER, spacing: { after: 600 }, children: [new TextRun({ text: "The Founder's Master Guide", size: 26, italics: true, color: GREY })] }),
  ...img(A + "dashboard_scan_apartment1.png", 460, 259, "A real autonomous scan run — the system you are pitching."),
  new Paragraph({ alignment: AlignmentType.CENTER, spacing: { before: 300 }, children: [new TextRun({ text: "Everything in this project, explained — so you can speak to any part of it.", size: 22, color: GREY })] }),
  new Paragraph({ alignment: AlignmentType.CENTER, spacing: { before: 40 }, children: [new TextRun({ text: "Prepared for the founder · June 2026 · github.com/sypherdark/AetherScan", size: 18, color: GREY })] }),
  br(),
);

// ===================== HOW TO USE + TOC =====================
children.push(H1("How to use this guide"));
children.push(P([B("Read it once cover to cover, then drill the two sections that win pitches: "), T("§2 (the pitch you memorize) and §18 (the investor Q&A). "), T("Every technical section is written twice — first in plain English ('what it does, in one breath'), then deeper ('how it actually works'). You only need the plain-English version to pitch; the deeper version is there so a hard question never catches you flat.")]));
children.push(callout("The single most important framing", [P([T("You built a "), B("validated autonomy system in simulation"), T(" and a "), B("complete, reviewed hardware design"), T(". You have "), B("not"), T(" yet flown a physical drone — and that is exactly what the raise funds. Say this plainly and you will be trusted. Hide it and one sharp question ends the meeting.")])], AMBER));
children.push(new Paragraph({ spacing: { before: 160 }, children: [new TextRun({ text: "Contents", bold: true, size: 26, color: NAVY })] }));
children.push(new TableOfContents("Contents", { hyperlink: true, headingStyleRange: "1-1" }));
children.push(br());

// ===================== 1. WHAT IT IS =====================
children.push(H1("1.  What AetherScan is (in one paragraph)"));
children.push(P([B("AetherScan is a drone that flies itself through the inside of a building — with no GPS and no pilot — maps the space in real time, and hands back a measured 3D model. "), T("The hard part is not the flying; it is that indoors there is no GPS, so the drone has to figure out where it is purely from its own sensors, decide where to go to finish the map, and stay stable the whole time. That bundle of capabilities — sense, localize, decide, fly, reconstruct — is the product. We have built and proven it in a high-fidelity simulator, and we have fully designed the physical aircraft that runs it.")]));
children.push(callout("The one-sentence version (memorize)", [P([new TextRun({ text: "“AetherScan is autonomous indoor 3D scanning — a drone that flies itself through GPS-denied interiors and gives you the 3D model, no pilot required.”", italics: true, size: 24 })])]));

// ===================== 2. THE PITCH =====================
children.push(H1("2.  The pitch (what you say out loud)"));
children.push(H3("The 60-second version"));
children.push(plainP([B("Problem: "), T("Capturing the inside of a building in 3D today is slow and manual — someone walks around with a tripod scanner (Matterport, NavVis), or flies a piloted drone. Indoors there's no GPS, so most autonomous drones simply can't operate there.")]));
children.push(plainP([B("Product: "), T("AetherScan is a small drone that does it autonomously. You set it down, it explores the space on its own, and it produces a measured 3D reconstruction and floor plan.")]));
children.push(plainP([B("Why us / why now: "), T("The whole autonomy stack — perception, GPS-denied localization (SLAM), exploration, flight control — is built from scratch and validated end-to-end in simulation against realistic sensor noise and pose drift. The aircraft is fully designed to real, purchasable parts.")]));
children.push(plainP([B("Traction / proof: "), T("In simulation the drone autonomously covers ~88% of a real apartment floor-plan in a single ~5-minute run, flying on its own drift-corrected position estimate — and it reaches the same coverage flying “blind” on that estimate as it does with perfect god's-eye position. That's the core technical risk, retired in sim.")]));
children.push(plainP([B("The ask: "), T("Seed capital to build the first physical aircraft. Bench-first staging means ~$1,060 of parts gets us to a first autonomous indoor flight; ~$1,659 is the full bill of materials.")]));
children.push(space());
children.push(H3("The one-liner for a noisy room"));
children.push(callout("", [P([new TextRun({ text: "“It's a drone that scans the inside of buildings by itself — no GPS, no pilot. Think Matterport, but it flies and it's autonomous.”", italics: true, size: 24 })])]));

// ===================== 3. PROBLEM & WHY NOW =====================
children.push(H1("3.  The problem and why now"));
children.push(lead("The problem.", "Indoor 3D capture is a real, paid activity — construction progress tracking, facility and asset surveys, real-estate capture, insurance, inspection. Today it is done by a human walking a tripod scanner room by room, or by a skilled pilot flying a drone manually. Both are slow, need a trained operator, and don't scale to “do this every day on an active site.”"));
children.push(lead("Why drones don't already do this.", "Outdoor drones lean on GPS to know where they are. Inside a building there is no GPS. Without it, a drone has to estimate its own position from its sensors — and those estimates drift. Most autonomous drones therefore stop at the door."));
children.push(lead("Why now.", "Three things converged: cheap 360° LiDAR and depth cameras, small AI compute you can fly (NVIDIA Jetson-class), and mature open flight stacks (PX4). The missing piece is the autonomy software that ties them together for GPS-denied interiors — which is exactly what we built."));

// ===================== 4. MARKET =====================
children.push(H1("4.  The market and who pays"));
children.push(P([T("We deliberately focus on "), B("one"), T(" beachhead rather than “any indoor drone use-case.” The lead market the design is tuned for:")]));
children.push(table(["Market", "The job to be done", "Why autonomy wins"], [
  ["Construction progress", "Daily/weekly volumetric capture of an active interior build", "Repeatable, no operator, compare to BIM model"],
  ["Facility / asset survey", "Map plant rooms, warehouses, utilities", "GPS-denied, tight spaces, hands-off"],
  ["Real-estate capture", "Interior 3D tours and floor plans", "Faster and cheaper than a tripod operator"],
], [2400, 4200, 2760]));
children.push(space());
children.push(lead("Reference points (not our claims, the category's).", "Matterport and NavVis defined buyer expectations for indoor 3D capture quality and built large businesses on manual/tripod workflows. The opportunity is to deliver comparable data autonomously and faster. (Use these names to anchor the category, not to claim their revenue.)"));

// ===================== 5. ARCHITECTURE OVERVIEW =====================
children.push(br());
children.push(H1("5.  The system, end to end"));
children.push(P([T("AetherScan is two cooperating layers. Keep this split in your head — it answers half of all technical questions.")]));
children.push(table(["Layer", "What it is", "In the sim today", "On the real drone"], [
  ["High-level autonomy", "Perception, mapping, SLAM, where-to-go, what setpoint to fly", "The redwood_sim Python stack", "Runs on the Jetson Orin Nano"],
  ["Low-level flight", "Keep the aircraft stable, hit the setpoint, mix the motors", "controls.py + physics.py", "Pixhawk 6C running PX4"],
  ["The link", "How the two talk", "A WebSocket message bus", "A MAVLink serial link"],
], [2200, 2700, 2230, 2230]));
children.push(space());
children.push(callout("Why this matters for a pitch", [P([T("If a brownout or a software bug hits the autonomy brain, the flight controller "), B("keeps the aircraft stable on its own"), T(" — it never depends on the brain to stay in the air, only to know where to go. That is a safety story investors like, and it is exactly how the simulation is structured.")])]));

// ===================== 6. THE AUTONOMY STACK =====================
children.push(H1("6.  How the autonomy works (the core technology)"));
children.push(P([T("This is the heart of the company. Six pieces, in the order the drone uses them every fraction of a second: "), B("sense → map → decide → localize → fly → reconstruct."), T(" Each is explained in one breath, then in depth.")]));

children.push(H2("6.1  Sensing — the drone's eyes"));
children.push(lead("In one breath.", "A spinning 360° laser scanner (LiDAR) measures distance to every wall around the drone; a forward depth camera sees obstacles ahead; a downward sensor measures height and sideways drift; an IMU measures rotation. Together they tell the drone the shape of the room and how it's moving."));
children.push(lead("Deeper.", "The primary sensor is a 360° 2D LiDAR (RPLIDAR A2-class) — it is the one sensor our coverage results depend on. We proved a camera-only setup (87° field of view) roughly halves coverage, which is why the LiDAR is non-negotiable. The simulator models the LiDAR honestly: 168 rays per scan, range noise that grows with distance, dropouts at grazing angles — not a perfect laser. A forward Intel RealSense D435i adds a depth cone for obstacle avoidance; a downward optical-flow + time-of-flight module gives altitude and sideways velocity; the IMU (in the flight controller) measures attitude."));

children.push(H2("6.2  Mapping — building the floor plan as it flies"));
children.push(lead("In one breath.", "The drone divides the floor into a grid of 20 cm squares and, from the laser hits, marks each square as wall, free space, or unknown — building a live map of where it can and can't go."));
children.push(lead("Deeper.", "The map is a log-odds occupancy grid. “Log-odds” means each cell accumulates evidence rather than flipping on a single reading — so one noisy laser return can't punch a fake hole in a wall. A key subtlety we solved: downward laser hits used to paint the floor itself as an obstacle, trapping the drone in a tiny box; we treat floor hits as free space at flight altitude. This grid is what the planner reads."));

children.push(H2("6.3  Exploration — deciding where to go next"));
children.push(lead("In one breath.", "The drone always flies toward the boundary between “mapped” and “not-yet-seen” space — the frontier — and it plans a smart tour so it doesn't waste time walking the same corridor twice."));
children.push(lead("Deeper.", "This is frontier-based exploration combined with a coverage objective (it isn't done with a room just because it glimpsed it through a doorway — it goes in). On top, a FUEL-style global tour orders the candidate viewpoints into a short route, which measured +15–56% coverage-per-meter versus naive “go to the nearest frontier.” These are published robotics techniques (FUEL, Zhou et al. 2021), implemented and measured, not invented from scratch — a strength: it signals we know the literature."));

children.push(H2("6.4  Localization & SLAM — the hard problem, and the moat"));
children.push(callout("If you remember one technical thing, remember this", [P([T("Indoors there is no GPS. The drone must estimate its own position from its sensors, and that estimate "), B("drifts"), T(" over time. Correcting that drift — so the map and the flight stay consistent — is the single hardest problem in the product, and solving it well is the moat.")])], AMBER));
children.push(lead("In one breath.", "The drone keeps a running guess of its own position (from the IMU and motion), which slowly drifts. SLAM continuously matches the live laser scan against the map it has built and snaps the guess back into place, so the drift stays small and bounded."));
children.push(lead("Deeper.", "SLAM = Simultaneous Localization And Mapping. We use correlative scan matching (Olson, 2009): every fraction of a second the new LiDAR scan is aligned against an accumulated map and the position estimate is corrected. Measured result over a 5-minute run: with SLAM off, drift grows to ~0.33 m; with SLAM on it stays flat at 0.04–0.10 m, and the reconstruction's “ghosting” (the same wall appearing twice) drops 22%. The headline proof: when the drone navigates entirely on its own drifting-then-corrected estimate, it reaches the same coverage as when it flies on perfect god's-eye position. That means the autonomy doesn't secretly depend on information a real drone wouldn't have."));

children.push(H2("6.5  Flight control — staying stable and hitting the target"));
children.push(lead("In one breath.", "A control loop turns “go there” into precise motor speeds 500 times a second, keeping the drone level and on course."));
children.push(lead("Deeper.", "The simulator runs full 6-degree-of-freedom rigid-body physics integrated at 500 Hz (RK4), with realistic drag, ground effect, and wind. A cascaded PID controller (position → velocity → attitude → motor thrust) flies it. Measured behavior across scenes: peak tilt ~10–14°, altitude held to ~8 mm. One instructive bug we found and fixed: the original controller mapped desired direction to the wrong tilt axes, so the drone flew perpendicular to its target and trapped itself — fixing the body-frame math is what made autonomous navigation actually work."));

children.push(H2("6.6  Reconstruction & deliverables — the output the customer pays for"));
children.push(lead("In one breath.", "Every laser hit is added to a growing 3D point cloud; at the end you export a colored 3D model, a mesh, and a dimensioned floor plan."));
children.push(lead("Deeper.", "Points are de-duplicated into a 5 cm voxel cloud and streamed live to the dashboard. On export we produce a semantic-colored PLY point cloud (walls / floor / ceiling / objects), a Poisson-reconstructed GLB mesh, and an SVG floor plan with dimensions and area. That floor plan is the artifact a construction or real-estate customer actually wants."));

// reconstruction image
children.push(...img(A + "dashboard_scan_apartment1.png", 440, 248, "Live reconstruction during an autonomous run: ~88% coverage, 353K points, 58.8 m² mapped."));

// ===================== 7. WHAT'S PROVEN =====================
children.push(br());
children.push(H1("7.  What is actually proven (the numbers to cite)"));
children.push(P([T("These are real outputs of the system, reproducible on command. Memorize the headline five (§20).")]));
children.push(table(["Metric", "Result", "Why it matters"], [
  ["Coverage, single run", "~88% of a real apartment in ~5 min", "It actually finishes the map, autonomously"],
  ["Points captured", "~353,000", "Dense enough to be a real deliverable"],
  ["Area mapped", "58.8 m²", "Apartment-scale in one battery"],
  ["Estimate vs truth nav", "Coverage parity (e.g. 35.2% vs 37.0%)", "Doesn't cheat using GPS-like data"],
  ["SLAM drift bound", "0.04–0.10 m (vs 0.33 m off)", "Drift is controlled, map stays consistent"],
  ["Flight quality", "tilt ~10–14°, altitude ±8 mm", "Smooth, stable, scan-grade flight"],
  ["Scene library", "18 real Meta Replica scans", "Tested across many real layouts, not one demo"],
], [2600, 3260, 3500]));
children.push(space());
children.push(callout("How to say it honestly", [P([T("“These are simulation results — but the simulator models real sensor noise, real pose drift, and real physics, and the drone flies on the same imperfect information a real one would have. That's why we trust them as a forecast, and it's why the next dollar goes to confirming them on hardware.”")])]));

// ===================== 8. HARDWARE =====================
children.push(H1("8.  The aircraft (hardware)"));
children.push(P([T("The physical drone is fully specified to real, currently-purchasable parts, and the design was reviewed to a “deployment-ready” sign-off (§11). It has not been built yet.")]));
children.push(...img(A + "airframe_dark.png", 360, 270, "The airframe design: 360 mm quad-X with prop guards, a top LiDAR mast, a nose camera pod, and skids."));
children.push(table(["Item", "Choice", "In plain English"], [
  ["Class", "360 mm quad-X, 7″ props, guarded", "Palm-of-two-hands size, indoor-safe"],
  ["Weight / flight time", "1.45 kg, ~14–17 min", "One battery covers an apartment"],
  ["Primary sensor", "Slamtec RPLIDAR A2 (360°)", "The eyes; the validated sensor"],
  ["Also", "RealSense D435i, flow + ToF, IMU", "Forward depth, height, drift, attitude"],
  ["Autonomy compute", "NVIDIA Jetson Orin Nano 8GB", "Runs the brain on-board"],
  ["Flight controller", "Holybro Pixhawk 6C (PX4)", "Keeps it in the air"],
  ["Bill of materials", "~$1,659", "Real parts, real prices, today"],
], [2300, 3500, 3560]));
children.push(space());
children.push(lead("The LiDAR is on a mast for a reason.", "A 360° laser needs a clear horizon, so it sits on a short carbon mast above the propellers. The forward camera sits on a nose boom ahead of the props (tilted 12° down) so no propeller blade ever appears in its view. These aren't cosmetic — they're the geometry that makes the sensors usable."));

// ===================== 9. ELECTRICAL =====================
children.push(H1("9.  The electrical system (the custom board)"));
children.push(P([T("Almost everything on the drone is an off-the-shelf module. The "), B("one custom circuit board"), T(" we design is the Power & Sensor Distribution Board (PSDB). Its whole job: take the battery and produce clean, isolated power for the compute and the flight side, plus battery telemetry.")]));
children.push(...img(A + "psdb_kicad_3d.png", 380, 274, "The custom power board, rendered from the real KiCad design: two buck regulators (U1, U2), the INA226 monitor (U3), and the bulk capacitor."));
children.push(table(["Part", "What it does", "Why it's there"], [
  ["TI TPS568230 (8 A buck)", "Battery → 5 V for the Jetson", "Compute can spike to ~5 A — gets its own rail"],
  ["TI LMR33630 (3 A buck)", "Battery → 5 V for flight + sensors", "Isolated so a compute spike can't crash the autopilot"],
  ["TI INA226 + shunt", "Measures battery voltage & current", "Telemetry / low-battery return-to-home"],
], [2900, 3260, 3200]));
children.push(space());
children.push(lead("Status of the board.", "The schematic is complete and electrically verified — every connection is defined and the rules-check passes; both 5 V rails compute correctly. The remaining step is routing the copper traces, which is normal layout work in KiCad. It is not yet a finished, fabricated board."));
children.push(callout("Why “isolated rails” is worth saying", [P([T("If the AI computer briefly draws a big gulp of current, a shared power supply could sag and reset the flight controller — i.e. the drone falls. Giving the computer its "), B("own"), T(" regulator means a compute spike can never brown out the autopilot. Small design choice, big safety consequence.")])]));

// ===================== 10. HOW IT WAS ENGINEERED =====================
children.push(br());
children.push(H1("10.  How it was engineered (a credibility story)"));
children.push(P([T("Two things here make you look like a serious team, not a hobbyist. Use them when an investor probes rigor.")]));
children.push(H3("The multidisciplinary design reviews"));
children.push(P([T("The airframe wasn't sketched once — it was put through three structured design reviews with five engineering disciplines (aerospace, mechanical, electrical, software, systems) plus a business/CEO seat, each arguing the trade-offs and "), B("voting"), T(". It converged to a unanimous “deployment-ready” decision, with disagreements recorded. That process is documented in the repo.")]));
children.push(H3("The inertia finding (your favourite proof of rigor)"));
children.push(callout("Tell this story — it shows you catch problems before they cost money", [
  P([T("Our CAD model computes the real drone's "), B("rotational inertia"), T(" (how hard it is to spin) from the actual component weights and positions. It revealed that the value our flight controller had been tuned against was "), B("physically impossible"), T(" for a drone this size — you'd need half the aircraft's mass out at the propeller tips. So we corrected the simulation to the buildable reality and re-validated the controller; flight quality held. We caught a sim-to-real mismatch "), B("on paper, for free,"), T(" before bending any metal.")]),
]));

// ===================== 11. TOOLCHAIN =====================
children.push(H1("11.  Everything is defined in code (why investors should care)"));
children.push(P([T("The drone's body and its circuit board are both written as "), B("code"), T(", not clicked together in a GUI:")]));
children.push(bullet([B("The airframe "), T("is parametric CAD (build123d) — change one number, the whole model and its 3D files regenerate.")]));
children.push(bullet([B("The circuit board "), T("is code-defined (atopile) — the compiler even picks real, in-stock parts and runs electrical checks.")]));
children.push(bullet([B("A checker "), T("fails the build if the hardware ever drifts out of agreement with the simulation's assumptions.")]));
children.push(P([B("Why this matters to an investor: "), T("speed and reproducibility. Design changes are fast, versioned, and reviewable like software; nothing lives in one person's head or one fragile CAD file. It signals the project can move quickly and survive a team change.")]));

// ===================== 12. HONEST STATUS =====================
children.push(H1("12.  Honest status — done vs. not done"));
children.push(P([B("Be the one in the room who states this first. "), T("It builds more trust than any slide.")]));
children.push(table(["Area", "Status"], [
  ["Autonomy software (sense→map→SLAM→plan→fly)", "DONE — validated end-to-end in simulation"],
  ["GPS-denied navigation on own estimate", "DONE in sim — coverage parity with ground truth"],
  ["Reconstruction + export (PLY/mesh/floor plan)", "DONE in sim"],
  ["Aircraft design + bill of materials", "DONE — reviewed to deployment-ready"],
  ["Power board schematic", "DONE + electrically verified; not yet routed"],
  ["Physical drone built and flown", "NOT done — this is what the raise funds"],
  ["Global loop closure (very long missions)", "Open R&D item, scoped"],
  ["Routing the PCB + fabricating it", "Remaining build-phase work"],
], [4680, 4680]));
children.push(space());
children.push(callout("The line that turns 'it's just a sim' into a strength", [P([T("“We deliberately spent the cheap phase — software and design — proving the hardest risk, which is the autonomy, before spending money on parts. The dominant risk in a drone like this isn't the airframe, it's whether it can navigate GPS-denied. We retired that risk in simulation first. That's disciplined capital use, not a gap.”")])], MINT));

// ===================== 13. ROADMAP & ASK =====================
children.push(H1("13.  Roadmap and the ask"));
children.push(P([T("The build is staged so the first dollar buys the highest-confidence risk reduction. You can make real progress with a partial raise.")]));
children.push(table(["Stage", "What it buys", "What it proves", "Approx."], [
  ["Now", "(done) sim + design", "Autonomy + design validated", "$0"],
  ["1", "Compute + sensors on a bench", "The brain runs on real sensors", "~$530"],
  ["2", "Airframe + first autonomous scan", "It flies and scans for real", "~$530"],
  ["Scale", "Pilot deployments, v2", "Customers, revenue", "—"],
], [1500, 3500, 3060, 1300]));
children.push(space());
children.push(callout("The ask, in one line", [P([B("“We're raising seed capital to build the first aircraft. ~$1,060 of parts gets us to a first autonomous indoor flight; the full bill of materials is ~$1,659. The money buys parts, bench bring-up, and the flight validation that turns our simulation results into a flying product.”")])]));

// ===================== 14. COMPETITION & MOAT =====================
children.push(br());
children.push(H1("14.  Competition and why we win"));
children.push(table(["Who", "What they do", "Why we're different"], [
  ["Skydio", "Excellent autonomous drones, GPS/vision", "Outdoor-first, GPS-reliant; we own GPS-denied interiors"],
  ["DJI", "Market-leading hardware + ecosystem", "Pilot-flown / GPS; not autonomous indoor scanning"],
  ["Flyability (Elios)", "Caged drones for confined spaces", "Pilot-flown; we're autonomous and produce the model"],
  ["Matterport / NavVis", "Indoor 3D capture (tripod/manual)", "Manual & static; we fly it and it's autonomous"],
], [1700, 3560, 4100]));
children.push(space());
children.push(H3("The moat, stated three ways"));
children.push(num([B("The capability: "), T("GPS-denied indoor autonomy is genuinely hard — perception + drift-corrected SLAM + exploration + stable flight, all on-board. Most players need GPS or a pilot.")]));
children.push(num([B("The accumulation: "), T("the autonomy improves with every flight and every scene; the validated sim is a moat-builder (test 18 layouts before breakfast).")]));
children.push(num([B("The focus: "), T("we tune the whole stack for one buyer (indoor scanning), not a general-purpose drone — a sharper product than a big platform can justify building.")]));

// ===================== 15. GLOSSARY =====================
children.push(H1("15.  Glossary (every term, plain English)"));
const glossary = [
  ["Autonomy stack", "The software that lets the drone sense, decide and fly by itself."],
  ["GPS-denied", "Operating where GPS doesn't work (indoors). The core challenge."],
  ["SLAM", "Simultaneous Localization And Mapping — figuring out where you are while building the map, by matching live scans to the map."],
  ["Pose / pose estimate", "The drone's position and orientation; the “estimate” is its own (drifting) guess of it."],
  ["Drift", "The slow error that builds up in a position estimate with no GPS to correct it."],
  ["LiDAR", "A laser that measures distance by timing reflections; ours spins to see 360°."],
  ["Occupancy grid", "The map, as a grid of cells each marked free / wall / unknown."],
  ["Frontier exploration", "Always heading to the edge of the known map to discover new space."],
  ["Coverage", "The % of the reachable floor area the drone has mapped."],
  ["IMU", "Inertial Measurement Unit — sensors that measure rotation and acceleration."],
  ["PID / cascaded PID", "A standard control loop that turns “go there” into stable motor commands."],
  ["6-DoF physics", "Six degrees of freedom (3 move + 3 rotate) — full rigid-body flight simulation."],
  ["Point cloud", "The 3D scan, as millions of measured points in space."],
  ["Poisson mesh", "A watertight surface reconstructed from the point cloud."],
  ["Voxel", "A 3D pixel; we de-duplicate points into 5 cm voxels."],
  ["PX4 / Pixhawk", "Open-source flight control software / the board it runs on."],
  ["MAVLink", "The standard messaging protocol between the autonomy brain and the flight controller."],
  ["Jetson Orin Nano", "NVIDIA's small AI computer that runs the autonomy on-board."],
  ["Buck regulator", "A circuit that steps a higher voltage down efficiently (battery → 5 V)."],
  ["BOM", "Bill of Materials — the full parts list with prices."],
  ["TWR", "Thrust-to-Weight Ratio — how much lift vs weight (ours ~3.9×; >2 is safe)."],
  ["Replica dataset", "Meta's library of real-world scanned interiors we test against."],
];
children.push(table(["Term", "Meaning"], glossary, [2400, 6960]));

// ===================== 16. Q&A =====================
children.push(br());
children.push(H1("16.  Investor Q&A — the hard questions, and your answers"));
children.push(P([B("Drill these out loud. "), T("The format is the question, then a strong, honest answer. Don't memorize word-for-word — own the idea so you can say it your way.")]));

const QAs = [
  ["Have you actually flown a drone, or is this all simulation?",
    "Be direct: “We haven't flown a physical aircraft yet — and that's exactly what this raise funds. What we've done is build and validate the entire autonomy stack in a high-fidelity simulator that models real sensor noise, real position drift, and real physics, plus a complete, reviewed hardware design to purchasable parts. We spent the cheap phase retiring the expensive risk — the autonomy — before buying hardware.”"],
  ["What stops DJI or Skydio from doing this tomorrow?",
    "“They could enter, but their focus is elsewhere: DJI is hardware-and-ecosystem with a pilot or GPS; Skydio is outdoor-first vision autonomy. GPS-denied indoor scanning is a specific, hard niche neither prioritizes, and a focused product beats a general platform's side-feature. Our edge is a stack tuned end-to-end for this one job, plus the autonomy IP and the data flywheel we build by being first and focused.”"],
  ["Why hasn't this been solved already if the pieces exist?",
    "“The components are recent and cheap now — 360° LiDAR, flyable AI compute, mature flight stacks. The missing piece is the autonomy software that fuses them for GPS-denied interiors and tolerates real sensor drift. That integration is the hard, non-obvious work, and it's what we built.”"],
  ["What's the single hardest technical problem, and how do you know you've solved it?",
    "“Localization without GPS — the position estimate drifts, and everything downstream depends on it. We solved it with on-board SLAM that bounds the drift to under ~10 cm, and we proved it the right way: the drone flies on its own drifting-then-corrected estimate and reaches the same coverage as flying on perfect position. If it secretly needed god's-eye data, those two numbers would diverge. They don't.”"],
  ["Your simulation results could be optimistic. Why trust them?",
    "“Fair — so we engineered against optimism. The sensors in the sim are noisy, the position estimate drifts, the physics is full 6-DoF, and the drone only ever uses information a real one would have. We also found and fixed a sim-to-real inertia mismatch on paper before it could bite us. The results are a forecast we trust enough to bet hardware money on — and validating them on hardware is precisely the milestone we're funding.”"],
  ["What's the market size and who actually pays?",
    "“Our beachhead is construction progress capture and facility survey — recurring, operator-heavy work today (tripod scanners, manual drone pilots). The buyer is a contractor, facility manager, or survey firm paying for repeatable interior 3D. We anchor the category on Matterport/NavVis, who built real businesses on the manual version of this job; we automate it.”"],
  ["What's defensible here — what's the IP?",
    "“Three layers: the autonomy software itself (perception + SLAM + exploration tuned for this domain), the data and tuning flywheel that compounds with every flight and scene, and product focus. The hardware is deliberately off-the-shelf — we don't want to defend a circuit board, we want to defend the brain.”"],
  ["How much money and time to a flying product?",
    "“Bench-first: ~$530 puts the compute and sensors on a desk to confirm perception on real hardware; another ~$530 builds the airframe and gets a first autonomous scan — about $1,060 to first flight, $1,659 for the full build. The staging means even a partial raise produces a real, de-risking milestone.”"],
  ["Why indoor and GPS-denied specifically — isn't that a smaller market?",
    "“It's smaller but it's defensible and underserved precisely because it's hard. GPS-denied is where the incumbents stop. We'd rather own a hard niche than compete in the crowded GPS/outdoor space, then expand from a position of strength.”"],
  ["What if the cheaper camera-only version is good enough and undercuts you?",
    "“We tested that: a camera-only 87° field of view roughly halves coverage and destabilizes avoidance. The 360° LiDAR isn't a luxury, it's what makes reliable autonomous coverage work. If anything that's a barrier — competitors who cut the LiDAR get a worse product.”"],
  ["You're a small team / solo — can you actually build this?",
    "“The whole system already exists and is reproducible — the autonomy runs, the design is reviewed, it's all in version control and defined in code so it's not trapped in one person's head. The funding lets me bring on the airframe/flight-test help for the build phase; the hard software risk is already retired.”"],
  ["What could kill this?",
    "“Honestly: (1) sim-to-real surprises on first flights — mitigated by bench-first staging and the inertia work; (2) a focused incumbent move — mitigated by speed and the data flywheel; (3) localization breaking down on very long missions, which is our one open R&D item (global loop closure) and is scoped. I'd rather name these than pretend they don't exist.”"],
  ["What's the business model?",
    "“Hardware plus software/data subscription is the natural shape — the drone captures, but the recurring value is the processed 3D deliverables and progress comparisons. We'll validate pricing with the first pilot customers; the wedge is selling the autonomy-as-a-service, not just a drone.”"],
  ["Why should we believe the reconstruction is good enough to sell?",
    "“The output isn't a vague cloud — it's a semantic-labeled point cloud, a watertight mesh, and a dimensioned floor plan with area. The floor plan is exactly the artifact a construction or real-estate customer already pays for. The first pilots will confirm the quality bar against their current Matterport/tripod workflow.”"],
];
QAs.forEach(([q, a]) => qa(q, a).forEach((p) => children.push(p)));

children.push(space());
children.push(H3("Three traps, and how to step around them"));
children.push(bullet([B("Don't overclaim flight. "), T("Never imply you've flown. “Validated in simulation, hardware designed, not yet built” — every time.")]));
children.push(bullet([B("Don't get cornered on a giant TAM. "), T("Talk beachhead (construction/facility) and expansion, not “all drones everywhere.”")]));
children.push(bullet([B("Don't pretend there are no risks. "), T("Naming risks (and your mitigations) reads as competence, not weakness.")]));

// ===================== 17. NUMBERS TO MEMORIZE =====================
children.push(br());
children.push(H1("17.  The numbers to memorize"));
children.push(P([B("If you know nothing else cold, know these.")]));
children.push(table(["Number", "What it is"], [
  ["~88% coverage in ~5 min", "Autonomous mapping of a real apartment, one run"],
  ["Coverage parity, estimate vs truth", "Proof it navigates without GPS-like data"],
  ["< 0.10 m drift (SLAM on)", "Localization is bounded and reliable"],
  ["1.45 kg, ~15 min, 360 mm", "The aircraft: small, indoor, one-battery"],
  ["~$1,060 to first flight / $1,659 full BOM", "The ask, in parts"],
  ["18 real scenes tested", "Breadth of validation, not a single demo"],
  ["One custom board (the PSDB)", "Everything else is off-the-shelf"],
], [3400, 5960]));
children.push(space());
children.push(callout("Final reminder", [P([T("You understand this system better than almost anyone who will be in the room. Lead with the problem, prove it with the autonomy, be honest about the build stage, and ask for what it costs to fly. "), B("Clarity and honesty are your edge.")])], NAVY));

// ---------- assemble ----------
const doc = new Document({
  creator: "AetherScan",
  title: "AetherScan — Founder's Master Guide",
  styles: {
    default: { document: { run: { font: FONT, size: 22, color: "222222" } } },
    paragraphStyles: [
      { id: "Heading1", name: "Heading 1", basedOn: "Normal", next: "Normal", quickFormat: true,
        run: { size: 34, bold: true, font: FONT, color: NAVY },
        paragraph: { spacing: { before: 320, after: 160 }, outlineLevel: 0, keepNext: true,
          border: { bottom: { style: BorderStyle.SINGLE, size: 6, color: "9FB7CC", space: 6 } } } },
      { id: "Heading2", name: "Heading 2", basedOn: "Normal", next: "Normal", quickFormat: true,
        run: { size: 27, bold: true, font: FONT, color: BLUE },
        paragraph: { spacing: { before: 220, after: 110 }, outlineLevel: 1, keepNext: true } },
      { id: "Heading3", name: "Heading 3", basedOn: "Normal", next: "Normal", quickFormat: true,
        run: { size: 23, bold: true, font: FONT, color: CYAN },
        paragraph: { spacing: { before: 160, after: 80 }, outlineLevel: 2, keepNext: true } },
    ],
  },
  numbering: { config: [
    { reference: "b", levels: [{ level: 0, format: LevelFormat.BULLET, text: "•", alignment: AlignmentType.LEFT, style: { paragraph: { indent: { left: 460, hanging: 260 } } } }] },
    { reference: "n", levels: [{ level: 0, format: LevelFormat.DECIMAL, text: "%1.", alignment: AlignmentType.LEFT, style: { paragraph: { indent: { left: 460, hanging: 260 } } } }] },
  ] },
  sections: [{
    properties: { page: { size: { width: 12240, height: 15840 }, margin: { top: 1440, right: 1440, bottom: 1440, left: 1440 } } },
    footers: { default: new Footer({ children: [new Paragraph({ alignment: AlignmentType.CENTER, children: [
      new TextRun({ text: "AetherScan — Founder's Master Guide   ·   ", size: 16, color: GREY }),
      new TextRun({ children: [PageNumber.CURRENT], size: 16, color: GREY }),
    ] })] }) },
    children,
  }],
});

Packer.toBuffer(doc).then((buf) => { fs.writeFileSync("AetherScan_Founder_Guide.docx", buf); console.log("wrote AetherScan_Founder_Guide.docx", buf.length, "bytes"); });
