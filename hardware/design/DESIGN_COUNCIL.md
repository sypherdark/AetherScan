# AetherScan — Design Council

A multidisciplinary design review for the airframe. Five engineers, each owning a
discipline, argue the real tradeoffs from one shared set of constraints (the
flight-software contract in `cad/parameters.py` + the BOM electronics) and
converge. Where they disagree, the disagreement and its resolution are recorded.
Nothing here is decoration — every decision drives `cad/frame.py` and the specs.

## The council

| | Persona | Owns | Cares most about |
|---|---|---|---|
| **AERO** | Dr. Lena Voss | Aerodynamics & flight performance | clean prop inflow, control authority, the LiDAR scan geometry |
| **MECH** | Marcus Reyes | Structures & manufacturing | load paths, stiffness, mass, CG, vibration, buildability |
| **ELEC** | Dr. Aisha Khan | Power, PCB, wiring, thermal | rails, EMI, Jetson cooling, connector access |
| **SW** | Tomás Oliveira | Autonomy, controls, sim accordance | sensor FOV/placement, coordinate frames, inertia↔control |
| **SYS** | Sora Tanaka (chair) | Integration, usability, product | portability, serviceability, forcing convergence |

The shared constraints nobody is allowed to silently break:
`m=1.45 kg`, `arm=0.18 m` (360 mm diagonal), `T_max≈12 N/motor` (TWR≈3.6),
360° LiDAR is the validated sensor, D435 87° forward, GPS-denied, +X=forward,
Z-up. (All cited in `cad/parameters.py`.)

---

## Round 1 — Frame topology

**AERO opens.** "Three real options: symmetric quad-X, deadcat (stretched-X to
clear a forward camera), or hexa. Hexa buys redundancy and lift but blows the mass
and the 360 mm envelope — overkill indoors. Deadcat is tempting because it pulls
the front rotors out of the camera's view, but it makes the frame
front/back-asymmetric."

**SW objects.** "Asymmetry is the problem. The simulation models a symmetric body —
`Ixx = Iyy` in `physics.py`. A deadcat deliberately makes `Ixx ≠ Iyy` and moves
the rotor thrust lines, which changes the control allocation the autopilot was
tuned against. If we want the real drone to *be* the simulated drone, symmetric-X
is the honest choice."

**MECH agrees, adds a build reason.** "Symmetric-X means four identical arms, one
clamp design, one spare. Deadcat needs handed parts. For a project that has to be
*easy to build*, identical arms win."

**AERO concedes, with a condition.** "Fine — symmetric-X. But then the front
rotors *are* near the camera's view cone, and I won't accept blades in the depth
image. We solve the camera geometry separately (Round 4), not by bending the
frame."

> **Decision 1:** Symmetric quad-X, 360 mm diagonal, four identical arms. Camera
> clearance handled by sensor placement, not frame asymmetry. *(All agree.)*

---

## Round 2 — Where does the 360° LiDAR go?

This is the defining geometric problem of an indoor scanner.

**SW states the hard requirement.** "`REALWORLD_READINESS.md` is unambiguous: the
360° LiDAR is the validated sensor, and the 87° camera-only fallback *halves*
coverage. So the LiDAR's horizontal scan plane must have a genuinely unobstructed
360° horizon. If an arm or a prop sits in that plane, we get phantom returns
exactly where the planner needs clean frontiers."

**AERO lays out the options.** "Three placements. (a) **Top mast** — LiDAR above
the prop disc, like Emesent's Hovermap. (b) **Belly** — below the props, between
them and the ground. (c) **Cage** — wrap the whole drone in a collision-tolerant
sphere, like Flyability's Elios. The cage is the safest indoors but it's heavy and
it occludes the LiDAR with its own structure. Belly fights the landing gear and
the downward flow sensor for space, and it eats ground clearance. Top mast gives
the cleanest horizon."

**MECH pushes back hard.** "A top mast is a cantilever with a 190 g mass on the
end. At our frame's stiffness that's a vibration tuning-fork — and vibration is
poison. It couples straight into the IMU. I don't want a 190 g lollipop wobbling
above the props."

**SW escalates.** "MECH is right that it matters, and it matters *more* than he's
saying. The entire `REALWORLD_READINESS.md` audit concludes the dominant
sim-to-real risk is **state estimation** — drift in the pose feeds the map, the
SLAM, and the planner. The IMU is the front end of that estimator. If a wobbling
mast injects vibration into the IMU, we degrade the one thing the whole project is
trying to protect."

**MECH proposes the resolution.** "Then the mast is not optional-flex. Make it a
**tapered carbon frustum** — wide, stiff base at the canopy, narrow top — so its
first bending mode is well above prop-pass frequency. Short as we can get away
with. And the LiDAR mounts on a damped interface, not bolted rigid."

**AERO sets the height.** "Short, but tall enough. Props spin at z≈26 mm. The scan
plane has to clear the prop tips with margin and clear the canopy. A frustum mast
that puts the scan plane near z≈135 mm does it — comfortably above the disc, still
under a 15 cm-tall package."

**SW accepts, names the cost.** "Agreed, but note for the record: a 190 g mass at
z≈125 mm *raises* `Ixx/Iyy` and the CG. That's actually helpful for our inertia
problem (Round 6) — but it raises CG, which AERO has to live with."

**AERO.** "CG at ~17 mm above the rotor plane is fine for a slow indoor scanner.
Accepted."

> **Decision 2:** Top **tapered carbon frustum mast**, LiDAR scan plane ≈135 mm
> (clears the 26 mm prop disc with margin). LiDAR on a damped mount; mast tuned
> stiff to keep its bending mode above prop-pass and protect the IMU/estimator.
> *(MECH dissent withdrawn once damping + frustum stiffness were in.)*

---

## Round 3 — Prop guards: safety vs. the mass budget

**SYS makes the product case.** "This drone flies indoors, near people and walls,
on its own estimate. It *will* clip things. Unguarded props indoors is a
non-starter — for safety and for survivability of the mission. I want full guard
rings."

**AERO supports with data.** "Guard rings (ducted-ish) actually recover a little
static thrust at the tips, and crucially they make wall contact a bump instead of
a crash — that's the Flyability lesson. I'm for them."

**MECH raises the bill.** "Four 7" rings in carbon are ~50–65 g. That pushes AUW
from ~1450 to ~1500+. We blow the `m=1.45 kg` contract."

**ELEC and SW both flag the knock-on.** SW: "AUW over 1.45 breaks accordance with
the sim mass." ELEC: "and heavier means more hover current, less endurance on the
same pack."

**SYS brokers it.** "Two-part resolution. One: the guards are **removable** — bolt
to the motor bosses. The *nominal certified config* is guarded; a stripped config
exists for max-endurance open-area flights. Two: we hit the 1.45 kg contract in
the guarded config by trimming the pack — a 4S 5200 mAh instead of 6000 mAh saves
~70 g. We trade ~2 minutes of endurance for the guards and stay on contract."

**AERO + MECH + SW accept.** SW adds: "And whichever pack we standardize on, the
BOM, the mass budget, and `parameters.py` all state the same number. No drift."

> **Decision 3:** Removable full prop-guard rings. Nominal guarded config hits
> 1.45 kg via a 5200 mAh pack; document both configs. *(All agree.)*

---

## Round 4 — The forward depth camera

**AERO returns to the debt from Round 1.** "Symmetric-X put rotors near the
camera's 87° cone. I will not accept blades in the depth frame — they create false
near-obstacles that the avoidance layer reacts to."

**SW specifies what the software needs.** "The D435 feeds forward obstacle
avoidance and floor sensing while moving. Two requirements: (1) no rotor in the
87° HFOV, and (2) it should see both ahead *and* the floor in front for safe
forward flight."

**MECH offers the mechanism.** "Put it on a **short nose boom** so the lens sits
*ahead* of the front rotor disc — then the rotors are behind the FOV origin, out
of frame. And tilt the pod **~12° down** so the cone covers ahead-and-floor."

**AERO checks it.** "Boom ahead of the disc plus 12° down-tilt — yes, that keeps
blades out and gives the floor coverage. Approved."

**SW adds the frame note.** "Mount it so the optical axis is +X with a known 12°
pitch offset, and write that offset into the extrinsics. The sim assumes the
camera looks along +X; the 12° has to live in the calibration, not as a surprise."

> **Decision 4:** D435 on a short nose boom ahead of the prop disc, 12° down-tilt,
> optical axis +X, tilt recorded in extrinsics. *(All agree.)*

---

## Round 5 — Electrical: power, EMI, thermal

**ELEC takes the floor.** "Three things I won't compromise. **One: rail isolation.**
The Jetson can spike ~5 A at 5 V. It gets its *own* 8 A BEC; the flight controller
and sensors get a separate 3 A rail. A compute transient must never brown out the
autopilot. **Two: EMI.** The D435 is USB3 — a notorious 2.4 GHz noise source — and
the RPLIDAR link and the IMU are sensitive. I route USB3 away from the LiDAR
cable and the FC, and I star-ground everything at the distribution board.
**Three: thermal.** The Jetson under load needs airflow."

**MECH on thermal placement.** "Mount the Jetson high in the canopy with its
heatsink in the rotor downwash channel — free forced convection. But that fights
CG; the Jetson is 150 g up high."

**AERO.** "150 g at mid-canopy height is fine; it's the LiDAR up top that drives CG,
not the Jetson. Put the heatsink in the downwash; I'll take the cooling."

**SW connects it to the architecture.** "The split is exactly the simulation's
architecture. The **Jetson runs the `redwood_sim` autonomy** — perception, mapping,
SLAM, planning. The **Pixhawk/PX4 runs the low-level control** — the real
counterpart of `controls.py`/`physics.py`. The **MAVLink UART between them is the
WebSocket bridge** (`bridge/server.py`) in hardware. If ELEC keeps that link clean
and isolated, the hardware *is* the sim's block diagram."

**ELEC.** "Then the distribution board carries: XT60 in, dual isolated 5 V bucks,
the MAVLink UART, and labelled sensor breakouts. That's the custom PCB — the
`electrical/` deliverable, requirements E1–E8."

> **Decision 5:** Custom Power & Sensor Distribution Board: isolated 8 A (Jetson) +
> 3 A (avionics) rails, star ground, USB3 routed clear of LiDAR/FC, Jetson
> heatsink in downwash. Jetson=autonomy, PX4=flight, MAVLink=the sim bridge.
> *(All agree.)*

---

## Round 6 — The inertia reckoning (the hardest round)

**SW brings the measurement.** "We have to talk about `cad/assembly.py`. I placed
every real component at its mount point with its datasheet mass and computed the
inertia tensor. The buildable drone has **Ixx≈0.0074, Iyy≈0.0095, Izz≈0.011
kg·m²**. The simulation assumes **0.014, 0.014, 0.026**. We're 30–57% low,
worst in yaw."

**MECH, blunt.** "So which is wrong — my structure or your sim?"

**AERO does the physics.** "The sim is. Hard ceiling: put *all* 1.5 kg on the
180 mm rim and Izz maxes at `1.5×0.18² ≈ 0.049`. To reach 0.026 you'd need ~half
the drone's mass living at the rim — that's a ring, not a scanner. Real quads
concentrate mass centrally; realistic Izz here is ~0.011. **0.026 was never a
buildable target.**"

**SW accepts the diagnosis, states the consequence.** "Then the real drone will be
*more* agile than the sim — about 2.3× quicker in yaw for the same torque. Gains
tuned in sim would be too hot on hardware: yaw oscillation, twitchy roll/pitch.
This is precisely the bounded sim-to-real delta the project exists to find — and
we found it on paper, before bending metal."

**MECH offers to help from the structure side.** "I can claw a little back — the
top-mast LiDAR and the outboard guard rings both add rim/height inertia. But not
2.3×. I'm not adding 700 g of rim ballast; that wrecks the mass budget and TWR."

**AERO.** "Correct — don't. Optimizing the airframe toward a wrong number is
backwards."

**SW's resolution.** "The sim moves to the hardware, not the reverse. Update
`physics.py` to `Ixx≈0.0075, Iyy≈0.011, Izz≈0.012`, mirror into
`cad/parameters.py` so the checker stays green, then **re-tune and re-validate the
attitude/rate loops**. That's a real controls task with consequences — it touches
gains that were extensively tuned — so it's a *decision to schedule*, not a
silent edit."

**SYS records it as a gated decision.** "Logged as the top open item. We do not
flip the constant casually; we flip it, then re-run the controller validation
suite."

> **Decision 6:** The CAD-derived inertia is authoritative. Reconcile the
> **simulation** down to it (≈0.0075 / 0.011 / 0.012) and re-tune/re-validate the
> controller — as a scheduled, gated change, not an auto-edit. Full analysis in
> `specs/inertia-findings.md`. *(Unanimous on direction; execution gated.)*

---

## Round 7 — Portability & serviceability

**SYS drives.** "Requirement from the top: *easy to carry around and to use.* Right
now it's a 455 mm-span object with a fragile mast. Make it travel."

**MECH delivers, guards the one sacred dimension.** "**Folding arms** — hinge at
the canopy, swing the four arms back to roughly halve the footprint. But the hinge
has a **hard stop at exactly arm=0.18 m deployed**; that dimension is the
software contract and the moment arm. Folded for transport, latched for flight,
repeatable to the millimetre. The **LiDAR mast is tool-less removable** (it's the
tall fragile bit), and the **battery is a belly quick-release** for fast swaps."

**ELEC.** "Folding arms means the motor wires flex at the hinge every fold — strain
relief and a service loop at each joint, or they fatigue and break."

**AERO.** "Folding can't change the deployed geometry or it changes the
aerodynamics and the control moment arm. As long as the hard stop is exact and
repeatable, I'm fine."

**SW.** "Same line: deployed arm length must be exactly 0.18 m every time, or the
controller's moment arm assumption drifts. With a hard latched stop, approved."

> **Decision 7:** Folding arms with an exact, latched 0.18 m hard stop;
> tool-less removable LiDAR mast; belly quick-release battery; strain-relieved
> motor leads at the hinges. *(All agree.)* *(Hinge mechanism is a follow-up CAD
> task; current model is the deployed, fixed-arm form.)*

---

## Consensus design (what `cad/frame.py` now builds)

| Decision | Resolution |
|---|---|
| Topology | Symmetric quad-X, 360 mm, four identical arms |
| LiDAR | Top tapered-frustum mast, scan plane ≈135 mm, damped mount |
| Prop guards | Removable full rings; 1.45 kg met via 5200 mAh pack |
| Depth camera | Nose boom ahead of disc, 12° down-tilt, axis +X |
| Electrical | Isolated dual 5 V rails, star ground, USB3 kept clear, downwash cooling |
| Inertia | Reconcile the **sim** down to CAD values + re-tune (gated) |
| Portability | Folding arms (exact 0.18 m stop), removable mast, QR battery |

## Open items / standing dissents
1. **[GATED] Inertia reconciliation + controller re-tune** (Decision 6) — top
   priority; touches validated gains.
2. **Folding-arm hinge** (Decision 7) — mechanism not yet modelled; current CAD is
   the deployed fixed-arm form.
3. **Mast vibration** (Decision 2) — frustum stiffness + damped LiDAR mount is the
   plan; needs a modal check (or a bench tap-test) before flight.
4. **Guarded-config endurance** (Decision 3) — ~2 min trade vs. 6000 mAh; confirm
   acceptable for target room sizes.

> Method note: this council is run inline so every persona argues from one shared
> constraint set (`cad/parameters.py` + the BOM). That is *how* accordance is
> enforced — the disciplines converge on one number each, and the CAD, the specs,
> and the simulation all read from it.
