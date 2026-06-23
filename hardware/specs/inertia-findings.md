# Inertia Findings — a real sim↔hardware discrepancy  ✅ RESOLVED 2026-06-23

> **Resolution (Design Review 3):** the simulation's inertia was reconciled to the
> CAD-derived values (`physics.py`: Ixx=0.0075, Iyy=0.0095, Izz=0.011) and the
> attitude controller was re-validated. The existing gains were **retained** —
> on the lower inertia, fixed gains give a faster *and* better-damped loop, and
> `_verify_flight.py` confirms the flight envelope is unchanged (|roll|≈10°,
> |pitch|≈12-14°, Z-std 8 mm; 8 unit tests green; CAD contract check green). The
> analysis below is kept for the record.

---


`cad/assembly.py` places every real component at its mount point with its
datasheet mass and computes the assembled inertia tensor. This is the most
rigorous "is the hardware in accordance with the software" check we have — and it
surfaced a genuine discrepancy worth a decision.

## Result (streamlined build, CG on the thrust centre)

| Axis | Buildable (CAD) | Sim assumes (`physics.py`) | Δ |
|---|---:|---:|---:|
| Ixx | 0.0074 kg·m² | 0.014 | **−47%** |
| Iyy | 0.0095 kg·m² | 0.014 | **−32%** |
| Izz | 0.0111 kg·m² | 0.026 | **−57%** |

Total mass and CG are fine (CG within 2 mm of centre). The **inertia is the
problem**: the drone we can actually build has roughly half the rotational inertia
the flight controller was tuned against — and the result is robust across design
revisions (the streamlined frame with a higher top-mast LiDAR moved the numbers by
only a few percent; the −57% yaw gap persists). Reproduce with `make inertia`.

## Why this happens (and why the sim's numbers are the suspect ones)

A real quad concentrates its heavy mass — battery, Jetson, LiDAR — within ~100 mm
of the centre. Only the motors, guards, and arm ends (~290 g, ~20% of mass) sit
out near the 180 mm rim. The hard upper bound on Izz, with **all** 1.476 kg jammed
onto the motor radius, is `1.476 × 0.18² = 0.048 kg·m²`. Reaching the sim's
**0.026** would require ~54% of the entire drone's mass to live at the rim — which
is not a buildable indoor scanner, it's a ring. **So 0.026 is not a target we can
hit; it's a value the sim should not have assumed.** The realistic Izz for this
airframe is ~0.012.

`Ixx ≠ Iyy` (0.0073 vs 0.0112) is also real and expected: the forward D435 and the
X-aligned battery add inertia about the Y axis but not the X axis. The sim's
symmetric `Ixx = Iyy` is an idealization; the diagonal-tensor physics model
already supports unequal values, so we can represent the truth.

## What this means for flight

Lower inertia ⇒ faster angular acceleration for the same torque ⇒ the **real
drone will be more agile (and more twitchy) than the simulation**, most of all in
yaw (−54%). Attitude/rate-loop gains tuned in sim would be effectively too hot on
hardware — risk of oscillation, especially yaw. This is exactly the bounded
sim-to-real delta the project exists to find *before* flying.

## Recommended reconciliation (a decision, not an auto-change)

Update the **simulation** to the buildable inertia, then re-tune + re-validate —
because the CAD-derived numbers are the physically trustworthy ones:

```python
# redwood_sim/core/physics.py : QuadcopterParams
Ixx: float = 0.0075   # was 0.014   (CAD: 0.0073)
Iyy: float = 0.011    # was 0.014   (CAD: 0.0112)
Izz: float = 0.012    # was 0.026   (CAD: 0.0120)
```

Then mirror them in `cad/parameters.py` (IXX/IYY/IZZ) so
`check_against_software.py` stays green, and re-run the controller validation
(the attitude PID gains will likely need to come down, per the memory of prior
tuning work).

> **Not done automatically:** this changes a constant the controller was
> extensively tuned around (see project memory), so it needs a deliberate
> re-tune + re-validation pass. Flagged here for that decision.

## Alternative (not recommended)
Add ~700 g of rim mass to raise hardware inertia toward the sim. This wrecks the
mass budget, the TWR, and the endurance — it optimizes the wrong direction. The
sim should move to the hardware, not the reverse.
