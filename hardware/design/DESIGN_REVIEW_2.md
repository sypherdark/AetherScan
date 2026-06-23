# AetherScan — Design Review 2 (with CEO + live research)

A second council pass, now with a **CEO** seat for the business/big-picture view,
plus a binding **vote**. Unlike Review 1 (reasoned inline), each persona here was
run as an **independent agent on Google `gemini-2.5-flash` with Search grounding**
(via the provided API key) so they could research competitors and real numbers and
argue from their own knowledge. Their raw outputs were kept outside the repo; this
file is the synthesis. A final **quantitative deep-analysis** (my own, with the
math) judges whether the airframe is actually good or just looks good.

> Method: 6 grounded model calls, one per persona, same shared brief + ballot.
> Votes are the personas'; the engineering verdict at the bottom is computed.

---

## CEO seat — the business frame (Alex Venture)

> "Technically impressive, but it's a hammer in search of a nail. Our moat is the
> **validated GPS-denied autonomy stack**, not the airframe. Stop optimizing parts
> and define the **one objective**: who buys this and for what single job."

The CEO's core challenge: we're drifting toward feature-sprawl (removable guards,
folding arms, two battery options) without a customer. Against the field —
**Skydio 3D Scan** (autonomy, pricey), **DJI** (ecosystem/reliability),
**Flyability Elios 3** (caged, confined-space, ~10–15 min), **Matterport / NavVis**
(set the output-quality bar) — a general-purpose dev drone loses. A *focused*
indoor scanner whose differentiator is "**flies itself through GPS-denied interiors
and hands you a reconstruction**" can win a niche. Recommended objective lanes
(business decision, owner = you): **construction-progress capture**, **facility /
asset survey**, or **real-estate-grade interior capture**. Pick one; let it veto
every feature.

---

## What each discipline said (grounded, condensed)

**AERO — Dr. Aris Thorne.** Streamlined canopy + round arms are right; low indoor
airspeed makes parasitic drag the main efficiency lever. **Prop guards are the real
aero cost** — they block inflow and cut propulsive efficiency, so the
guards-vs-endurance call needs a *quantified* trade, not a default. Lower-than-sim
inertia → "possibly *too* agile for precise scanning" until re-tuned. Wants
guard/prop co-optimization (airfoil-section rings) and CFD on the mast + nose boom.

**MECH — Alex Volkov.** Canopy, carbon arms, M3 system, raised LiDAR all sound.
Two worries: (1) **mast vibration into the IMU** — demands FEA / a wider or
triangulated base; (2) "removable" guards risk being flimsy or a bad load path —
make them an *integral* structural ring if they're in the 1.45 kg config. Folding
arms: mass + complexity likely not worth it for v1.

**ELEC — "Volt".** Biggest risk is **EMI: USB3 (D435, 5 Gbps) next to the RPLIDAR
and IMU** — mandates shielding, common-mode chokes/ferrites, twisted/shielded
pairs, a real grounding plan. Wants low-noise regulators with good ripple
rejection, on-board V/I telemetry for endurance + failsafes, and **CFD-verified
Jetson airflow** (throttling kills the autonomy if it cooks in the canopy).

**SW — autonomy lead.** Endorses LiDAR-first, the 135 mm scan plane, and the 12°
D435. The **inertia mismatch is "a fundamental mismatch that must be addressed
before any further hardware validation"** — un-reconciled, the tuned controller
gives poor tracking / instability and invalidates the sim's autonomy results. Flags
the nose-boom D435 as an inertia + collision liability; wants guard occlusion of
the depth FOV checked in the perception stack.

**SYS — integration/product.** "13–15 min with guards is **borderline** for a
scanner" — quantify the guard→endurance trade; maybe ship guard-less as an expert
config or a big battery as an accessory. Inertia reconciliation is **non-negotiable
and first**. Defer folding arms. Test the mast-LiDAR for real-world vibration /
prop interference in parallel with the inertia work.

---

## The vote

| # | Proposal | YES | NO | ABSTAIN | Outcome |
|---|---|:--:|:--:|:--:|---|
| P1 | Focus as an indoor-scanning **product** | 6 | 0 | 0 | **PASS** (unanimous) |
| P2 | Inertia reconcile + controller re-tune **first** | 6 | 0 | 0 | **PASS** (unanimous) |
| P3 | Keep removable guards + 5200 mAh @ 1.45 kg | 0 | 1 | 5 | **No mandate** → trade study |
| P4 | Folding arms + QR battery **now** | 0 | 5 | 1 | **REJECTED** → defer to v2 |
| P5 | Keep RPLIDAR **A2** | 6 | 0 | 0 | **PASS** |
| P6 | Commit to **360° top-mast** architecture | 6 | 0 | 0 | **PASS** |

**Reading it:** the council is unanimous on the strategy (focus the product, keep
the validated sensor + architecture) and unanimous that **inertia reconciliation is
the gating first action**. It explicitly **rejects folding arms for v1** (overturning
Review-1 Decision 7 — portability is deferred). It **withholds a mandate on guards**:
nobody defended the 5200 mAh-to-hit-1.45 kg compromise; they want it decided by a
quantified endurance trade tied to the chosen customer.

---

## Quantitative deep-analysis — is the airframe good or shit?

Computed from the actual geometry/components (not vibes). Script:
`/tmp/aetherscan_council/analysis.py` (kept out of the repo).

| Check | Result | Verdict |
|---|---|---|
| Thrust-to-weight | **3.89** (4×1.45 kgf / 1.49 kg) | ✅ healthy |
| Hover throttle | **26%** | ⚠️ motors **oversized** — efficient & lots of authority, but mass could be trimmed |
| Disk loading | **147 N/m² (15 kg/m²)** | ✅ moderate, normal for 7" |
| Hover power | ideal 113 W → real (FM 0.62, guards) **~204 W** w/ avionics | ✅ realistic |
| Endurance (hover) | **~18 min** @5200, **~21 min** @6000 (×~0.8 maneuvering ⇒ 14–17 min) | ✅ competitive (Elios 3 ≈ 10–15 min) |
| Adjacent guard-ring gap | **55 mm** | ✅ no interference |
| Prop-tip → guard inner | **5 mm** | ✅ acceptable (typical 5–10 mm) |
| LiDAR scan plane | z=135 mm; only the thin central mast is below it | ✅ genuinely clear 360° horizon |
| **Mast first bending mode** | **~745 Hz** (carbon tube) vs blade-pass **~90–180 Hz** | ✅ **~4–8× margin — MECH's vibration fear is quantitatively unfounded for a real carbon tube** (verify w/ tap-test; a flexible *printed* mast would not pass) |
| **Yaw agility** | real Izz 0.011 vs sim 0.026 → **2.36× more responsive** | ❌ **the one real must-fix** |

**Verdict: the airframe is fundamentally sound, not shit.** Geometry, clearances,
thrust margin, the scan-plane architecture, and even the much-debated mast
vibration all check out under real numbers. It has **one genuine defect — the
control model (inertia) doesn't match the buildable reality** — and **two
optimization opportunities**: motors are oversized (hover at 26%), and the
guard↔endurance trade is unquantified. None of these is a redesign; they're a tune,
a trim, and a trade study.

---

## Decisions out of Review 2

1. **[DO FIRST] Inertia reconciliation + controller re-tune.** Unanimous (P2), and
   every discipline named it the gate. Update `physics.py` + `parameters.py` to the
   CAD-derived inertia and re-validate the controller. (`specs/inertia-findings.md`.)
2. **[STRATEGY] Pick the single target customer/objective** (CEO/P1) and let it veto
   features. Lanes: construction-progress · facility survey · real-estate capture.
3. **[TRADE STUDY] Guards vs endurance vs 1.45 kg** (P3, no mandate). Quantify
   endurance delta and decide per the chosen customer; consider guard-less "expert"
   config or big-battery accessory.
4. **[DEFER] Folding arms / quick-release** to v2 (P4 rejected). Remove from the v1
   critical path; current fixed-arm CAD stands.
5. **[KEEP] RPLIDAR A2 + 360° top-mast** (P5, P6 unanimous) — the validated core.
6. **[ENGINEERING follow-ups]** EMI plan (USB3 vs LiDAR/IMU: shielding, chokes,
   grounding), CFD-verified Jetson cooling, on-board V/I telemetry + failsafes,
   bench tap-test of the mast, integral vs add-on guard structure.

> Personas: generated live on `gemini-2.5-flash` (Search-grounded) from one shared
> brief; engineering verdict computed independently. The point stands — one shared
> constraint set is what keeps aero, mech, electrical, software, business, and the
> simulation in accordance.
