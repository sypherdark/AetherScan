"""
Guardrail: assert the CAD contract still matches the flight software.

The airframe was sized and the controller tuned against specific constants in
redwood_sim. If someone edits physics.py (e.g. changes the arm length or mass)
without updating the hardware, the drone we build no longer matches the drone we
simulated. This script parses the *actual* source files and fails loudly on any
divergence. Wire it into CI and run it before every CAD export.

Run:  python hardware/cad/check_against_software.py
"""

from __future__ import annotations

import ast
import sys
from pathlib import Path

import parameters as P

ROOT = Path(__file__).resolve().parents[2]
PHYSICS = ROOT / "redwood_sim" / "core" / "physics.py"
CONFIG = ROOT / "redwood_sim" / "config.py"


def _class_defaults(path: Path, class_name: str) -> dict[str, float]:
    """Pull `name: type = literal` defaults from a dataclass without importing."""
    tree = ast.parse(path.read_text())
    out: dict[str, float] = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == class_name:
            for stmt in node.body:
                if isinstance(stmt, ast.AnnAssign) and stmt.value is not None:
                    if isinstance(stmt.target, ast.Name) and isinstance(
                        stmt.value, ast.Constant
                    ):
                        v = stmt.value.value
                        if isinstance(v, (int, float)):
                            out[stmt.target.id] = float(v)
    return out


def main() -> int:
    phys = _class_defaults(PHYSICS, "QuadcopterParams")
    cfg = _class_defaults(CONFIG, "SimConfig")

    checks = [
        ("mass",          phys.get("mass"),        P.MASS_KG),
        ("arm_length",    phys.get("arm_length"),  P.ARM_LENGTH_M),
        ("Ixx",           phys.get("Ixx"),         P.IXX),
        ("Iyy",           phys.get("Iyy"),         P.IYY),
        ("Izz",           phys.get("Izz"),         P.IZZ),
        ("max_tilt_rad",  phys.get("max_tilt_rad"), P.MAX_TILT_RAD),
        ("drone_body_radius", cfg.get("drone_body_radius"), P.BODY_RADIUS_M),
    ]

    failed = []
    for name, software_val, cad_val in checks:
        ok = software_val is not None and abs(software_val - cad_val) < 1e-9
        flag = "ok " if ok else "FAIL"
        print(f"  [{flag}] {name:18s} software={software_val!s:>8}  cad={cad_val!s:>8}")
        if not ok:
            failed.append(name)

    if failed:
        print(f"\n✗ {len(failed)} mismatch(es): {', '.join(failed)}")
        print("  The hardware no longer matches the simulated drone.")
        print("  Reconcile parameters.py with redwood_sim before exporting CAD.")
        return 1
    print("\n✓ CAD contract matches the flight software.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
