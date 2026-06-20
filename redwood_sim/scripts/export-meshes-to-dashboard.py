#!/usr/bin/env python3
"""Wrapper — runs repo-root export script (copy authentic PLY from cache)."""

from __future__ import annotations

import runpy
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]
runpy.run_path(str(_ROOT / "scripts" / "export-meshes-to-dashboard.py"), run_name="__main__")
