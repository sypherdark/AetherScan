#!/bin/bash
# Start the AetherScan web dashboard (local dev)
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
"$ROOT/run-aetherscan.sh" --dashboard
