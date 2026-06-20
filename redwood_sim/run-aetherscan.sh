#!/bin/bash
# Convenience wrapper — script lives in project root.
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
exec "$ROOT/run-aetherscan.sh" "$@"
