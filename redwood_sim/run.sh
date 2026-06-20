#!/bin/bash
# Prefer project .venv (setup-macos.sh) or Homebrew Python on Apple Silicon.
set -euo pipefail
cd "$(dirname "$0")"

pick_python() {
  local candidates=(
    "./.venv/bin/python"
    "/opt/homebrew/opt/python@3.12/bin/python3.12"
    "/opt/homebrew/opt/python@3.11/bin/python3.11"
    "/opt/homebrew/bin/python3.12"
    "/opt/homebrew/bin/python3.11"
    "/opt/homebrew/bin/python3"
    "python3"
  )
  for c in "${candidates[@]}"; do
    if [ -x "$c" ] 2>/dev/null || command -v "$c" &>/dev/null; then
      if [ -x "$c" ]; then
        echo "$c"
      else
        command -v "$c"
      fi
      return 0
    fi
  done
  return 1
}

PY="$(pick_python)" || {
  echo "Python 3 not found. Run: ./setup-macos.sh"
  exit 1
}

warn_clt_python() {
  case "$PY" in
    /usr/bin/python3)
      echo "WARNING: Apple's Command Line Tools Python 3.9 is unreliable with Open3D on ARM Macs."
      echo "         Segfaults in paint_uniform_color / create_coordinate_frame are common."
      echo "         Fix: ./setup-macos.sh   (Homebrew Python 3.11 + .venv)"
      echo ""
      ;;
  esac
}

# Warn if using CLT Python without venv
if [ "$PY" = "/usr/bin/python3" ] && [ ! -d ".venv" ]; then
  warn_clt_python
fi

echo "Using: $("$PY" --version) ($PY)"

if [ -d ".venv" ] && [ "$PY" != "$(cd .venv/bin && pwd)/python" ]; then
  echo "Note: .venv exists but was not selected — run: source .venv/bin/activate"
fi

"$PY" -m pip install -r requirements.txt
if [ $# -eq 0 ]; then
  echo "Unified UI: use ../run-aetherscan.sh --dashboard (physics in browser, no Open3D window)."
  echo "Or headless bridge only: $PY -m bridge --scene apartment"
  echo ""
fi
exec "$PY" main.py "$@"
