#!/bin/bash
# Recommended setup for Apple Silicon: Homebrew Python 3.11+ in a local venv.
# Avoids /usr/bin/python3 (Apple CLT 3.9) which has known Open3D extension issues.
set -euo pipefail
cd "$(dirname "$0")"

if ! command -v brew &>/dev/null; then
  echo "Homebrew is required: https://brew.sh"
  exit 1
fi

echo "==> Installing Homebrew Python 3.11 (if needed)..."
brew install python@3.11

PY="$(brew --prefix python@3.11)/bin/python3.11"
echo "==> Using: $($PY --version) ($PY)"

echo "==> Creating virtualenv at .venv ..."
"$PY" -m venv .venv
source .venv/bin/activate

pip install --upgrade pip wheel
pip install -r requirements.txt

echo ""
echo "Setup complete. Run the sim with:"
echo "  ./run.sh --scene apartment"
echo ""
echo "Or activate the venv manually:"
echo "  source .venv/bin/activate"
echo "  python main_sim.py --scene apartment"
