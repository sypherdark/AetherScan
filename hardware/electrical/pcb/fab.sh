#!/bin/bash
# PSDB fab outputs via KiCad's real engine (run after `ato build` + place_board.py).
#   ./fab.sh render | drc | gerbers | all
set -euo pipefail
CLI="/Applications/KiCad.app/Contents/MacOS/kicad-cli"
HERE="$(cd "$(dirname "$0")" && pwd)"
PCB="$HERE/elec/layout/psdb/psdb.kicad_pcb"
OUT="$HERE/fab"; mkdir -p "$OUT"

render() {
  echo "==> 3D render"
  "$CLI" pcb render --side top  --quality high --width 1800 --height 1300 \
     --background opaque -o "$OUT/psdb_top.png" "$PCB"
  "$CLI" pcb render --side front --quality high --width 1800 --height 1000 \
     --background opaque -o "$OUT/psdb_persp.png" "$PCB" --perspective || true
}
drc() {
  echo "==> DRC"
  "$CLI" pcb drc --severity-error --severity-warning -o "$OUT/drc.json" --format json "$PCB" || true
}
gerbers() {
  echo "==> Gerbers + drill (JLCPCB)"
  "$CLI" pcb export gerbers -o "$OUT/gerbers/" "$PCB"
  "$CLI" pcb export drill   -o "$OUT/gerbers/" "$PCB"
  (cd "$OUT" && zip -qr psdb_gerbers.zip gerbers && echo "  -> $OUT/psdb_gerbers.zip")
}
case "${1:-all}" in
  render) render ;;
  drc) drc ;;
  gerbers) gerbers ;;
  all) render; drc; gerbers ;;
esac
echo "done -> $OUT"
