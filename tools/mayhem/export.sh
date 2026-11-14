#!/usr/bin/env bash
# Regenerate src/main/include/commands/MayhemProject.h from robot.mayhem.json with Mayhem's own
# exporter, at the same Mayhem commit as the vendored headers (docs/mayhem/vendor-snapshot.json).
#
#   tools/mayhem/export.sh          # write the header
#   tools/mayhem/export.sh --check  # exit 1 if the header or vendored headers are stale (CI)
#
# Needs Node 24+ and git. Set MAYHEM_DIR to use an existing Mayhem checkout instead of a clone.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
COMMIT="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["commit"])' "$ROOT/docs/mayhem/vendor-snapshot.json")"
if [[ -z "${MAYHEM_DIR:-}" ]]; then
  MAYHEM_DIR="$ROOT/build/mayhem-tools"
  if [[ "$(git -C "$MAYHEM_DIR" rev-parse HEAD 2>/dev/null)" != "$COMMIT" ]]; then
    rm -rf "$MAYHEM_DIR"
    git init -q "$MAYHEM_DIR"
    git -C "$MAYHEM_DIR" fetch -q --depth 1 https://github.com/ryanabraham1/Mayhem.git "$COMMIT"
    git -C "$MAYHEM_DIR" checkout -q FETCH_HEAD
  fi
fi
HEADER=src/main/include/commands/MayhemProject.h
cd "$ROOT"
if [[ "${1:-}" == "--check" ]]; then
  node "$MAYHEM_DIR/scripts/mayhem.mjs" validate robot.mayhem.json
  node "$MAYHEM_DIR/scripts/mayhem.mjs" cpp robot.mayhem.json --check "$HEADER"
  node "$MAYHEM_DIR/scripts/mayhem.mjs" vendor "$ROOT" --snapshot docs/mayhem/vendor-snapshot.json --check
else
  node "$MAYHEM_DIR/scripts/mayhem.mjs" cpp robot.mayhem.json --out "$HEADER"
fi
