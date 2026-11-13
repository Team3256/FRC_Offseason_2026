#!/usr/bin/env bash
# Monte Carlo autonomous runs on the physics simulation: hundreds of seeded runs of one auto, each
# on a randomly perturbed robot (mass, tread friction, battery, start pose, camera noise, shot
# speed), summarized by how they fail. Each run writes a .wpilog to open in AdvantageScope.
#
#   tools/sim/montecarlo.sh "Depot Trench Sweep" [--red] [--runs 200] [--jobs 10] [--out sim-runs]
#
# Uses the desktop test binary (MonteCarlo.Run in src/test/cpp/AutoPhysicsTest.cpp) and
# tools/sim/montecarlo.py (from Mayhem's scripts/sim).
set -euo pipefail
cd "$(dirname "$0")/../.."
AUTO="${1:?usage: tools/sim/montecarlo.sh \"<auto name>\" [--red] [montecarlo.py options]}"
shift
RED=""
ARGS=()
for a in "$@"; do
  if [[ "$a" == "--red" ]]; then RED=1; else ARGS+=("$a"); fi
done

./gradlew installFrcUserProgramTestOsxuniversalReleaseGoogleTestExe -q
OUT="sim-runs/$(echo "$AUTO" | tr -cd '[:alnum:]')${RED:+-red}"
# Run a private copy, so a rebuild during the runs cannot swap the binary out from under them.
mkdir -p "$OUT"
rm -rf "$OUT/bin"
cp -R build/install/frcUserProgramTest/osxuniversal/release "$OUT/bin"
BIN="$PWD/$OUT/bin/frcUserProgramTest"
# macOS can SIGKILL a binary relinked in place; an ad-hoc signature fixes it.
if [[ "$(uname)" == "Darwin" ]]; then
  codesign --force -s - "$PWD/$OUT/bin/lib/frcUserProgramTest" 2>/dev/null || true
fi

export MAYHEM_SIM_AUTO="$AUTO"
if [[ -n "$RED" ]]; then export MAYHEM_SIM_RED=1; fi
# montecarlo.py needs Python 3.10+ (uv provides one if the system Python is older).
PY=(python3)
if ! python3 -c 'import sys; sys.exit(sys.version_info < (3, 10))' 2>/dev/null; then
  PY=(uv run -q --python 3.12)
fi
exec "${PY[@]}" tools/sim/montecarlo.py --cmd "$BIN --gtest_filter=MonteCarlo.Run" --out "$OUT" \
  --worst-metric scored "${ARGS[@]+"${ARGS[@]}"}"
