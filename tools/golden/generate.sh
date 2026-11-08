#!/usr/bin/env bash
# Regenerates src/test/resources/golden/*.csv from the ORIGINAL Java implementation:
#   - MayhemLib-java (the exact version in the old vendordeps/MayhemLib.json)
#   - the robot's Java TrenchAssist, ShotCalculator and FieldConstants (tools/golden/reference)
# compiled against small stand-ins for the WPILib math classes (tools/golden/stubs), which mirror
# WPILib's formulas. The C++ unit tests (src/test/cpp/GoldenTest.cpp) compare the C++ port with it.
#
# Only needed if the reference behaviour should change (normally never). Requires a JDK 17+.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
WORK="$ROOT/build-golden"
MAYHEM_VERSION="2026.4.5"
JACKSON_VERSION="2.15.2"
mkdir -p "$WORK/lib" "$WORK/classes"
fetch() { [ -f "$WORK/lib/$2" ] || curl -sSfL -o "$WORK/lib/$2" "$1"; }
fetch "https://ryanabraham1.github.io/Mayhem/maven/mayhemlib/MayhemLib-java/$MAYHEM_VERSION/MayhemLib-java-$MAYHEM_VERSION.jar" mayhemlib.jar
for a in databind core annotations; do
  fetch "https://repo1.maven.org/maven2/com/fasterxml/jackson/core/jackson-$a/$JACKSON_VERSION/jackson-$a-$JACKSON_VERSION.jar" "jackson-$a.jar"
done
CP="$WORK/lib/mayhemlib.jar:$WORK/lib/jackson-databind.jar:$WORK/lib/jackson-core.jar:$WORK/lib/jackson-annotations.jar"
find "$ROOT/tools/golden/stubs" "$ROOT/tools/golden/reference" "$ROOT/tools/golden/src" -name '*.java' > "$WORK/sources.txt"
javac -nowarn -d "$WORK/classes" -cp "$CP" @"$WORK/sources.txt"
# stubs first on the classpath so they shadow nothing else
java -cp "$WORK/classes:$CP" Golden "$ROOT/src/main/deploy" "$ROOT/src/test/resources/golden"
echo "Golden data written to src/test/resources/golden"
