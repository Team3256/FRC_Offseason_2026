#!/usr/bin/env bash
# Fetches the native libraries needed to build and test the robot code on a Linux x86-64 desktop
# WITHOUT GradleRIO (for environments that cannot reach frcmaven.wpi.edu).
#
#   WPILib C++ (wpilibc, wpimath, ntcore, HAL sim, ...)  -> RobotPy "robotpy-native-*" wheels (PyPI)
#   Phoenix 6 C++ (headers + desktop sim libs)           -> maven.ctr-electronics.com
#   PhotonLib C++ headers (syntax check only)            -> maven.photonvision.org
#
# The real robot build (./gradlew build / deploy) always uses the official WPILib libraries from the
# vendordeps.
set -euo pipefail
WPILIB_VERSION="${WPILIB_VERSION:-2026.2.2}"
PHOENIX_VERSION="26.3.0"
PHOTON_VERSION="v2026.3.4"
OUT="${1:-$(cd "$(dirname "$0")/../.." && pwd)/build-desktop/deps}"
mkdir -p "$OUT"/{dl,wheels,inc,lib}
cd "$OUT"

if [ ! -f .wpilib-done ]; then
  python3 -m pip download --no-deps -q -d wheels \
    robotpy-native-wpilib==$WPILIB_VERSION robotpy-native-wpimath==$WPILIB_VERSION \
    robotpy-native-wpiutil==$WPILIB_VERSION robotpy-native-ntcore==$WPILIB_VERSION \
    robotpy-native-wpihal==$WPILIB_VERSION robotpy-native-wpinet==$WPILIB_VERSION \
    robotpy-native-apriltag==$WPILIB_VERSION
  rm -rf wheel-x && mkdir wheel-x
  for w in wheels/robotpy_native_*.whl; do unzip -qo "$w" -d wheel-x; done
  for p in wheel-x/native/*/; do
    cp -r "$p"include/. inc/
    cp -a "$p"lib/. lib/
  done
  touch .wpilib-done
fi

get() { curl -sSfL -o "dl/$2" "$1"; }
if [ ! -f .phoenix-done ]; then
  C=https://maven.ctr-electronics.com/release/com/ctre/phoenix6
  get $C/wpiapi-cpp/$PHOENIX_VERSION/wpiapi-cpp-$PHOENIX_VERSION-headers.zip p6wpi-h.zip
  get $C/tools/$PHOENIX_VERSION/tools-$PHOENIX_VERSION-headers.zip p6tools-h.zip
  get $C/sim/wpiapi-cpp-sim/$PHOENIX_VERSION/wpiapi-cpp-sim-$PHOENIX_VERSION-linuxx86-64.zip p6wpisim.zip
  get $C/sim/tools-sim/$PHOENIX_VERSION/tools-sim-$PHOENIX_VERSION-linuxx86-64.zip p6toolssim.zip
  for a in simTalonSRX simVictorSPX simPigeonIMU simProTalonFX simProTalonFXS simProCANcoder \
           simProPigeon2 simProCANrange simProCANdi simProCANdle; do
    get $C/sim/$a/$PHOENIX_VERSION/$a-$PHOENIX_VERSION-linuxx86-64.zip $a.zip
  done
  touch .phoenix-done
fi
if [ ! -f .photon-done ]; then
  P=https://maven.photonvision.org/repository/internal/org/photonvision
  for a in photonlib-cpp photontargeting-cpp; do
    get $P/$a/$PHOTON_VERSION/$a-$PHOTON_VERSION-headers.zip $a-h.zip
  done
  touch .photon-done
fi
rm -rf libx && mkdir libx
for z in dl/*.zip; do
  case "$z" in
    *-h.zip) unzip -qo "$z" -d inc ;;
    *) unzip -qo "$z" -d libx ;;
  esac
done
cp -a libx/linux/x86-64/shared/. lib/
echo "Desktop dependencies ready in $OUT"
