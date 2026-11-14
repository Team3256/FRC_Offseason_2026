# Golden data from the original Java implementation

`generate.sh` compiles and runs the **original Java code** and writes what it computes to
`src/test/resources/golden/*.csv`. `src/test/cpp/GoldenTest.cpp` then feeds the C++ port the same
inputs and requires the same outputs, so behavioural equivalence with the Java robot code stays
tested in CI without Java.

What runs:

* `MayhemLib-java` 2026.4.5: the exact jar the Java robot used (from its vendordep maven).
* The robot's Java `TrenchAssist`, `ShotCalculator` and `FieldConstants`, unchanged copies in
  `reference/`. `SwerveConstants` is copied with its CTRE/units-typed fields removed; values are
  unchanged.
* `stubs/` holds small stand-ins for the WPILib Java math classes this code touches (Rotation2d,
  Pose2d, MathUtil, InterpolatingDoubleTreeMap, ...). WPILib's Java jars are only published on
  frcmaven. The stand-ins use WPILib's formulas, and the C++ side uses the real WPILib C++, so any
  difference would show up as a test failure.

You only need to re-run this if the reference behaviour should change (normally never). It needs a
JDK 17+ and internet access:

```bash
./tools/golden/generate.sh
```
