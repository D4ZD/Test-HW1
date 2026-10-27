# Unit tests (JUnit 5)

The production source files are unchanged. All test classes are in
`src/test/java/autopilot/core/`, and the brake test data is in
`src/test/resources/autopilot/core/brake-cases.csv`.

Run from the project root:

```bash
mvn test
```

The tests use the `method_condition_expectedOutcome` naming convention.
Each test is isolated: controllers are created afresh, including for every
parameterized invocation. Stateful behavior is tested only through deliberate
sequences of calls within a test.

## Parameterized sources (five sources, five different methods)

| Source | Tested method | Why |
| --- | --- | --- |
| `@CsvSource` | `AdaptiveCruise.safeDistanceFor` | Speed-to-distance input/output pairs |
| `@MethodSource` | `AdaptiveCruise.update` | Multiple input values plus expected mode and pressure/acceleration |
| `@EnumSource` | `GearBox.changeTo` | Forbidden gear destinations from park |
| `@ValueSource` | `SteeringController.update` | Non-finite requested angles |
| `@CsvFileSource` | `BrakeController.apply` | Brake pressure, speed, gear and expected pressure from an external CSV |

## Important conditions covered

- **AdaptiveCruise**: safe distance and invalid inputs; emergency boundary;
  normal proportional braking; exact safe distance; accelerate/hold decisions;
  maximum acceleration step; release threshold and braking hysteresis.
- **BrakeController**: zero/regular/capped pressure; ABS thresholds in speed
  and pressure; alternating ABS pulses; reset of pulses; emergency braking;
  illegal park/invalid input cases.
- **GearBox**: allowed and forbidden transitions; speed limit just below and
  at 5 km/h; stopped-only reverse/drive changes; validation; unchanged gear
  after failed shift; no-op shifts.
- **SteeringController**: positive/negative requests; 60-degree tick limit;
  physical range; 120 km/h high-speed threshold; self-centering; invalid input.
- **AutoDriver**: refactored only the five existing cases (normal, positive
  and negative lane offset, speed sensor failure, obstacle sensor failure).
  The original repeated assertions and anonymous classes were replaced with
  sensor lambdas, one helper factory, and a single record assertion per test.

Note: Java 26 Temurin can run this project; the existing `pom.xml` targets
Java 17 bytecode (`maven.compiler.release=17`), which is compatible with
Java 26. No production code or project compiler settings needed changing.
