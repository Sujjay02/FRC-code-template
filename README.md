# FRC Forge

A VS Code extension that generates production-ready WPILib 2026 Java robot code from a curated template database — drops files straight into `src/main/java/frc/robot/`.

## Template families

- **Swerve Drive** — 4-module swerve with mix-and-match Phoenix 6 / REVLib motors across SDS MK4 / MK4i / MK4n, REV MAXSwerve, and WCP SwerveX. CANcoder absolute encoders, NavX or Pigeon 2 gyro, pose estimator, and PathPlanner AutoBuilder pre-wired.
- **Tank Drive** — Leader/follower differential drive for REVLib or Phoenix 6.
- **Generic Mechanism** — Single or multi-motor arm / elevator / intake / shooter with preset command factory methods auto-generated from a comma-separated list.
- **Addressable LEDs** — Solid, rainbow, alliance-aware, chase, strobe, progress bar, boolean indicator.
- **Vision** — Limelight through NetworkTables or PhotonVision with `PhotonPoseEstimator`. Emits an `AlignToTarget` yaw-PID command that preserves driver translation.
- **PathPlanner Autos** — `AutoBuilder` config, `NamedCommands` registry, alliance flipping, `SendableChooser` auto selection.

## Design

Every line of Java output comes from hand-written TypeScript emitting code through template literals — no LLM-driven generation. Output is deterministic, reviewable, and free of hallucinated APIs. Templates are verified against the 2026 APIs: the REVLib `com.revrobotics.spark` namespace with `SparkMaxConfig` + `configure(config, ResetMode, PersistMode)`, Studica NavX with `NavXComType`, Phoenix 6 with `TalonFXConfiguration`, and current PathPlanner `AutoBuilder` / `PPHolonomicDriveController` patterns.

## Commands

| Command | Title |
|---|---|
| `frcForge.open` | FRC Forge: Open |
| `frcForge.newSwerve` | FRC Forge: New Swerve |
| `frcForge.newTank` | FRC Forge: New Tank |
| `frcForge.newMechanism` | FRC Forge: New Mechanism |
| `frcForge.newLeds` | FRC Forge: New LEDs |
| `frcForge.newVision` | FRC Forge: New Vision |
| `frcForge.newPathPlanner` | FRC Forge: New PathPlanner |

## Development

```bash
npm install
npm run compile
# F5 into the extension host to test in a fresh WPILib workspace
npm run package   # produces frc-forge-<version>.vsix
```

The extension detects WPILib projects by scanning for `build.gradle` plus either `.wpilib/wpilib_preferences.json` or `src/main/java/frc/robot/`. Files are written into the configured Java package, with a modal prompt before any existing file is overwritten.
