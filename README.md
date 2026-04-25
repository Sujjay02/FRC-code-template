# FRC Jumpstart

A VS Code extension that generates production-ready WPILib 2026 Java robot code from a curated template library — drops files straight into `src/main/java/frc/robot/`.

## Template families

- **Swerve Drive** — REV MAXSwerve (SparkMax) or CTRE (TalonFX + CANcoder), NavX or Pigeon 2 gyro, pose estimator with vision fusion.
- **Tank Drive** — Leader/follower differential drive for REV or CTRE.
- **Mecanum Drive** — 4-motor mecanum with field-relative cartesian drive.
- **Flywheel Shooter** — Dual-motor velocity PID shooter.
- **Shooter + Hopper** — Shooter with beam-break feeder/indexer.
- **Turret** — Absolute-encoder position PID rotating base.
- **Puncher / Kicker** — DoubleSolenoid punch with configurable hold time.
- **Catapult** — Motor windup + home sensor fire cycle.
- **Intake (Roller)** — Single roller with beam break auto-stop.
- **Intake with Deploy Arm** — Pivot arm + roller with deploy/stow commands.
- **Conveyor / Belt** — Belt transport with piece detection.
- **Serializer** — Two-zone indexer with independent zone control.
- **Elevator** — REV SparkMax position PID or CTRE MotionMagic (jerk-limited, gravity feedforward) with L1–L4 presets.
- **Pivot Arm** — REV SparkMax + absolute encoder or CTRE MotionMagic with cosine gravity compensation.
- **Double-Jointed Arm** — Independent shoulder + elbow position PIDs with combined presets.
- **Wrist** — Small rotational end-effector with absolute encoder.
- **Telescope / Extension** — Linear slide position PID.
- **Generic Mechanism** — Custom name + comma-separated preset positions auto-generates a full subsystem.
- **Climb** — Dual-motor winch with extend/retract encoder limits.
- **Buddy Climb** — Deploy arm for alliance partner assist.
- **Claw / Gripper** — Roller gripper with piece detection and hold mode.
- **Pneumatics** — REV PH compressor management + DoubleSolenoid template.
- **Vision (PhotonVision)** — `PhotonPoseEstimator` with `MULTI_TAG_PNP_ON_COPROCESSOR`, yaw-PID align command.
- **Vision (Limelight)** — NetworkTables 4 with botpose, pipeline switching, yaw-PID align command.
- **Addressable LEDs** — Solid, rainbow, breathe, strobe, chase, progress bar, alliance-aware, indicator.
- **PathPlanner Autos** — `AutoBuilder` config, `NamedCommands` registry, alliance flipping, `SendableChooser`.
- **Choreo Autos** — `AutoFactory` with PID feed-forward correction, multi-segment auto template.

## Design

Every line of Java output comes from hand-written TypeScript template literals — no LLM generation. Output is deterministic and verified against 2026 APIs: REVLib `com.revrobotics.spark` with `SparkMaxConfig` + `configure(ResetMode, PersistMode)`, CTRE Phoenix 6 `TalonFXConfiguration` + `MotionMagicVoltage`, Studica NavX, and current PathPlanner / ChoreoLib patterns.

The panel detects your WPILib project year and language from `.wpilib/wpilib_preferences.json` and displays it at the bottom.

## Development

```bash
npm install
npm run compile
# F5 to launch the Extension Development Host in a WPILib workspace
npm run package   # produces frc-jumpstart-<version>.vsix
```

Files are written into `src/main/java/frc/robot/subsystems/` with a modal overwrite prompt if the file already exists.
