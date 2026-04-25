// ─── Elevator (REV SparkMax) ──────────────────────────────────────────────────

export function tplElevatorRev(): string { return `package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

    private static final int    kLeaderCanId  = 4;   // TODO: set CAN ID
    private static final int    kFollowerCanId = 5;  // TODO: set CAN ID
    private static final double kP             = 0.1;
    private static final double kI             = 0.0;
    private static final double kD             = 0.0;
    private static final double kFF            = 0.0;
    private static final double kMaxOutput     = 0.6;
    private static final double kTolerance     = 0.5; // rotations

    private static final double kStow = 0.0;
    private static final double kL1   = 10.0; // TODO: tune (rotations)
    private static final double kL2   = 22.0; // TODO: tune
    private static final double kL3   = 38.0; // TODO: tune
    private static final double kL4   = 55.0; // TODO: tune

    private final SparkMax m_leader   = new SparkMax(kLeaderCanId,  MotorType.kBrushless);
    private final SparkMax m_follower = new SparkMax(kFollowerCanId, MotorType.kBrushless);
    private final RelativeEncoder           m_encoder = m_leader.getEncoder();
    private final SparkClosedLoopController m_pid     = m_leader.getClosedLoopController();
    private double m_setpoint = 0;

    public ElevatorSubsystem() {
        SparkMaxConfig lCfg = new SparkMaxConfig();
        lCfg.idleMode(IdleMode.kBrake).smartCurrentLimit(50);
        lCfg.closedLoop.pid(kP, kI, kD).velocityFF(kFF).outputRange(-kMaxOutput, kMaxOutput);
        m_leader.configure(lCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        SparkMaxConfig fCfg = new SparkMaxConfig();
        fCfg.follow(m_leader, true);
        m_follower.configure(fCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator/Position",  m_encoder.getPosition());
        SmartDashboard.putNumber("Elevator/Setpoint",  m_setpoint);
        SmartDashboard.putBoolean("Elevator/AtTarget", atTarget());
    }

    public void setHeight(double rotations) {
        m_setpoint = rotations;
        m_pid.setReference(rotations, ControlType.kPosition);
    }

    public boolean atTarget() { return Math.abs(m_encoder.getPosition() - m_setpoint) < kTolerance; }

    public void goToStow() { setHeight(kStow); }
    public void goToL1()   { setHeight(kL1);   }
    public void goToL2()   { setHeight(kL2);   }
    public void goToL3()   { setHeight(kL3);   }
    public void goToL4()   { setHeight(kL4);   }

    public Command stowCommand() { return runOnce(this::goToStow).andThen(run(() -> {}).until(this::atTarget)); }
    public Command l1Command()   { return runOnce(this::goToL1).andThen(run(() -> {}).until(this::atTarget)); }
    public Command l2Command()   { return runOnce(this::goToL2).andThen(run(() -> {}).until(this::atTarget)); }
    public Command l3Command()   { return runOnce(this::goToL3).andThen(run(() -> {}).until(this::atTarget)); }
    public Command l4Command()   { return runOnce(this::goToL4).andThen(run(() -> {}).until(this::atTarget)); }
}
`; }

// ─── Elevator (CTRE MotionMagic) ─────────────────────────────────────────────

export function tplElevatorMM(): string { return `package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

    private static final int    kLeaderCanId  = 4;      // TODO: set CAN ID
    private static final int    kFollowerCanId = 5;     // TODO: set CAN ID

    // Slot 0 gains — run Phoenix Tuner X SysId to find kS/kV/kA, then tune kP/kD
    private static final double kS = 0.25;   // TODO: static friction
    private static final double kV = 0.12;   // TODO: velocity feedforward
    private static final double kA = 0.01;   // TODO: acceleration feedforward
    private static final double kP = 4.8;    // TODO: proportional
    private static final double kI = 0.0;
    private static final double kD = 0.1;    // TODO: derivative
    private static final double kG = 0.61;   // TODO: gravity feedforward (hold weight)

    private static final double kCruiseVel   = 80.0;   // rot/s  — TODO: tune
    private static final double kAcceleration = 160.0; // rot/s² — TODO: tune
    private static final double kJerk         = 1600.0;// rot/s³ — TODO: tune
    private static final double kTolerance    = 0.5;   // rotations

    private static final double kStow = 0.0;
    private static final double kL1   = 10.0; // TODO: tune (rotations)
    private static final double kL2   = 22.0; // TODO: tune
    private static final double kL3   = 38.0; // TODO: tune
    private static final double kL4   = 55.0; // TODO: tune

    private final TalonFX m_leader   = new TalonFX(kLeaderCanId);
    private final TalonFX m_follower = new TalonFX(kFollowerCanId);
    private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    private double m_setpoint = 0;

    public ElevatorSubsystem() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfg.MotorOutput.Inverted    = InvertedValue.CounterClockwise_Positive;
        cfg.Slot0.kS = kS;  cfg.Slot0.kV = kV;  cfg.Slot0.kA = kA;
        cfg.Slot0.kP = kP;  cfg.Slot0.kI = kI;  cfg.Slot0.kD = kD;
        cfg.Slot0.kG = kG;
        cfg.Slot0.GravityType                    = GravityTypeValue.Elevator_Static;
        cfg.MotionMagic.MotionMagicCruiseVelocity = kCruiseVel;
        cfg.MotionMagic.MotionMagicAcceleration   = kAcceleration;
        cfg.MotionMagic.MotionMagicJerk           = kJerk;
        m_leader.getConfigurator().apply(cfg);
        m_follower.setControl(new Follower(kLeaderCanId, true));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator/Position",  m_leader.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Elevator/Setpoint",  m_setpoint);
        SmartDashboard.putBoolean("Elevator/AtTarget", atTarget());
    }

    public void setHeight(double rotations) {
        m_setpoint = rotations;
        m_leader.setControl(m_request.withPosition(rotations));
    }

    public boolean atTarget() {
        return Math.abs(m_leader.getPosition().getValueAsDouble() - m_setpoint) < kTolerance;
    }

    public void goToStow() { setHeight(kStow); }
    public void goToL1()   { setHeight(kL1);   }
    public void goToL2()   { setHeight(kL2);   }
    public void goToL3()   { setHeight(kL3);   }
    public void goToL4()   { setHeight(kL4);   }

    public Command stowCommand() { return runOnce(this::goToStow).andThen(run(() -> {}).until(this::atTarget)); }
    public Command l1Command()   { return runOnce(this::goToL1).andThen(run(() -> {}).until(this::atTarget)); }
    public Command l2Command()   { return runOnce(this::goToL2).andThen(run(() -> {}).until(this::atTarget)); }
    public Command l3Command()   { return runOnce(this::goToL3).andThen(run(() -> {}).until(this::atTarget)); }
    public Command l4Command()   { return runOnce(this::goToL4).andThen(run(() -> {}).until(this::atTarget)); }
}
`; }

// ─── Pivot Arm (REV SparkMax + AbsoluteEncoder) ───────────────────────────────

export function tplArmRev(): string { return `package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

    private static final int    kCanId     = 13;   // TODO: set CAN ID
    private static final double kP         = 1.5;
    private static final double kI         = 0.0;
    private static final double kD         = 0.0;
    private static final double kTolerance = 0.01; // rotations

    // Absolute encoder positions (0–1 = full revolution)
    private static final double kStow   = 0.0;   // TODO: tune
    private static final double kIntake = 0.20;  // TODO: tune
    private static final double kScore  = 0.40;  // TODO: tune

    private final SparkMax m_motor = new SparkMax(kCanId, MotorType.kBrushless);
    private final AbsoluteEncoder           m_encoder = m_motor.getAbsoluteEncoder();
    private final SparkClosedLoopController m_pid     = m_motor.getClosedLoopController();
    private double m_setpoint = 0;

    public ArmSubsystem() {
        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
        cfg.absoluteEncoder.positionConversionFactor(1.0);
        cfg.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(kP, kI, kD).outputRange(-0.6, 0.6)
            .positionWrappingEnabled(false);
        m_motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm/Position",  m_encoder.getPosition());
        SmartDashboard.putNumber("Arm/Setpoint",  m_setpoint);
        SmartDashboard.putBoolean("Arm/AtTarget", atTarget());
    }

    public void setAngle(double rotations) {
        m_setpoint = rotations;
        m_pid.setReference(rotations, ControlType.kPosition);
    }

    public boolean atTarget() { return Math.abs(m_encoder.getPosition() - m_setpoint) < kTolerance; }

    public void goToStow()   { setAngle(kStow);   }
    public void goToIntake() { setAngle(kIntake); }
    public void goToScore()  { setAngle(kScore);  }

    public Command stowCommand()   { return runOnce(this::goToStow).andThen(run(() -> {}).until(this::atTarget)); }
    public Command intakeCommand() { return runOnce(this::goToIntake).andThen(run(() -> {}).until(this::atTarget)); }
    public Command scoreCommand()  { return runOnce(this::goToScore).andThen(run(() -> {}).until(this::atTarget)); }
}
`; }

// ─── Pivot Arm (CTRE MotionMagic, Arm_Cosine gravity) ────────────────────────

export function tplArmMM(): string { return `package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

    private static final int    kCanId        = 13;   // TODO: set CAN ID

    // Slot 0 gains — use Phoenix Tuner X SysId, then tune kP/kD
    private static final double kS = 0.25;   // TODO
    private static final double kV = 0.12;   // TODO
    private static final double kA = 0.01;   // TODO
    private static final double kP = 4.0;    // TODO
    private static final double kI = 0.0;
    private static final double kD = 0.1;    // TODO
    private static final double kG = 0.5;    // TODO: cosine gravity feedforward

    private static final double kCruiseVel    = 40.0;  // rot/s  — TODO: tune
    private static final double kAcceleration = 80.0;  // rot/s² — TODO: tune
    private static final double kJerk         = 800.0; // rot/s³ — TODO: tune
    private static final double kTolerance    = 0.5;   // rotations

    private static final double kStow   = 0.0;  // TODO: tune (rotations from encoder zero)
    private static final double kIntake = 5.0;  // TODO: tune
    private static final double kScore  = 15.0; // TODO: tune

    private final TalonFX m_motor = new TalonFX(kCanId);
    private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    private double m_setpoint = 0;

    public ArmSubsystem() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfg.Slot0.kS = kS;  cfg.Slot0.kV = kV;  cfg.Slot0.kA = kA;
        cfg.Slot0.kP = kP;  cfg.Slot0.kI = kI;  cfg.Slot0.kD = kD;
        cfg.Slot0.kG = kG;
        cfg.Slot0.GravityType                    = GravityTypeValue.Arm_Cosine;
        cfg.MotionMagic.MotionMagicCruiseVelocity = kCruiseVel;
        cfg.MotionMagic.MotionMagicAcceleration   = kAcceleration;
        cfg.MotionMagic.MotionMagicJerk           = kJerk;
        m_motor.getConfigurator().apply(cfg);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm/Position",  m_motor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Arm/Setpoint",  m_setpoint);
        SmartDashboard.putBoolean("Arm/AtTarget", atTarget());
    }

    public void setAngle(double rotations) {
        m_setpoint = rotations;
        m_motor.setControl(m_request.withPosition(rotations));
    }

    public boolean atTarget() {
        return Math.abs(m_motor.getPosition().getValueAsDouble() - m_setpoint) < kTolerance;
    }

    public void goToStow()   { setAngle(kStow);   }
    public void goToIntake() { setAngle(kIntake); }
    public void goToScore()  { setAngle(kScore);  }

    public Command stowCommand()   { return runOnce(this::goToStow).andThen(run(() -> {}).until(this::atTarget)); }
    public Command intakeCommand() { return runOnce(this::goToIntake).andThen(run(() -> {}).until(this::atTarget)); }
    public Command scoreCommand()  { return runOnce(this::goToScore).andThen(run(() -> {}).until(this::atTarget)); }
}
`; }

// ─── Double-Jointed Arm (shoulder + elbow, REV) ───────────────────────────────

export function tplDoubleArm(): string { return `package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

    private static final int    kShoulderCanId = 13;   // TODO: set CAN ID
    private static final int    kElbowCanId    = 14;   // TODO: set CAN ID
    private static final double kShoulderP     = 1.5;
    private static final double kElbowP        = 1.5;
    private static final double kTolerance     = 0.01; // rotations

    // Shoulder presets (absolute encoder 0–1)
    private static final double kShoulderStow   = 0.0;  // TODO: tune
    private static final double kShoulderIntake = 0.15; // TODO: tune
    private static final double kShoulderL2     = 0.30; // TODO: tune
    private static final double kShoulderL3     = 0.35; // TODO: tune
    private static final double kShoulderL4     = 0.40; // TODO: tune

    // Elbow presets (absolute encoder 0–1)
    private static final double kElbowStow   = 0.0;  // TODO: tune
    private static final double kElbowIntake = 0.25; // TODO: tune
    private static final double kElbowL2     = 0.45; // TODO: tune
    private static final double kElbowL3     = 0.50; // TODO: tune
    private static final double kElbowL4     = 0.55; // TODO: tune

    private final SparkMax m_shoulder = new SparkMax(kShoulderCanId, MotorType.kBrushless);
    private final SparkMax m_elbow    = new SparkMax(kElbowCanId,    MotorType.kBrushless);
    private final AbsoluteEncoder           m_shoulderEnc = m_shoulder.getAbsoluteEncoder();
    private final AbsoluteEncoder           m_elbowEnc    = m_elbow.getAbsoluteEncoder();
    private final SparkClosedLoopController m_shoulderPID = m_shoulder.getClosedLoopController();
    private final SparkClosedLoopController m_elbowPID    = m_elbow.getClosedLoopController();

    public ArmSubsystem() {
        SparkMaxConfig sCfg = new SparkMaxConfig();
        sCfg.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
        sCfg.absoluteEncoder.positionConversionFactor(1.0);
        sCfg.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(kShoulderP, 0, 0).outputRange(-0.6, 0.6).positionWrappingEnabled(false);
        m_shoulder.configure(sCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig eCfg = new SparkMaxConfig();
        eCfg.idleMode(IdleMode.kBrake).smartCurrentLimit(30);
        eCfg.absoluteEncoder.positionConversionFactor(1.0);
        eCfg.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(kElbowP, 0, 0).outputRange(-0.5, 0.5).positionWrappingEnabled(false);
        m_elbow.configure(eCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm/ShoulderPos", m_shoulderEnc.getPosition());
        SmartDashboard.putNumber("Arm/ElbowPos",    m_elbowEnc.getPosition());
        SmartDashboard.putBoolean("Arm/AtTarget",   atTarget());
    }

    public void setAngles(double shoulder, double elbow) {
        m_shoulderPID.setReference(shoulder, ControlType.kPosition);
        m_elbowPID.setReference(elbow,    ControlType.kPosition);
    }

    public boolean atTarget() {
        return Math.abs(m_shoulderEnc.getPosition() - m_shoulder.getAbsoluteEncoder().getPosition()) < kTolerance
            && Math.abs(m_elbowEnc.getPosition()    - m_elbow.getAbsoluteEncoder().getPosition())    < kTolerance;
    }

    public void goToStow()   { setAngles(kShoulderStow,   kElbowStow);   }
    public void goToIntake() { setAngles(kShoulderIntake, kElbowIntake); }
    public void goToL2()     { setAngles(kShoulderL2,     kElbowL2);     }
    public void goToL3()     { setAngles(kShoulderL3,     kElbowL3);     }
    public void goToL4()     { setAngles(kShoulderL4,     kElbowL4);     }

    public Command stowCommand()   { return runOnce(this::goToStow).andThen(run(() -> {}).until(this::atTarget)); }
    public Command intakeCommand() { return runOnce(this::goToIntake).andThen(run(() -> {}).until(this::atTarget)); }
    public Command l2Command()     { return runOnce(this::goToL2).andThen(run(() -> {}).until(this::atTarget)); }
    public Command l3Command()     { return runOnce(this::goToL3).andThen(run(() -> {}).until(this::atTarget)); }
    public Command l4Command()     { return runOnce(this::goToL4).andThen(run(() -> {}).until(this::atTarget)); }
}
`; }

// ─── Wrist ────────────────────────────────────────────────────────────────────

export function tplWrist(): string { return `package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase {

    private static final int    kCanId     = 15;   // TODO: set CAN ID
    private static final double kP         = 2.0;
    private static final double kI         = 0.0;
    private static final double kD         = 0.0;
    private static final double kTolerance = 0.01; // rotations

    private static final double kStraight = 0.0;  // TODO: tune
    private static final double kAngled   = 0.15; // TODO: tune
    private static final double kDown     = 0.25; // TODO: tune

    private final SparkMax m_motor = new SparkMax(kCanId, MotorType.kBrushless);
    private final AbsoluteEncoder           m_encoder = m_motor.getAbsoluteEncoder();
    private final SparkClosedLoopController m_pid     = m_motor.getClosedLoopController();
    private double m_setpoint = 0;

    public WristSubsystem() {
        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.idleMode(IdleMode.kBrake).smartCurrentLimit(20);
        cfg.absoluteEncoder.positionConversionFactor(1.0);
        cfg.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(kP, kI, kD).outputRange(-0.4, 0.4)
            .positionWrappingEnabled(false);
        m_motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Wrist/Position",  m_encoder.getPosition());
        SmartDashboard.putBoolean("Wrist/AtTarget", atTarget());
    }

    public void setAngle(double rotations) {
        m_setpoint = rotations;
        m_pid.setReference(rotations, ControlType.kPosition);
    }

    public boolean atTarget() { return Math.abs(m_encoder.getPosition() - m_setpoint) < kTolerance; }

    public void goStraight() { setAngle(kStraight); }
    public void goAngled()   { setAngle(kAngled);   }
    public void goDown()     { setAngle(kDown);     }

    public Command straightCommand() { return runOnce(this::goStraight).andThen(run(() -> {}).until(this::atTarget)); }
    public Command angledCommand()   { return runOnce(this::goAngled).andThen(run(() -> {}).until(this::atTarget)); }
    public Command downCommand()     { return runOnce(this::goDown).andThen(run(() -> {}).until(this::atTarget)); }
}
`; }

// ─── Telescope / Linear Extension ────────────────────────────────────────────

export function tplTelescope(): string { return `package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TelescopeSubsystem extends SubsystemBase {

    private static final int    kCanId      = 16;  // TODO: set CAN ID
    private static final double kP          = 0.1;
    private static final double kI          = 0.0;
    private static final double kD          = 0.0;
    private static final double kMaxOutput  = 0.6;
    private static final double kTolerance  = 0.5; // rotations

    private static final double kRetracted = 0.0;
    private static final double kMid       = 15.0; // TODO: tune
    private static final double kExtended  = 30.0; // TODO: tune

    private final SparkMax m_motor = new SparkMax(kCanId, MotorType.kBrushless);
    private final RelativeEncoder           m_encoder = m_motor.getEncoder();
    private final SparkClosedLoopController m_pid     = m_motor.getClosedLoopController();
    private double m_setpoint = 0;

    public TelescopeSubsystem() {
        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
        cfg.closedLoop.pid(kP, kI, kD).outputRange(-kMaxOutput, kMaxOutput);
        m_motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Telescope/Position",  m_encoder.getPosition());
        SmartDashboard.putBoolean("Telescope/AtTarget", atTarget());
    }

    public void setExtension(double rotations) {
        m_setpoint = rotations;
        m_pid.setReference(rotations, ControlType.kPosition);
    }

    public boolean atTarget() { return Math.abs(m_encoder.getPosition() - m_setpoint) < kTolerance; }

    public void retract()   { setExtension(kRetracted); }
    public void goToMid()   { setExtension(kMid);       }
    public void extend()    { setExtension(kExtended);  }

    public Command retractCommand() { return runOnce(this::retract).andThen(run(() -> {}).until(this::atTarget)); }
    public Command midCommand()     { return runOnce(this::goToMid).andThen(run(() -> {}).until(this::atTarget)); }
    public Command extendCommand()  { return runOnce(this::extend).andThen(run(() -> {}).until(this::atTarget)); }
}
`; }

// ─── Generic Mechanism ────────────────────────────────────────────────────────

export function tplMechanism(name: string, presets: string[]): string {
    const fields = presets.map((p, i) =>
        `    private static final double k${cap(p)} = ${i === 0 ? '0.0' : (i * 10).toFixed(1)}; // TODO: tune`
    ).join('\n');
    const goMethods = presets.map(p =>
        `    public void goTo${cap(p)}() { setSetpoint(k${cap(p)}); }`
    ).join('\n');
    const cmdMethods = presets.map(p =>
        `    public Command ${p}Command() { return runOnce(() -> goTo${cap(p)}()).andThen(run(() -> {}).until(this::atSetpoint)); }`
    ).join('\n');
    return `package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ${name}Subsystem extends SubsystemBase {

    private static final int    kMotorCanId = 0;   // TODO: set CAN ID
    private static final double kP          = 0.1;
    private static final double kI          = 0.0;
    private static final double kD          = 0.0;
    private static final double kFF         = 0.0;
    private static final double kTolerance  = 0.5;

${fields}

    private final SparkMax m_motor = new SparkMax(kMotorCanId, MotorType.kBrushless);
    private final RelativeEncoder           m_encoder = m_motor.getEncoder();
    private final SparkClosedLoopController m_pid     = m_motor.getClosedLoopController();
    private double m_setpoint = 0;

    public ${name}Subsystem() {
        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
        cfg.closedLoop.pid(kP, kI, kD).velocityFF(kFF).outputRange(-1, 1);
        m_motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("${name}/Position",   m_encoder.getPosition());
        SmartDashboard.putNumber("${name}/Setpoint",   m_setpoint);
        SmartDashboard.putBoolean("${name}/AtSetpoint", atSetpoint());
    }

    public void setSetpoint(double setpoint) {
        m_setpoint = setpoint;
        m_pid.setReference(setpoint, ControlType.kPosition);
    }

    public void setSpeed(double speed) { m_motor.set(speed); }
    public void stop() { m_motor.stopMotor(); }
    public boolean atSetpoint() { return Math.abs(m_encoder.getPosition() - m_setpoint) < kTolerance; }
    public double getPosition() { return m_encoder.getPosition(); }

${goMethods}

${cmdMethods}
}
`;
}

function cap(s: string): string {
    return s.charAt(0).toUpperCase() + s.slice(1);
}
