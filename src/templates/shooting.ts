// ─── Flywheel Shooter ─────────────────────────────────────────────────────────

export function tplShooter(): string { return `package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
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

public class ShooterSubsystem extends SubsystemBase {

    private static final int    kTopCanId     = 0;      // TODO: set CAN ID
    private static final int    kBottomCanId  = 1;      // TODO: set CAN ID
    private static final double kP            = 0.0005;
    private static final double kI            = 0.0;
    private static final double kD            = 0.0;
    private static final double kFF           = 0.000175;
    private static final double kTargetRpm    = 4000;   // TODO: tune
    private static final double kToleranceRpm = 100;

    private final SparkMax m_top    = new SparkMax(kTopCanId,    MotorType.kBrushless);
    private final SparkMax m_bottom = new SparkMax(kBottomCanId, MotorType.kBrushless);
    private final RelativeEncoder           m_encoder = m_top.getEncoder();
    private final SparkClosedLoopController m_pid     = m_top.getClosedLoopController();
    private double m_targetRpm = 0;

    public ShooterSubsystem() {
        SparkMaxConfig topCfg = new SparkMaxConfig();
        topCfg.idleMode(IdleMode.kCoast).smartCurrentLimit(40);
        topCfg.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(kP, kI, kD).velocityFF(kFF).outputRange(-1, 1);
        m_top.configure(topCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        SparkMaxConfig botCfg = new SparkMaxConfig();
        botCfg.idleMode(IdleMode.kCoast).smartCurrentLimit(40).follow(m_top, true);
        m_bottom.configure(botCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/RPM",       m_encoder.getVelocity());
        SmartDashboard.putNumber("Shooter/TargetRPM", m_targetRpm);
        SmartDashboard.putBoolean("Shooter/AtSpeed",  atTargetSpeed());
    }

    public void spinUp() { setRpm(kTargetRpm); }
    public void stop()   { m_targetRpm = 0; m_top.stopMotor(); }

    public void setRpm(double rpm) {
        m_targetRpm = rpm;
        m_pid.setReference(rpm, ControlType.kVelocity);
    }

    public boolean atTargetSpeed() {
        return m_targetRpm > 0 && Math.abs(m_encoder.getVelocity() - m_targetRpm) < kToleranceRpm;
    }

    public Command spinUpCommand() {
        return runOnce(this::spinUp).andThen(run(() -> {}).until(this::atTargetSpeed));
    }
    public Command stopCommand() { return runOnce(this::stop); }
}
`; }

// ─── Hopper / Indexer ─────────────────────────────────────────────────────────

export function tplHopper(): string { return `package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSubsystem extends SubsystemBase {

    private static final int    kFeederCanId = 2;   // TODO: set CAN ID
    private static final int    kSensorPort  = 0;   // TODO: set DIO port
    private static final double kFeedSpeed   = 0.6; // TODO: tune

    private final SparkMax     m_feeder = new SparkMax(kFeederCanId, MotorType.kBrushless);
    private final DigitalInput m_sensor = new DigitalInput(kSensorPort);

    public HopperSubsystem() {
        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.idleMode(IdleMode.kBrake).smartCurrentLimit(30);
        m_feeder.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Hopper/HasPiece", hasPiece());
    }

    public void feed()    { m_feeder.set(kFeedSpeed); }
    public void reverse() { m_feeder.set(-kFeedSpeed); }
    public void stop()    { m_feeder.stopMotor(); }
    public boolean hasPiece() { return !m_sensor.get(); }

    public Command feedCommand()    { return startEnd(this::feed,    this::stop); }
    public Command reverseCommand() { return startEnd(this::reverse, this::stop); }
}
`; }

// ─── Turret ───────────────────────────────────────────────────────────────────

export function tplTurret(): string { return `package frc.robot.subsystems;

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

public class TurretSubsystem extends SubsystemBase {

    private static final int    kCanId      = 8;     // TODO: set CAN ID
    private static final double kP          = 1.5;
    private static final double kI          = 0.0;
    private static final double kD          = 0.0;
    private static final double kTolerance  = 0.01;  // rotations (absolute encoder)

    // Preset positions in rotations (0–1 = full revolution)
    private static final double kForward = 0.0;  // TODO: tune
    private static final double kLeft    = 0.25; // TODO: tune
    private static final double kRight   = 0.75; // TODO: tune

    private final SparkMax m_motor = new SparkMax(kCanId, MotorType.kBrushless);
    private final AbsoluteEncoder           m_encoder = m_motor.getAbsoluteEncoder();
    private final SparkClosedLoopController m_pid     = m_motor.getClosedLoopController();
    private double m_setpoint = 0;

    public TurretSubsystem() {
        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.idleMode(IdleMode.kBrake).smartCurrentLimit(30);
        cfg.absoluteEncoder.positionConversionFactor(1.0);
        cfg.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(kP, kI, kD).outputRange(-0.5, 0.5)
            .positionWrappingEnabled(true)
            .positionWrappingMinInput(0).positionWrappingMaxInput(1);
        m_motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret/Position",  m_encoder.getPosition());
        SmartDashboard.putNumber("Turret/Setpoint",  m_setpoint);
        SmartDashboard.putBoolean("Turret/AtTarget", atTarget());
    }

    public void setAngle(double rotations) {
        m_setpoint = rotations;
        m_pid.setReference(rotations, ControlType.kPosition);
    }

    public boolean atTarget() {
        return Math.abs(m_encoder.getPosition() - m_setpoint) < kTolerance;
    }

    public void faceForward() { setAngle(kForward); }
    public void faceLeft()    { setAngle(kLeft);    }
    public void faceRight()   { setAngle(kRight);   }

    public Command faceForwardCommand() { return runOnce(this::faceForward).andThen(run(() -> {}).until(this::atTarget)); }
    public Command faceLeftCommand()    { return runOnce(this::faceLeft).andThen(run(() -> {}).until(this::atTarget)); }
    public Command faceRightCommand()   { return runOnce(this::faceRight).andThen(run(() -> {}).until(this::atTarget)); }
}
`; }

// ─── Puncher / Kicker (solenoid) ──────────────────────────────────────────────

export function tplPuncher(): string { return `package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PuncherSubsystem extends SubsystemBase {

    private static final int    kForwardChannel = 0;   // TODO: set solenoid channel
    private static final int    kReverseChannel = 1;   // TODO: set solenoid channel
    private static final double kExtendSeconds  = 0.1; // TODO: tune hold time

    private final DoubleSolenoid m_solenoid = new DoubleSolenoid(
        PneumaticsModuleType.REVPH, kForwardChannel, kReverseChannel);

    public PuncherSubsystem() { retract(); }

    @Override
    public void periodic() {
        SmartDashboard.putString("Puncher/State", m_solenoid.get().toString());
    }

    public void extend()  { m_solenoid.set(Value.kForward); }
    public void retract() { m_solenoid.set(Value.kReverse); }
    public void off()     { m_solenoid.set(Value.kOff); }

    /** Extend briefly then retract. */
    public Command punchCommand() {
        return runOnce(this::extend)
            .andThen(run(() -> {}).withTimeout(kExtendSeconds))
            .andThen(runOnce(this::retract))
            .withName("Punch");
    }
}
`; }

// ─── Catapult (motor windup + home sensor) ────────────────────────────────────

export function tplCatapult(): string { return `package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CatapultSubsystem extends SubsystemBase {

    private static final int    kCanId     = 9;   // TODO: set CAN ID
    private static final int    kHomePort  = 2;   // TODO: DIO — limit switch at home position
    private static final double kWindSpeed = 0.4; // TODO: tune
    private static final double kFireSpeed = 1.0; // TODO: tune

    private final SparkMax     m_motor   = new SparkMax(kCanId, MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_motor.getEncoder();
    private final DigitalInput    m_home    = new DigitalInput(kHomePort);

    public CatapultSubsystem() {
        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
        m_motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Catapult/Position", m_encoder.getPosition());
        SmartDashboard.putBoolean("Catapult/AtHome",  atHome());
    }

    public void windUp() { m_motor.set(kWindSpeed); }
    public void fire()   { m_motor.set(kFireSpeed); }
    public void stop()   { m_motor.stopMotor(); }

    /** Limit switch true = at home (normally-closed wiring). */
    public boolean atHome() { return !m_home.get(); }

    /** Fire through a full cycle and return to home. */
    public Command fireCommand() {
        return runOnce(this::fire)
            .andThen(run(() -> {}).until(this::atHome))
            .andThen(runOnce(this::stop))
            .withName("Fire");
    }

    /** Wind back to the cocked position (used before match if discharged). */
    public Command windUpCommand() {
        return startEnd(this::windUp, this::stop).until(() -> !atHome());
    }
}
`; }
