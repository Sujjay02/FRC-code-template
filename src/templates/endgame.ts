// ─── Climb ────────────────────────────────────────────────────────────────────

export function tplClimb(): string { return `package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {

    private static final int    kLeftCanId  = 6;    // TODO: set CAN ID
    private static final int    kRightCanId = 7;    // TODO: set CAN ID
    private static final double kSpeed      = 0.6;  // TODO: tune
    private static final double kExtendPos  = 80.0; // TODO: tune (rotations)
    private static final double kTolerance  = 1.0;

    private final SparkMax m_left  = new SparkMax(kLeftCanId,  MotorType.kBrushless);
    private final SparkMax m_right = new SparkMax(kRightCanId, MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_left.getEncoder();

    public ClimbSubsystem() {
        SparkMaxConfig lCfg = new SparkMaxConfig();
        lCfg.idleMode(IdleMode.kBrake).smartCurrentLimit(60);
        m_left.configure(lCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        SparkMaxConfig rCfg = new SparkMaxConfig();
        rCfg.idleMode(IdleMode.kBrake).smartCurrentLimit(60).follow(m_left, true);
        m_right.configure(rCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climb/Position",   m_encoder.getPosition());
        SmartDashboard.putBoolean("Climb/Extended",  isExtended());
        SmartDashboard.putBoolean("Climb/Retracted", isRetracted());
    }

    public void extend()  { m_left.set(kSpeed);  }
    public void retract() { m_left.set(-kSpeed); }
    public void stop()    { m_left.stopMotor();   }

    public boolean isExtended()  { return Math.abs(m_encoder.getPosition() - kExtendPos) < kTolerance; }
    public boolean isRetracted() { return m_encoder.getPosition() < kTolerance; }

    public Command extendCommand()  { return startEnd(this::extend,  this::stop).until(this::isExtended); }
    public Command retractCommand() { return startEnd(this::retract, this::stop).until(this::isRetracted); }
    public Command manualExtend()   { return startEnd(this::extend,  this::stop); }
    public Command manualRetract()  { return startEnd(this::retract, this::stop); }
}
`; }

// ─── Buddy Climb (deploy ramp/hook for an alliance partner) ──────────────────

export function tplBuddyClimb(): string { return `package frc.robot.subsystems;

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

public class BuddyClimbSubsystem extends SubsystemBase {

    private static final int    kCanId      = 17;   // TODO: set CAN ID
    private static final double kP          = 0.1;
    private static final double kTolerance  = 0.5;  // rotations

    private static final double kStowed   = 0.0;   // TODO: tune
    private static final double kDeployed = 30.0;  // TODO: tune (ramp fully lowered)

    private final SparkMax m_motor = new SparkMax(kCanId, MotorType.kBrushless);
    private final RelativeEncoder           m_encoder = m_motor.getEncoder();
    private final SparkClosedLoopController m_pid     = m_motor.getClosedLoopController();
    private double m_setpoint = 0;

    public BuddyClimbSubsystem() {
        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.idleMode(IdleMode.kBrake).smartCurrentLimit(30);
        cfg.closedLoop.pid(kP, 0, 0).outputRange(-0.5, 0.5);
        m_motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("BuddyClimb/Position",  m_encoder.getPosition());
        SmartDashboard.putBoolean("BuddyClimb/Deployed", isDeployed());
    }

    private void setPos(double rot) {
        m_setpoint = rot;
        m_pid.setReference(rot, ControlType.kPosition);
    }

    public void deploy() { setPos(kDeployed); }
    public void stow()   { setPos(kStowed);   }

    public boolean isDeployed() { return Math.abs(m_encoder.getPosition() - kDeployed) < kTolerance; }
    public boolean isStowed()   { return Math.abs(m_encoder.getPosition() - kStowed)   < kTolerance; }

    public Command deployCommand() { return runOnce(this::deploy).andThen(run(() -> {}).until(this::isDeployed)); }
    public Command stowCommand()   { return runOnce(this::stow).andThen(run(() -> {}).until(this::isStowed)); }
}
`; }

// ─── Claw / Gripper ───────────────────────────────────────────────────────────

export function tplClaw(): string { return `package frc.robot.subsystems;

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

public class ClawSubsystem extends SubsystemBase {

    private static final int    kCanId      = 18;   // TODO: set CAN ID
    private static final int    kSensorPort = 6;    // TODO: DIO — beam break or limit switch
    private static final double kGripSpeed  = 0.4;  // TODO: tune (inward)
    private static final double kOpenSpeed  = -0.3; // TODO: tune (outward)

    private final SparkMax     m_motor  = new SparkMax(kCanId, MotorType.kBrushless);
    private final DigitalInput m_sensor = new DigitalInput(kSensorPort);

    public ClawSubsystem() {
        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.idleMode(IdleMode.kBrake).smartCurrentLimit(20);
        m_motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Claw/HasPiece", hasPiece());
    }

    public void grip()   { m_motor.set(kGripSpeed);  }
    public void open()   { m_motor.set(kOpenSpeed);  }
    public void hold()   { m_motor.set(kGripSpeed * 0.15); } // low hold current
    public void stop()   { m_motor.stopMotor(); }

    public boolean hasPiece() { return !m_sensor.get(); }

    public Command graspCommand()   { return startEnd(this::grip, this::hold).until(this::hasPiece); }
    public Command releaseCommand() { return startEnd(this::open, this::stop).withTimeout(0.3); }
}
`; }

// ─── Pneumatics Controller ────────────────────────────────────────────────────

export function tplPneumatics(): string { return `package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Manages the REV Pneumatic Hub, compressor, and an example DoubleSolenoid.
 * Duplicate the solenoid pattern for each additional actuator.
 */
public class PneumaticsSubsystem extends SubsystemBase {

    // REV Pneumatic Hub (REVPH) or CTRE PCM (CTREPCM)
    private static final PneumaticsModuleType kModuleType = PneumaticsModuleType.REVPH;

    private static final int kFwdChannel = 0; // TODO: set channel
    private static final int kRevChannel = 1; // TODO: set channel

    private final Compressor    m_compressor = new Compressor(kModuleType);
    private final DoubleSolenoid m_solenoid  = new DoubleSolenoid(kModuleType, kFwdChannel, kRevChannel);

    public PneumaticsSubsystem() {
        m_compressor.enableDigital(); // or enableAnalog(minPSI, maxPSI) for analog sensor
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Pneumatics/Enabled",  m_compressor.isEnabled());
        SmartDashboard.putBoolean("Pneumatics/AtPressure", m_compressor.getPressureSwitchValue());
        SmartDashboard.putNumber("Pneumatics/CurrentAmps", m_compressor.getCurrent());
    }

    public void extend()  { m_solenoid.set(Value.kForward); }
    public void retract() { m_solenoid.set(Value.kReverse); }
    public void off()     { m_solenoid.set(Value.kOff); }

    public void enableCompressor()  { m_compressor.enableDigital(); }
    public void disableCompressor() { m_compressor.disable(); }

    public Command extendCommand()  { return runOnce(this::extend); }
    public Command retractCommand() { return runOnce(this::retract); }
}
`; }
