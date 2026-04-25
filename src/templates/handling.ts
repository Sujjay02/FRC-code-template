// ─── Roller Intake ────────────────────────────────────────────────────────────

export function tplIntake(): string { return `package frc.robot.subsystems;

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

public class IntakeSubsystem extends SubsystemBase {

    private static final int    kRollerCanId = 3;    // TODO: set CAN ID
    private static final int    kSensorPort  = 1;    // TODO: set DIO port
    private static final double kIntakeSpeed = 0.8;  // TODO: tune
    private static final double kEjectSpeed  = -0.5; // TODO: tune

    private final SparkMax     m_roller = new SparkMax(kRollerCanId, MotorType.kBrushless);
    private final DigitalInput m_sensor = new DigitalInput(kSensorPort);

    public IntakeSubsystem() {
        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.idleMode(IdleMode.kBrake).smartCurrentLimit(30);
        m_roller.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Intake/HasPiece", hasPiece());
    }

    public void intake() { m_roller.set(kIntakeSpeed); }
    public void eject()  { m_roller.set(kEjectSpeed); }
    public void stop()   { m_roller.stopMotor(); }
    public boolean hasPiece() { return !m_sensor.get(); }

    public Command intakeCommand() { return startEnd(this::intake, this::stop).until(this::hasPiece); }
    public Command ejectCommand()  { return startEnd(this::eject,  this::stop); }
}
`; }

// ─── Intake with Deploy Arm ───────────────────────────────────────────────────

export function tplDeployIntake(): string { return `package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    private static final int    kArmCanId    = 3;    // TODO: set CAN ID
    private static final int    kRollerCanId = 4;    // TODO: set CAN ID
    private static final int    kSensorPort  = 0;    // TODO: set DIO port
    private static final double kP           = 0.1;
    private static final double kTolerance   = 0.5;  // rotations

    private static final double kStowed   = 0.0;   // TODO: tune (rotations)
    private static final double kDeployed = 25.0;  // TODO: tune

    private static final double kIntakeSpeed = 0.8;
    private static final double kEjectSpeed  = -0.5;

    private final SparkMax m_arm    = new SparkMax(kArmCanId,    MotorType.kBrushless);
    private final SparkMax m_roller = new SparkMax(kRollerCanId, MotorType.kBrushless);
    private final RelativeEncoder           m_armEnc = m_arm.getEncoder();
    private final SparkClosedLoopController m_armPID = m_arm.getClosedLoopController();
    private final DigitalInput              m_sensor = new DigitalInput(kSensorPort);

    public IntakeSubsystem() {
        SparkMaxConfig armCfg = new SparkMaxConfig();
        armCfg.idleMode(IdleMode.kBrake).smartCurrentLimit(30);
        armCfg.closedLoop.pid(kP, 0, 0).outputRange(-0.5, 0.5);
        m_arm.configure(armCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig rollerCfg = new SparkMaxConfig();
        rollerCfg.idleMode(IdleMode.kBrake).smartCurrentLimit(20);
        m_roller.configure(rollerCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/ArmPos",  m_armEnc.getPosition());
        SmartDashboard.putBoolean("Intake/HasPiece", hasPiece());
        SmartDashboard.putBoolean("Intake/Deployed", isDeployed());
    }

    public void deploy()       { setArmPos(kDeployed); }
    public void stow()         { setArmPos(kStowed);   }
    public void intake()       { m_roller.set(kIntakeSpeed); }
    public void eject()        { m_roller.set(kEjectSpeed);  }
    public void stopRollers()  { m_roller.stopMotor(); }

    private void setArmPos(double rot) { m_armPID.setReference(rot, ControlType.kPosition); }

    public boolean isDeployed() { return Math.abs(m_armEnc.getPosition() - kDeployed) < kTolerance; }
    public boolean isStowed()   { return Math.abs(m_armEnc.getPosition() - kStowed)   < kTolerance; }
    public boolean hasPiece()   { return !m_sensor.get(); }

    public Command deployAndIntakeCommand() {
        return runOnce(this::deploy)
            .andThen(run(() -> {}).until(this::isDeployed))
            .andThen(startEnd(this::intake, this::stopRollers).until(this::hasPiece));
    }

    public Command stowCommand() {
        return runOnce(this::stopRollers)
            .andThen(runOnce(this::stow))
            .andThen(run(() -> {}).until(this::isStowed));
    }

    public Command ejectCommand() { return startEnd(this::eject, this::stopRollers); }
}
`; }

// ─── Conveyor / Belt Transport ────────────────────────────────────────────────

export function tplConveyor(): string { return `package frc.robot.subsystems;

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

public class ConveyorSubsystem extends SubsystemBase {

    private static final int    kCanId      = 10;  // TODO: set CAN ID
    private static final int    kSensorPort = 3;   // TODO: set DIO port
    private static final double kSpeed      = 0.7; // TODO: tune

    private final SparkMax     m_motor  = new SparkMax(kCanId, MotorType.kBrushless);
    private final DigitalInput m_sensor = new DigitalInput(kSensorPort);

    public ConveyorSubsystem() {
        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.idleMode(IdleMode.kBrake).smartCurrentLimit(25);
        m_motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Conveyor/HasPiece", hasPiece());
    }

    public void run()     { m_motor.set(kSpeed);  }
    public void reverse() { m_motor.set(-kSpeed); }
    public void stop()    { m_motor.stopMotor();   }
    public boolean hasPiece() { return !m_sensor.get(); }

    public Command runCommand()     { return startEnd(this::run,     this::stop); }
    public Command reverseCommand() { return startEnd(this::reverse, this::stop); }
    public Command feedToTopCommand() { return startEnd(this::run, this::stop).until(this::hasPiece); }
}
`; }

// ─── Serializer (two-zone indexer) ───────────────────────────────────────────

export function tplSerializer(): string { return `package frc.robot.subsystems;

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

public class SerializerSubsystem extends SubsystemBase {

    private static final int    kLowerCanId  = 11;  // TODO: set CAN ID
    private static final int    kUpperCanId  = 12;  // TODO: set CAN ID
    private static final int    kLowerDIO    = 4;   // TODO: set DIO port
    private static final int    kUpperDIO    = 5;   // TODO: set DIO port
    private static final double kSpeed       = 0.6; // TODO: tune

    private final SparkMax     m_lower       = new SparkMax(kLowerCanId, MotorType.kBrushless);
    private final SparkMax     m_upper       = new SparkMax(kUpperCanId, MotorType.kBrushless);
    private final DigitalInput m_lowerSensor = new DigitalInput(kLowerDIO);
    private final DigitalInput m_upperSensor = new DigitalInput(kUpperDIO);

    public SerializerSubsystem() {
        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.idleMode(IdleMode.kBrake).smartCurrentLimit(25);
        m_lower.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_upper.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Serializer/LowerZone", lowerHasPiece());
        SmartDashboard.putBoolean("Serializer/UpperZone", upperHasPiece());
        SmartDashboard.putBoolean("Serializer/ReadyToShoot", readyToShoot());
    }

    public void feedAll()   { m_lower.set(kSpeed);  m_upper.set(kSpeed);  }
    public void feedLower() { m_lower.set(kSpeed);  m_upper.stopMotor();  }
    public void feedUpper() { m_lower.stopMotor();  m_upper.set(kSpeed);  }
    public void eject()     { m_lower.set(-kSpeed); m_upper.set(-kSpeed); }
    public void stopAll()   { m_lower.stopMotor();  m_upper.stopMotor();  }

    public boolean lowerHasPiece()  { return !m_lowerSensor.get(); }
    public boolean upperHasPiece()  { return !m_upperSensor.get(); }
    public boolean readyToShoot()   { return upperHasPiece(); }

    public Command feedToShooterCommand() { return startEnd(this::feedAll, this::stopAll).until(this::readyToShoot); }
    public Command ejectCommand()         { return startEnd(this::eject,   this::stopAll); }
}
`; }
