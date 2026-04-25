// ─── Swerve Module (REV MAXSwerve) ───────────────────────────────────────────

export function tplSwerveModuleRev(): string { return `package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private final SparkMax m_drivingSpark;
    private final SparkMax m_turningSpark;
    private final RelativeEncoder m_drivingEncoder;
    private final AbsoluteEncoder m_turningEncoder;
    private final SparkClosedLoopController m_drivingPID;
    private final SparkClosedLoopController m_turningPID;
    private final double m_chassisAngularOffset;

    public SwerveModule(int drivingCanId, int turningCanId, double chassisAngularOffset) {
        m_drivingSpark = new SparkMax(drivingCanId, MotorType.kBrushless);
        m_turningSpark  = new SparkMax(turningCanId,  MotorType.kBrushless);

        SparkMaxConfig dc = new SparkMaxConfig();
        dc.idleMode(IdleMode.kBrake).smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
        dc.encoder
            .positionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor)
            .velocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);
        dc.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD)
            .velocityFF(ModuleConstants.kDrivingFF).outputRange(-1, 1);
        m_drivingSpark.configure(dc, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig tc = new SparkMaxConfig();
        tc.idleMode(IdleMode.kBrake).smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);
        tc.absoluteEncoder
            .inverted(ModuleConstants.kTurningEncoderInverted)
            .positionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor)
            .velocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);
        tc.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD)
            .outputRange(-1, 1)
            .positionWrappingEnabled(true)
            .positionWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput)
            .positionWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);
        m_turningSpark.configure(tc, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_drivingEncoder       = m_drivingSpark.getEncoder();
        m_turningEncoder       = m_turningSpark.getAbsoluteEncoder();
        m_drivingPID           = m_drivingSpark.getClosedLoopController();
        m_turningPID           = m_turningSpark.getClosedLoopController();
        m_chassisAngularOffset = chassisAngularOffset;
        m_drivingEncoder.setPosition(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            m_drivingEncoder.getVelocity(),
            new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            m_drivingEncoder.getPosition(),
            new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
    }

    public void setDesiredState(SwerveModuleState desired) {
        var corrected = new SwerveModuleState(
            desired.speedMetersPerSecond,
            desired.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset)));
        corrected.optimize(new Rotation2d(m_turningEncoder.getPosition()));
        m_drivingPID.setReference(corrected.speedMetersPerSecond, ControlType.kVelocity);
        m_turningPID.setReference(corrected.angle.getRadians(),   ControlType.kPosition);
    }

    public void resetEncoders() { m_drivingEncoder.setPosition(0); }
}
`; }

// ─── Swerve Module (CTRE TalonFX + CANcoder) ─────────────────────────────────

export function tplSwerveModuleCtre(): string { return `package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private final TalonFX  m_driveTalon;
    private final TalonFX  m_turnTalon;
    private final CANcoder m_cancoder;

    private final VelocityVoltage m_driveReq = new VelocityVoltage(0).withSlot(0);
    private final PositionVoltage m_turnReq  = new PositionVoltage(0).withSlot(0);
    private final double m_chassisAngularOffset;

    public SwerveModule(int driveId, int turnId, int cancoderId, double chassisAngularOffset) {
        m_driveTalon = new TalonFX(driveId);
        m_turnTalon  = new TalonFX(turnId);
        m_cancoder   = new CANcoder(cancoderId);

        CANcoderConfiguration ccCfg = new CANcoderConfiguration();
        ccCfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        m_cancoder.getConfigurator().apply(ccCfg);

        TalonFXConfiguration driveCfg = new TalonFXConfiguration();
        driveCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveCfg.MotorOutput.Inverted    = InvertedValue.CounterClockwise_Positive;
        driveCfg.Slot0.kS = ModuleConstants.kDriveKS;
        driveCfg.Slot0.kV = ModuleConstants.kDriveKV;
        driveCfg.Slot0.kP = ModuleConstants.kDrivingP;
        driveCfg.Feedback.SensorToMechanismRatio = ModuleConstants.kDrivingMotorReduction;
        m_driveTalon.getConfigurator().apply(driveCfg);

        TalonFXConfiguration turnCfg = new TalonFXConfiguration();
        turnCfg.MotorOutput.NeutralMode              = NeutralModeValue.Brake;
        turnCfg.Slot0.kP                             = ModuleConstants.kTurningP;
        turnCfg.Slot0.kD                             = ModuleConstants.kTurningD;
        turnCfg.ClosedLoopGeneral.ContinuousWrap     = true;
        turnCfg.Feedback.FeedbackRemoteSensorID      = cancoderId;
        turnCfg.Feedback.FeedbackSensorSource        = FeedbackSensorSourceValue.RemoteCANcoder;
        m_turnTalon.getConfigurator().apply(turnCfg);

        m_chassisAngularOffset = chassisAngularOffset;
    }

    public SwerveModuleState getState() {
        double vel = m_driveTalon.getVelocity().getValueAsDouble() * ModuleConstants.kWheelCircumferenceMeters;
        double ang = m_cancoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI - m_chassisAngularOffset;
        return new SwerveModuleState(vel, new Rotation2d(ang));
    }

    public SwerveModulePosition getPosition() {
        double pos = m_driveTalon.getPosition().getValueAsDouble() * ModuleConstants.kWheelCircumferenceMeters;
        double ang = m_cancoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI - m_chassisAngularOffset;
        return new SwerveModulePosition(pos, new Rotation2d(ang));
    }

    public void setDesiredState(SwerveModuleState desired) {
        var corrected = new SwerveModuleState(
            desired.speedMetersPerSecond,
            desired.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset)));
        corrected.optimize(Rotation2d.fromRotations(m_cancoder.getAbsolutePosition().getValueAsDouble()));
        m_driveTalon.setControl(m_driveReq.withVelocity(
            corrected.speedMetersPerSecond / ModuleConstants.kWheelCircumferenceMeters));
        m_turnTalon.setControl(m_turnReq.withPosition(corrected.angle.getRotations()));
    }

    public void resetEncoders() { m_driveTalon.setPosition(0); }
}
`; }

// ─── Swerve Drive (NavX) ─────────────────────────────────────────────────────

export function tplSwerveDriveNavx(isRev: boolean): string {
    const mod = isRev
        ? `new SwerveModule(%CID%, %TID%, %OFF%)`
        : `new SwerveModule(%DID%, %TID%, %CCID%, %OFF%)`;
    const fl = isRev
        ? `new SwerveModule(DriveConstants.kFrontLeftDrivingCanId,  DriveConstants.kFrontLeftTurningCanId,  DriveConstants.kFrontLeftChassisAngularOffset)`
        : `new SwerveModule(DriveConstants.kFrontLeftDrivingCanId,  DriveConstants.kFrontLeftTurningCanId,  DriveConstants.kFrontLeftCanCoderId,  DriveConstants.kFrontLeftChassisAngularOffset)`;
    const fr = isRev
        ? `new SwerveModule(DriveConstants.kFrontRightDrivingCanId, DriveConstants.kFrontRightTurningCanId, DriveConstants.kFrontRightChassisAngularOffset)`
        : `new SwerveModule(DriveConstants.kFrontRightDrivingCanId, DriveConstants.kFrontRightTurningCanId, DriveConstants.kFrontRightCanCoderId, DriveConstants.kFrontRightChassisAngularOffset)`;
    const rl = isRev
        ? `new SwerveModule(DriveConstants.kRearLeftDrivingCanId,   DriveConstants.kRearLeftTurningCanId,   DriveConstants.kBackLeftChassisAngularOffset)`
        : `new SwerveModule(DriveConstants.kRearLeftDrivingCanId,   DriveConstants.kRearLeftTurningCanId,   DriveConstants.kRearLeftCanCoderId,   DriveConstants.kBackLeftChassisAngularOffset)`;
    const rr = isRev
        ? `new SwerveModule(DriveConstants.kRearRightDrivingCanId,  DriveConstants.kRearRightTurningCanId,  DriveConstants.kBackRightChassisAngularOffset)`
        : `new SwerveModule(DriveConstants.kRearRightDrivingCanId,  DriveConstants.kRearRightTurningCanId,  DriveConstants.kRearRightCanCoderId,  DriveConstants.kBackRightChassisAngularOffset)`;
    void mod;
    return `package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveDriveSubsystem extends SubsystemBase {

    private final SwerveModule m_frontLeft  = ${fl};
    private final SwerveModule m_frontRight = ${fr};
    private final SwerveModule m_rearLeft   = ${rl};
    private final SwerveModule m_rearRight  = ${rr};

    private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);
    private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics, getGyroYaw(), getModulePositions(), new Pose2d());
    private final Field2d m_field = new Field2d();

    public SwerveDriveSubsystem() { SmartDashboard.putData("Field", m_field); }

    @Override
    public void periodic() {
        m_poseEstimator.update(getGyroYaw(), getModulePositions());
        m_field.setRobotPose(getPose());
        SmartDashboard.putNumber("Drive/HeadingDeg", getHeading());
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        var speeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getGyroYaw())
            : new ChassisSpeeds(xSpeed, ySpeed, rot);
        setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds));
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds));
    }

    public ChassisSpeeds getChassisSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void setX() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    private void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(states[0]);
        m_frontRight.setDesiredState(states[1]);
        m_rearLeft.setDesiredState(states[2]);
        m_rearRight.setDesiredState(states[3]);
    }

    private SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[]{
            m_frontLeft.getState(), m_frontRight.getState(),
            m_rearLeft.getState(),  m_rearRight.getState()};
    }

    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[]{
            m_frontLeft.getPosition(), m_frontRight.getPosition(),
            m_rearLeft.getPosition(),  m_rearRight.getPosition()};
    }

    private Rotation2d getGyroYaw() { return Rotation2d.fromDegrees(-m_gyro.getAngle()); }

    public Pose2d  getPose()    { return m_poseEstimator.getEstimatedPosition(); }
    public double  getHeading() { return getGyroYaw().getDegrees(); }
    public void    zeroHeading(){ m_gyro.reset(); }

    public void resetOdometry(Pose2d pose) {
        m_poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }
    public void addVisionMeasurement(Pose2d visionPose, double ts) {
        m_poseEstimator.addVisionMeasurement(visionPose, ts);
    }
}
`; }

// ─── Swerve Drive (Pigeon 2) ──────────────────────────────────────────────────

export function tplSwerveDrivePigeon(isRev: boolean): string {
    const fl = isRev
        ? `new SwerveModule(DriveConstants.kFrontLeftDrivingCanId,  DriveConstants.kFrontLeftTurningCanId,  DriveConstants.kFrontLeftChassisAngularOffset)`
        : `new SwerveModule(DriveConstants.kFrontLeftDrivingCanId,  DriveConstants.kFrontLeftTurningCanId,  DriveConstants.kFrontLeftCanCoderId,  DriveConstants.kFrontLeftChassisAngularOffset)`;
    const fr = isRev
        ? `new SwerveModule(DriveConstants.kFrontRightDrivingCanId, DriveConstants.kFrontRightTurningCanId, DriveConstants.kFrontRightChassisAngularOffset)`
        : `new SwerveModule(DriveConstants.kFrontRightDrivingCanId, DriveConstants.kFrontRightTurningCanId, DriveConstants.kFrontRightCanCoderId, DriveConstants.kFrontRightChassisAngularOffset)`;
    const rl = isRev
        ? `new SwerveModule(DriveConstants.kRearLeftDrivingCanId,   DriveConstants.kRearLeftTurningCanId,   DriveConstants.kBackLeftChassisAngularOffset)`
        : `new SwerveModule(DriveConstants.kRearLeftDrivingCanId,   DriveConstants.kRearLeftTurningCanId,   DriveConstants.kRearLeftCanCoderId,   DriveConstants.kBackLeftChassisAngularOffset)`;
    const rr = isRev
        ? `new SwerveModule(DriveConstants.kRearRightDrivingCanId,  DriveConstants.kRearRightTurningCanId,  DriveConstants.kBackRightChassisAngularOffset)`
        : `new SwerveModule(DriveConstants.kRearRightDrivingCanId,  DriveConstants.kRearRightTurningCanId,  DriveConstants.kRearRightCanCoderId,  DriveConstants.kBackRightChassisAngularOffset)`;
    return `package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveDriveSubsystem extends SubsystemBase {

    private final SwerveModule m_frontLeft  = ${fl};
    private final SwerveModule m_frontRight = ${fr};
    private final SwerveModule m_rearLeft   = ${rl};
    private final SwerveModule m_rearRight  = ${rr};

    private final Pigeon2 m_gyro = new Pigeon2(DriveConstants.kPigeonCanId);
    private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics, getGyroYaw(), getModulePositions(), new Pose2d());
    private final Field2d m_field = new Field2d();

    public SwerveDriveSubsystem() {
        m_gyro.reset();
        SmartDashboard.putData("Field", m_field);
    }

    @Override
    public void periodic() {
        m_poseEstimator.update(getGyroYaw(), getModulePositions());
        m_field.setRobotPose(getPose());
        SmartDashboard.putNumber("Drive/HeadingDeg", getHeading());
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        var speeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getGyroYaw())
            : new ChassisSpeeds(xSpeed, ySpeed, rot);
        setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds));
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds));
    }

    public ChassisSpeeds getChassisSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void setX() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    private void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(states[0]);
        m_frontRight.setDesiredState(states[1]);
        m_rearLeft.setDesiredState(states[2]);
        m_rearRight.setDesiredState(states[3]);
    }

    private SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[]{
            m_frontLeft.getState(), m_frontRight.getState(),
            m_rearLeft.getState(),  m_rearRight.getState()};
    }

    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[]{
            m_frontLeft.getPosition(), m_frontRight.getPosition(),
            m_rearLeft.getPosition(),  m_rearRight.getPosition()};
    }

    private Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(-m_gyro.getYaw().getValueAsDouble());
    }

    public Pose2d  getPose()    { return m_poseEstimator.getEstimatedPosition(); }
    public double  getHeading() { return getGyroYaw().getDegrees(); }
    public void    zeroHeading(){ m_gyro.reset(); }

    public void resetOdometry(Pose2d pose) {
        m_poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }
    public void addVisionMeasurement(Pose2d visionPose, double ts) {
        m_poseEstimator.addVisionMeasurement(visionPose, ts);
    }
}
`; }

// ─── Tank Drive (REV) ─────────────────────────────────────────────────────────

export function tplTankRev(): string { return `package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class TankDriveSubsystem extends SubsystemBase {

    private final SparkMax m_leftLeader  = new SparkMax(DriveConstants.kLeftMotor1Port,  MotorType.kBrushless);
    private final SparkMax m_leftFollow  = new SparkMax(DriveConstants.kLeftMotor2Port,  MotorType.kBrushless);
    private final SparkMax m_rightLeader = new SparkMax(DriveConstants.kRightMotor1Port, MotorType.kBrushless);
    private final SparkMax m_rightFollow = new SparkMax(DriveConstants.kRightMotor2Port, MotorType.kBrushless);

    private final DifferentialDrive m_drive = new DifferentialDrive(m_leftLeader::set, m_rightLeader::set);
    private final RelativeEncoder m_leftEnc  = m_leftLeader.getEncoder();
    private final RelativeEncoder m_rightEnc = m_rightLeader.getEncoder();
    private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);
    private final DifferentialDriveOdometry m_odometry;
    private final Field2d m_field = new Field2d();

    public TankDriveSubsystem() {
        SparkMaxConfig lc = new SparkMaxConfig();
        lc.idleMode(IdleMode.kBrake).smartCurrentLimit(50);
        lc.encoder
            .positionConversionFactor(DriveConstants.kLinearDistanceConversionFactor)
            .velocityConversionFactor(DriveConstants.kLinearDistanceConversionFactor / 60.0);
        m_leftLeader.configure(lc, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        SparkMaxConfig lf = new SparkMaxConfig(); lf.follow(m_leftLeader);
        m_leftFollow.configure(lf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig rc = new SparkMaxConfig();
        rc.idleMode(IdleMode.kBrake).smartCurrentLimit(50).inverted(true);
        rc.encoder
            .positionConversionFactor(DriveConstants.kLinearDistanceConversionFactor)
            .velocityConversionFactor(DriveConstants.kLinearDistanceConversionFactor / 60.0);
        m_rightLeader.configure(rc, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        SparkMaxConfig rf = new SparkMaxConfig(); rf.follow(m_rightLeader);
        m_rightFollow.configure(rf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        resetEncoders();
        m_odometry = new DifferentialDriveOdometry(
            m_gyro.getRotation2d(), m_leftEnc.getPosition(), m_rightEnc.getPosition());
        SmartDashboard.putData("Field", m_field);
    }

    @Override
    public void periodic() {
        m_odometry.update(m_gyro.getRotation2d(), m_leftEnc.getPosition(), m_rightEnc.getPosition());
        m_field.setRobotPose(getPose());
    }

    public void arcadeDrive(double fwd, double rot) { m_drive.arcadeDrive(fwd, rot); }
    public void tankDrive(double left, double right) { m_drive.tankDrive(left, right); }
    public void tankDriveVolts(double lv, double rv) {
        m_leftLeader.setVoltage(lv); m_rightLeader.setVoltage(rv); m_drive.feed();
    }
    public void resetEncoders() { m_leftEnc.setPosition(0); m_rightEnc.setPosition(0); }
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(m_leftEnc.getVelocity(), m_rightEnc.getVelocity());
    }
    public Pose2d getPose() { return m_odometry.getPoseMeters(); }
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(m_gyro.getRotation2d(), 0, 0, pose);
    }
    public void zeroHeading() { m_gyro.reset(); }
    public double getHeading() { return m_gyro.getRotation2d().getDegrees(); }
}
`; }

// ─── Tank Drive (CTRE) ────────────────────────────────────────────────────────

export function tplTankCtre(): string { return `package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class TankDriveSubsystem extends SubsystemBase {

    private final TalonFX m_leftLeader  = new TalonFX(DriveConstants.kLeftMotor1Port);
    private final TalonFX m_leftFollow  = new TalonFX(DriveConstants.kLeftMotor2Port);
    private final TalonFX m_rightLeader = new TalonFX(DriveConstants.kRightMotor1Port);
    private final TalonFX m_rightFollow = new TalonFX(DriveConstants.kRightMotor2Port);
    private final DifferentialDrive m_drive = new DifferentialDrive(
        v -> m_leftLeader.set(v), v -> m_rightLeader.set(v));
    private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);
    private final DifferentialDriveOdometry m_odometry;
    private final Field2d m_field = new Field2d();

    public TankDriveSubsystem() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfg.Feedback.SensorToMechanismRatio = DriveConstants.kGearRatio;
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        m_leftLeader.getConfigurator().apply(cfg);
        m_leftFollow.setControl(new Follower(DriveConstants.kLeftMotor1Port, false));
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_rightLeader.getConfigurator().apply(cfg);
        m_rightFollow.setControl(new Follower(DriveConstants.kRightMotor1Port, false));
        resetEncoders();
        m_odometry = new DifferentialDriveOdometry(
            m_gyro.getRotation2d(), getLeftMeters(), getRightMeters());
        SmartDashboard.putData("Field", m_field);
    }

    @Override
    public void periodic() {
        m_odometry.update(m_gyro.getRotation2d(), getLeftMeters(), getRightMeters());
        m_field.setRobotPose(getPose());
    }

    private double getLeftMeters()  {
        return m_leftLeader.getPosition().getValueAsDouble()  / DriveConstants.kGearRatio * DriveConstants.kWheelCircumferenceMeters;
    }
    private double getRightMeters() {
        return m_rightLeader.getPosition().getValueAsDouble() / DriveConstants.kGearRatio * DriveConstants.kWheelCircumferenceMeters;
    }
    public void arcadeDrive(double fwd, double rot) { m_drive.arcadeDrive(fwd, rot); }
    public void tankDrive(double left, double right) { m_drive.tankDrive(left, right); }
    public void resetEncoders() { m_leftLeader.setPosition(0); m_rightLeader.setPosition(0); }
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        double lv = m_leftLeader.getVelocity().getValueAsDouble()  / DriveConstants.kGearRatio * DriveConstants.kWheelCircumferenceMeters;
        double rv = m_rightLeader.getVelocity().getValueAsDouble() / DriveConstants.kGearRatio * DriveConstants.kWheelCircumferenceMeters;
        return new DifferentialDriveWheelSpeeds(lv, rv);
    }
    public Pose2d getPose() { return m_odometry.getPoseMeters(); }
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(m_gyro.getRotation2d(), 0, 0, pose);
    }
    public void zeroHeading() { m_gyro.reset(); }
    public double getHeading() { return m_gyro.getRotation2d().getDegrees(); }
}
`; }

// ─── Mecanum Drive (REV) ──────────────────────────────────────────────────────

export function tplMecanum(): string { return `package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class MecanumDriveSubsystem extends SubsystemBase {

    private final SparkMax m_frontLeft  = new SparkMax(DriveConstants.kFrontLeftCanId,  MotorType.kBrushless);
    private final SparkMax m_frontRight = new SparkMax(DriveConstants.kFrontRightCanId, MotorType.kBrushless);
    private final SparkMax m_rearLeft   = new SparkMax(DriveConstants.kRearLeftCanId,   MotorType.kBrushless);
    private final SparkMax m_rearRight  = new SparkMax(DriveConstants.kRearRightCanId,  MotorType.kBrushless);

    private final MecanumDrive m_drive = new MecanumDrive(
        m_frontLeft::set, m_rearLeft::set, m_frontRight::set, m_rearRight::set);

    public MecanumDriveSubsystem() {
        SparkMaxConfig lCfg = new SparkMaxConfig();
        lCfg.idleMode(IdleMode.kBrake).smartCurrentLimit(50);
        m_frontLeft.configure(lCfg,  ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_rearLeft.configure(lCfg,   ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig rCfg = new SparkMaxConfig();
        rCfg.idleMode(IdleMode.kBrake).smartCurrentLimit(50).inverted(true);
        m_frontRight.configure(rCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_rearRight.configure(rCfg,  ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Drive/FLOutput", m_frontLeft.getAppliedOutput());
    }

    /** xSpeed: forward, ySpeed: strafe, zRotation: rotate. All -1..1. */
    public void driveCartesian(double xSpeed, double ySpeed, double zRotation) {
        m_drive.driveCartesian(xSpeed, ySpeed, zRotation);
    }

    /** Field-relative cartesian drive — pass current robot heading. */
    public void driveCartesianFieldRelative(double xSpeed, double ySpeed, double zRotation, Rotation2d robotAngle) {
        m_drive.driveCartesian(xSpeed, ySpeed, zRotation, robotAngle);
    }

    public void stop() { m_drive.stopMotor(); }
}
`; }
