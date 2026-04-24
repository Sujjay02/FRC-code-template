import * as vscode from 'vscode';
import * as fs from 'fs';
import * as path from 'path';

// ─── helpers ─────────────────────────────────────────────────────────────────

function findRobotDir(wsRoot: string): string | undefined {
    const hasBuild    = fs.existsSync(path.join(wsRoot, 'build.gradle'));
    const hasPrefs    = fs.existsSync(path.join(wsRoot, '.wpilib', 'wpilib_preferences.json'));
    const hasSrc      = fs.existsSync(path.join(wsRoot, 'src', 'main', 'java', 'frc', 'robot'));
    if (!hasBuild || (!hasPrefs && !hasSrc)) { return undefined; }
    return path.join(wsRoot, 'src', 'main', 'java', 'frc', 'robot');
}

async function requireRobotDir(): Promise<string | undefined> {
    const folders = vscode.workspace.workspaceFolders;
    if (!folders?.length) {
        vscode.window.showErrorMessage('Open a WPILib project folder first.');
        return undefined;
    }
    const dir = findRobotDir(folders[0].uri.fsPath);
    if (!dir) {
        vscode.window.showErrorMessage('No WPILib project detected (missing build.gradle).');
        return undefined;
    }
    return dir;
}

async function writeSafe(filePath: string, content: string): Promise<boolean> {
    if (fs.existsSync(filePath)) {
        const ans = await vscode.window.showWarningMessage(
            `${path.basename(filePath)} already exists — overwrite?`, { modal: true }, 'Overwrite');
        if (ans !== 'Overwrite') { return false; }
    }
    fs.mkdirSync(path.dirname(filePath), { recursive: true });
    fs.writeFileSync(filePath, content, 'utf8');
    return true;
}

async function reveal(filePath: string) {
    const doc = await vscode.workspace.openTextDocument(vscode.Uri.file(filePath));
    vscode.window.showTextDocument(doc, { preview: false });
}

function qp(items: string[], placeholder: string) {
    return vscode.window.showQuickPick(items, { placeHolder: placeholder });
}

// ─── swerve ──────────────────────────────────────────────────────────────────

function tplSwerveModuleRev(): string { return `package frc.robot.subsystems;

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
    private SwerveModuleState m_desired = new SwerveModuleState();

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
        m_desired.angle        = new Rotation2d(m_turningEncoder.getPosition());
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
        m_desired = desired;
    }

    public void resetEncoders() { m_drivingEncoder.setPosition(0); }
}
`; }

function tplSwerveModuleCtre(): string { return `package frc.robot.subsystems;

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

function tplSwerveDriveNavx(isRev: boolean): string {
    const ctor = isRev
        ? `new SwerveModule(DriveConstants.kFrontLeftDrivingCanId,  DriveConstants.kFrontLeftTurningCanId,  DriveConstants.kFrontLeftChassisAngularOffset)`
        : `new SwerveModule(DriveConstants.kFrontLeftDrivingCanId,  DriveConstants.kFrontLeftTurningCanId,  DriveConstants.kFrontLeftCanCoderId,  DriveConstants.kFrontLeftChassisAngularOffset)`;
    const frCtor = isRev
        ? `new SwerveModule(DriveConstants.kFrontRightDrivingCanId, DriveConstants.kFrontRightTurningCanId, DriveConstants.kFrontRightChassisAngularOffset)`
        : `new SwerveModule(DriveConstants.kFrontRightDrivingCanId, DriveConstants.kFrontRightTurningCanId, DriveConstants.kFrontRightCanCoderId, DriveConstants.kFrontRightChassisAngularOffset)`;
    const rlCtor = isRev
        ? `new SwerveModule(DriveConstants.kRearLeftDrivingCanId,   DriveConstants.kRearLeftTurningCanId,   DriveConstants.kBackLeftChassisAngularOffset)`
        : `new SwerveModule(DriveConstants.kRearLeftDrivingCanId,   DriveConstants.kRearLeftTurningCanId,   DriveConstants.kRearLeftCanCoderId,   DriveConstants.kBackLeftChassisAngularOffset)`;
    const rrCtor = isRev
        ? `new SwerveModule(DriveConstants.kRearRightDrivingCanId,  DriveConstants.kRearRightTurningCanId,  DriveConstants.kBackRightChassisAngularOffset)`
        : `new SwerveModule(DriveConstants.kRearRightDrivingCanId,  DriveConstants.kRearRightTurningCanId,  DriveConstants.kRearRightCanCoderId,  DriveConstants.kBackRightChassisAngularOffset)`;
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

    private final SwerveModule m_frontLeft  = ${ctor};
    private final SwerveModule m_frontRight = ${frCtor};
    private final SwerveModule m_rearLeft   = ${rlCtor};
    private final SwerveModule m_rearRight  = ${rrCtor};

    private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

    private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        getGyroYaw(),
        getModulePositions(),
        new Pose2d());

    private final Field2d m_field = new Field2d();

    public SwerveDriveSubsystem() {
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
        return Rotation2d.fromDegrees(-m_gyro.getAngle());
    }

    public Pose2d getPose() { return m_poseEstimator.getEstimatedPosition(); }

    public void resetOdometry(Pose2d pose) {
        m_poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds) {
        m_poseEstimator.addVisionMeasurement(visionPose, timestampSeconds);
    }

    public void zeroHeading() { m_gyro.reset(); }
    public double getHeading() { return getGyroYaw().getDegrees(); }
}
`; }

function tplSwerveDrivePigeon(isRev: boolean): string {
    const ctor = isRev
        ? `new SwerveModule(DriveConstants.kFrontLeftDrivingCanId,  DriveConstants.kFrontLeftTurningCanId,  DriveConstants.kFrontLeftChassisAngularOffset)`
        : `new SwerveModule(DriveConstants.kFrontLeftDrivingCanId,  DriveConstants.kFrontLeftTurningCanId,  DriveConstants.kFrontLeftCanCoderId,  DriveConstants.kFrontLeftChassisAngularOffset)`;
    const frCtor = isRev
        ? `new SwerveModule(DriveConstants.kFrontRightDrivingCanId, DriveConstants.kFrontRightTurningCanId, DriveConstants.kFrontRightChassisAngularOffset)`
        : `new SwerveModule(DriveConstants.kFrontRightDrivingCanId, DriveConstants.kFrontRightTurningCanId, DriveConstants.kFrontRightCanCoderId, DriveConstants.kFrontRightChassisAngularOffset)`;
    const rlCtor = isRev
        ? `new SwerveModule(DriveConstants.kRearLeftDrivingCanId,   DriveConstants.kRearLeftTurningCanId,   DriveConstants.kBackLeftChassisAngularOffset)`
        : `new SwerveModule(DriveConstants.kRearLeftDrivingCanId,   DriveConstants.kRearLeftTurningCanId,   DriveConstants.kRearLeftCanCoderId,   DriveConstants.kBackLeftChassisAngularOffset)`;
    const rrCtor = isRev
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

    private final SwerveModule m_frontLeft  = ${ctor};
    private final SwerveModule m_frontRight = ${frCtor};
    private final SwerveModule m_rearLeft   = ${rlCtor};
    private final SwerveModule m_rearRight  = ${rrCtor};

    private final Pigeon2 m_gyro = new Pigeon2(DriveConstants.kPigeonCanId);

    private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        getGyroYaw(),
        getModulePositions(),
        new Pose2d());

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

    public Pose2d getPose() { return m_poseEstimator.getEstimatedPosition(); }

    public void resetOdometry(Pose2d pose) {
        m_poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds) {
        m_poseEstimator.addVisionMeasurement(visionPose, timestampSeconds);
    }

    public void zeroHeading() { m_gyro.reset(); }
    public double getHeading() { return getGyroYaw().getDegrees(); }
}
`; }

// ─── tank ────────────────────────────────────────────────────────────────────

function tplTankRev(): string { return `package frc.robot.subsystems;

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

    private final SparkMax m_leftLeader   = new SparkMax(DriveConstants.kLeftMotor1Port,  MotorType.kBrushless);
    private final SparkMax m_leftFollow   = new SparkMax(DriveConstants.kLeftMotor2Port,  MotorType.kBrushless);
    private final SparkMax m_rightLeader  = new SparkMax(DriveConstants.kRightMotor1Port, MotorType.kBrushless);
    private final SparkMax m_rightFollow  = new SparkMax(DriveConstants.kRightMotor2Port, MotorType.kBrushless);

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

        SparkMaxConfig lf = new SparkMaxConfig();
        lf.follow(m_leftLeader);
        m_leftFollow.configure(lf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig rc = new SparkMaxConfig();
        rc.idleMode(IdleMode.kBrake).smartCurrentLimit(50).inverted(true);
        rc.encoder
            .positionConversionFactor(DriveConstants.kLinearDistanceConversionFactor)
            .velocityConversionFactor(DriveConstants.kLinearDistanceConversionFactor / 60.0);
        m_rightLeader.configure(rc, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig rf = new SparkMaxConfig();
        rf.follow(m_rightLeader);
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

    public void tankDriveVolts(double leftV, double rightV) {
        m_leftLeader.setVoltage(leftV);
        m_rightLeader.setVoltage(rightV);
        m_drive.feed();
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

    public double getAverageDistance() {
        return (m_leftEnc.getPosition() + m_rightEnc.getPosition()) / 2.0;
    }

    public void zeroHeading() { m_gyro.reset(); }
    public double getHeading() { return m_gyro.getRotation2d().getDegrees(); }
}
`; }

function tplTankCtre(): string { return `package frc.robot.subsystems;

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

    private double getLeftMeters() {
        return m_leftLeader.getPosition().getValueAsDouble()
               / DriveConstants.kGearRatio * DriveConstants.kWheelCircumferenceMeters;
    }

    private double getRightMeters() {
        return m_rightLeader.getPosition().getValueAsDouble()
               / DriveConstants.kGearRatio * DriveConstants.kWheelCircumferenceMeters;
    }

    public void arcadeDrive(double fwd, double rot) { m_drive.arcadeDrive(fwd, rot); }
    public void tankDrive(double left, double right) { m_drive.tankDrive(left, right); }

    public void resetEncoders() {
        m_leftLeader.setPosition(0);
        m_rightLeader.setPosition(0);
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        double lv = m_leftLeader.getVelocity().getValueAsDouble()
                    / DriveConstants.kGearRatio * DriveConstants.kWheelCircumferenceMeters;
        double rv = m_rightLeader.getVelocity().getValueAsDouble()
                    / DriveConstants.kGearRatio * DriveConstants.kWheelCircumferenceMeters;
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

// ─── mechanism ───────────────────────────────────────────────────────────────

function tplMechanism(name: string, presets: string[]): string {
    const presetFields = presets.map((p, i) =>
        `    private static final double k${p.charAt(0).toUpperCase()}${p.slice(1)} = ${i === 0 ? 0.0 : (i * 10.0)}; // TODO`
    ).join('\n');

    const presetMethods = presets.map(p => {
        const cap = p.charAt(0).toUpperCase() + p.slice(1);
        return `    public void goTo${cap}() { setSetpoint(k${cap}); }`;
    }).join('\n');

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

    private static final int    kMotorCanId   = 0;    // TODO: set CAN ID
    private static final double kP            = 0.1;
    private static final double kI            = 0.0;
    private static final double kD            = 0.0;
    private static final double kFF           = 0.0;
    private static final double kTolerance    = 0.5;  // rotations

${presetFields}

    private final SparkMax m_motor = new SparkMax(kMotorCanId, MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_motor.getEncoder();
    private final SparkClosedLoopController m_pid = m_motor.getClosedLoopController();

    private double m_setpoint = 0;

    public ${name}Subsystem() {
        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
        cfg.closedLoop.pid(kP, kI, kD).velocityFF(kFF).outputRange(-1, 1);
        m_motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("${name}/Position", m_encoder.getPosition());
        SmartDashboard.putNumber("${name}/Setpoint", m_setpoint);
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

    // ── preset shortcuts ──
${presetMethods}

    // ── command factories ──
${presets.map(p => {
    const cap = p.charAt(0).toUpperCase() + p.slice(1);
    return `    public Command goTo${cap}Command() { return runOnce(() -> goTo${cap}()).andThen(run(() -> {}).until(this::atSetpoint)); }`;
}).join('\n')}
}
`; }

// ─── LEDs ────────────────────────────────────────────────────────────────────

function tplLeds(): string { return `package frc.robot.subsystems;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

    private static final int kPort      = 9;   // TODO: set PWM port
    private static final int kLength    = 60;  // TODO: set strip length

    private final AddressableLED m_led         = new AddressableLED(kPort);
    private final AddressableLEDBuffer m_buffer = new AddressableLEDBuffer(kLength);

    public LEDSubsystem() {
        m_led.setLength(kLength);
        m_led.setData(m_buffer);
        m_led.start();
    }

    @Override
    public void periodic() {
        m_led.setData(m_buffer);
    }

    // ── patterns ──

    public void setSolid(Color color) {
        LEDPattern.solid(color).applyTo(m_buffer);
    }

    public void setRainbow() {
        LEDPattern.rainbow(255, 128).applyTo(m_buffer);
    }

    public void setBreathe(Color color) {
        LEDPattern.solid(color).breathe(Units.Seconds.of(2)).applyTo(m_buffer);
    }

    public void setStrobe(Color color, double periodSeconds) {
        LEDPattern.solid(color).blink(Units.Seconds.of(periodSeconds)).applyTo(m_buffer);
    }

    public void setChase(Color color) {
        LEDPattern.solid(color).scrollAtRelativeSpeed(Units.Percent.of(25)).applyTo(m_buffer);
    }

    public void setProgressBar(Color color, double fraction) {
        LEDPattern.solid(color).mask(LEDPattern.progressMaskLayer(() -> fraction)).applyTo(m_buffer);
    }

    public void setAllianceColor(boolean isRed) {
        setSolid(isRed ? Color.kRed : Color.kBlue);
    }

    public void setIndicator(boolean condition) {
        setSolid(condition ? Color.kGreen : Color.kRed);
    }

    public void setOff() {
        LEDPattern.solid(Color.kBlack).applyTo(m_buffer);
    }

    // ── command factories ──

    public Command solidCommand(Color color)       { return runOnce(() -> setSolid(color)); }
    public Command rainbowCommand()                 { return run(this::setRainbow); }
    public Command breatheCommand(Color color)      { return run(() -> setBreathe(color)); }
    public Command offCommand()                     { return runOnce(this::setOff); }
    public Command allianceCommand(boolean isRed)   { return runOnce(() -> setAllianceColor(isRed)); }
}
`; }

// ─── vision ──────────────────────────────────────────────────────────────────

function tplVisionPhoton(): string { return `package frc.robot.subsystems;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    private static final String kCameraName = "photonvision"; // TODO: match PhotonVision camera name
    private static final Transform3d kRobotToCamera = new Transform3d(
        new Translation3d(0.5, 0.0, 0.5),
        new Rotation3d(0, Math.toRadians(-15), 0));

    private static final double kYawP  = 0.04;
    private static final double kYawTolerance = 1.0; // degrees

    private final PhotonCamera m_camera = new PhotonCamera(kCameraName);
    private final AprilTagFieldLayout m_layout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026Reefscape);
    private final PhotonPoseEstimator m_estimator = new PhotonPoseEstimator(
        m_layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCamera);

    private final PIDController m_yawPID = new PIDController(kYawP, 0, 0);

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Vision/HasTargets", hasTargets());
        getBestTarget().ifPresent(t -> {
            SmartDashboard.putNumber("Vision/TargetYaw",   t.getYaw());
            SmartDashboard.putNumber("Vision/TargetPitch", t.getPitch());
            SmartDashboard.putNumber("Vision/TagId",       t.getFiducialId());
        });
    }

    public boolean hasTargets() {
        return m_camera.getLatestResult().hasTargets();
    }

    public Optional<PhotonTrackedTarget> getBestTarget() {
        var r = m_camera.getLatestResult();
        return r.hasTargets() ? Optional.of(r.getBestTarget()) : Optional.empty();
    }

    public Optional<EstimatedRobotPose> getEstimatedPose(Pose2d prevPose) {
        m_estimator.setReferencePose(prevPose);
        return m_estimator.update(m_camera.getLatestResult());
    }

    public double getYawToTarget() {
        return getBestTarget().map(PhotonTrackedTarget::getYaw).orElse(0.0);
    }

    /**
     * Returns a rotation output (rad/s) to drive the robot toward the target yaw.
     * Pass this as the rot parameter of your drive method.
     */
    public double alignRotationOutput() {
        m_yawPID.setSetpoint(0);
        m_yawPID.setTolerance(kYawTolerance);
        return -m_yawPID.calculate(getYawToTarget());
    }

    public boolean isAligned() { return m_yawPID.atSetpoint() && hasTargets(); }

    /** Command that rotates the robot until the best target is centered. */
    public Command alignToTargetCommand(java.util.function.DoubleConsumer rotConsumer) {
        return run(() -> rotConsumer.accept(hasTargets() ? alignRotationOutput() : 0))
            .until(this::isAligned)
            .withName("AlignToTarget");
    }
}
`; }

function tplVisionLimelight(): string { return `package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    private static final String kTableName    = "limelight"; // TODO: match Limelight hostname
    private static final double kYawP         = 0.04;
    private static final double kYawTolerance = 1.0; // degrees

    private final NetworkTable m_table =
        NetworkTableInstance.getDefault().getTable(kTableName);
    private final PIDController m_yawPID = new PIDController(kYawP, 0, 0);

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Vision/HasTargets", hasTargets());
        if (hasTargets()) {
            SmartDashboard.putNumber("Vision/TX", getTx());
            SmartDashboard.putNumber("Vision/TY", getTy());
            SmartDashboard.putNumber("Vision/TagId", getTagId());
        }
    }

    private double entry(String key) {
        return m_table.getEntry(key).getDouble(0.0);
    }

    public boolean hasTargets()   { return entry("tv") == 1.0; }
    public double  getTx()        { return entry("tx"); }
    public double  getTy()        { return entry("ty"); }
    public double  getTa()        { return entry("ta"); }
    public int     getTagId()     { return (int) entry("tid"); }

    public void setPipelineIndex(int idx) {
        m_table.getEntry("pipeline").setNumber(idx);
    }

    public Pose2d getBotPose() {
        double[] raw = m_table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        if (raw.length < 6) { return new Pose2d(); }
        return new Pose2d(raw[0], raw[1],
            edu.wpi.first.math.geometry.Rotation2d.fromDegrees(raw[5]));
    }

    public double getLatencySeconds() {
        return (entry("tl") + entry("cl")) / 1000.0;
    }

    public double alignRotationOutput() {
        m_yawPID.setSetpoint(0);
        m_yawPID.setTolerance(kYawTolerance);
        return -m_yawPID.calculate(getTx());
    }

    public boolean isAligned() { return m_yawPID.atSetpoint() && hasTargets(); }

    public Command alignToTargetCommand(java.util.function.DoubleConsumer rotConsumer) {
        return run(() -> rotConsumer.accept(hasTargets() ? alignRotationOutput() : 0))
            .until(this::isAligned)
            .withName("AlignToTarget");
    }
}
`; }

// ─── PathPlanner ─────────────────────────────────────────────────────────────

function tplPathPlanner(): string { return `package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * Drop this into RobotContainer and call new AutoConfig(m_drive) in the constructor.
 * Then call getSelectedAuto() from getAutonomousCommand().
 */
public class AutoConfig {

    private final SendableChooser<Command> m_chooser;

    public AutoConfig(SwerveDriveSubsystem drive) {
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            DriverStation.reportError("PathPlanner config failed: " + e.getMessage(), false);
            m_chooser = new SendableChooser<>();
            SmartDashboard.putData("Auto", m_chooser);
            return;
        }

        AutoBuilder.configure(
            drive::getPose,
            drive::resetOdometry,
            drive::getChassisSpeeds,
            (speeds, feedforwards) -> drive.driveRobotRelative(speeds),
            new PPHolonomicDriveController(
                new PIDConstants(5.0, 0.0, 0.0),  // translation
                new PIDConstants(5.0, 0.0, 0.0)   // rotation
            ),
            config,
            () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                      == DriverStation.Alliance.Red,
            drive
        );

        // Register named commands that PathPlanner paths can trigger by name.
        // Example: NamedCommands.registerCommand("Shoot", new ShootCommand(m_shooter));
        registerNamedCommands();

        m_chooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto", m_chooser);
    }

    private void registerNamedCommands() {
        // TODO: register your named commands here.
        // NamedCommands.registerCommand("Example", Commands.print("Running example"));
    }

    public Command getSelectedAuto() {
        return m_chooser.getSelected();
    }
}
`; }

// ─── command handlers ────────────────────────────────────────────────────────

async function cmdSwerve() {
    const robotDir = await requireRobotDir();
    if (!robotDir) { return; }

    const motor = await qp(['REV (MAXSwerve / SparkMax)', 'CTRE (TalonFX + CANcoder)'],
        'Swerve motor library');
    if (!motor) { return; }

    const gyro  = await qp(['NavX (Studica)', 'Pigeon 2 (CTRE)'], 'Gyro');
    if (!gyro) { return; }

    const isRev    = motor.startsWith('REV');
    const isPigeon = gyro.startsWith('Pigeon');
    const subDir   = path.join(robotDir, 'subsystems');

    const modulePath = path.join(subDir, 'SwerveModule.java');
    const drivePath  = path.join(subDir, 'SwerveDriveSubsystem.java');

    const modWritten   = await writeSafe(modulePath, isRev ? tplSwerveModuleRev() : tplSwerveModuleCtre());
    const driveWritten = await writeSafe(drivePath,
        isPigeon ? tplSwerveDrivePigeon(isRev) : tplSwerveDriveNavx(isRev));

    if (driveWritten) { await reveal(drivePath); }
    else if (modWritten) { await reveal(modulePath); }

    vscode.window.showInformationMessage(
        `FRC Forge: Swerve written (${isRev ? 'REV' : 'CTRE'} modules, ${isPigeon ? 'Pigeon 2' : 'NavX'} gyro). Fill in DriveConstants + ModuleConstants to finish.`);
}

async function cmdTank() {
    const robotDir = await requireRobotDir();
    if (!robotDir) { return; }

    const motor = await qp(['REV (SparkMax)', 'CTRE (TalonFX)'], 'Tank drive motor library');
    if (!motor) { return; }

    const isRev  = motor.startsWith('REV');
    const p      = path.join(robotDir, 'subsystems', 'TankDriveSubsystem.java');
    const ok     = await writeSafe(p, isRev ? tplTankRev() : tplTankCtre());
    if (ok) { await reveal(p); }
    vscode.window.showInformationMessage('FRC Forge: TankDriveSubsystem.java written. Fill in DriveConstants to finish.');
}

async function cmdMechanism() {
    const robotDir = await requireRobotDir();
    if (!robotDir) { return; }

    const name = await vscode.window.showInputBox({
        prompt: 'Subsystem name (e.g. Arm, Elevator, Intake)',
        validateInput: v => /^[A-Z][A-Za-z0-9]*$/.test(v) ? null : 'Must start with a capital letter, letters/digits only'
    });
    if (!name) { return; }

    const presetInput = await vscode.window.showInputBox({
        prompt: 'Preset positions (comma-separated, e.g. stow,intake,score)',
        value: 'stow,intake,score'
    });
    if (presetInput === undefined) { return; }

    const presets = presetInput.split(',').map(s => s.trim()).filter(Boolean);
    const p       = path.join(robotDir, 'subsystems', `${name}Subsystem.java`);
    const ok      = await writeSafe(p, tplMechanism(name, presets));
    if (ok) { await reveal(p); }
    vscode.window.showInformationMessage(`FRC Forge: ${name}Subsystem.java written with presets: ${presets.join(', ')}.`);
}

async function cmdLeds() {
    const robotDir = await requireRobotDir();
    if (!robotDir) { return; }

    const p  = path.join(robotDir, 'subsystems', 'LEDSubsystem.java');
    const ok = await writeSafe(p, tplLeds());
    if (ok) { await reveal(p); }
    vscode.window.showInformationMessage('FRC Forge: LEDSubsystem.java written. Set kPort and kLength.');
}

async function cmdVision() {
    const robotDir = await requireRobotDir();
    if (!robotDir) { return; }

    const lib = await qp(['PhotonVision (PhotonPoseEstimator)', 'Limelight (NetworkTables 4)'], 'Vision library');
    if (!lib) { return; }

    const isPhoton = lib.startsWith('Photo');
    const p        = path.join(robotDir, 'subsystems', 'VisionSubsystem.java');
    const ok       = await writeSafe(p, isPhoton ? tplVisionPhoton() : tplVisionLimelight());
    if (ok) { await reveal(p); }
    vscode.window.showInformationMessage(`FRC Forge: VisionSubsystem.java written (${isPhoton ? 'PhotonVision' : 'Limelight'}).`);
}

async function cmdPathPlanner() {
    const robotDir = await requireRobotDir();
    if (!robotDir) { return; }

    const p  = path.join(robotDir, 'AutoConfig.java');
    const ok = await writeSafe(p, tplPathPlanner());
    if (ok) { await reveal(p); }
    vscode.window.showInformationMessage('FRC Forge: AutoConfig.java written. Instantiate it in RobotContainer and register your NamedCommands.');
}

async function cmdOpen() {
    const choice = await qp([
        '$(circuit-board)  Swerve Drive',
        '$(arrow-both)     Tank Drive',
        '$(gear)           Generic Mechanism',
        '$(lightbulb)      Addressable LEDs',
        '$(eye)            Vision',
        '$(map)            PathPlanner Autos',
    ], 'FRC Forge — choose a template');
    if (!choice) { return; }

    if (choice.includes('Swerve'))     { await cmdSwerve(); }
    else if (choice.includes('Tank'))  { await cmdTank(); }
    else if (choice.includes('Mech'))  { await cmdMechanism(); }
    else if (choice.includes('LED'))   { await cmdLeds(); }
    else if (choice.includes('Vis'))   { await cmdVision(); }
    else if (choice.includes('Path'))  { await cmdPathPlanner(); }
}

// ─── activate / deactivate ───────────────────────────────────────────────────

export function activate(context: vscode.ExtensionContext) {
    context.subscriptions.push(
        vscode.commands.registerCommand('frcForge.open',           cmdOpen),
        vscode.commands.registerCommand('frcForge.newSwerve',      cmdSwerve),
        vscode.commands.registerCommand('frcForge.newTank',        cmdTank),
        vscode.commands.registerCommand('frcForge.newMechanism',   cmdMechanism),
        vscode.commands.registerCommand('frcForge.newLeds',        cmdLeds),
        vscode.commands.registerCommand('frcForge.newVision',      cmdVision),
        vscode.commands.registerCommand('frcForge.newPathPlanner', cmdPathPlanner),
    );
}

export function deactivate() {}
