import * as vscode from 'vscode';
import * as fs from 'fs';
import * as path from 'path';

// ─── helpers ─────────────────────────────────────────────────────────────────

function findRobotDir(wsRoot: string): string | undefined {
    const hasBuild = fs.existsSync(path.join(wsRoot, 'build.gradle'));
    const hasPrefs = fs.existsSync(path.join(wsRoot, '.wpilib', 'wpilib_preferences.json'));
    const hasSrc   = fs.existsSync(path.join(wsRoot, 'src', 'main', 'java', 'frc', 'robot'));
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

// ─── swerve module templates ──────────────────────────────────────────────────

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

    private Rotation2d getGyroYaw() { return Rotation2d.fromDegrees(-m_gyro.getAngle()); }

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

// ─── shooter ─────────────────────────────────────────────────────────────────

function tplShooter(): string { return `package frc.robot.subsystems;

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
        SmartDashboard.putNumber("Shooter/RPM",      m_encoder.getVelocity());
        SmartDashboard.putNumber("Shooter/TargetRPM", m_targetRpm);
        SmartDashboard.putBoolean("Shooter/AtSpeed", atTargetSpeed());
    }

    public void spinUp()           { setRpm(kTargetRpm); }
    public void stop()             { m_targetRpm = 0; m_top.stopMotor(); }

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

// ─── hopper ──────────────────────────────────────────────────────────────────

function tplHopper(): string { return `package frc.robot.subsystems;

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

// ─── intake ──────────────────────────────────────────────────────────────────

function tplIntake(): string { return `package frc.robot.subsystems;

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

// ─── elevator ────────────────────────────────────────────────────────────────

function tplElevator(): string { return `package frc.robot.subsystems;

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

    private static final int    kLeaderCanId   = 4;   // TODO: set CAN ID
    private static final int    kFollowerCanId  = 5;  // TODO: set CAN ID
    private static final double kP              = 0.1;
    private static final double kI              = 0.0;
    private static final double kD              = 0.0;
    private static final double kFF             = 0.0;
    private static final double kMaxOutput      = 0.6;
    private static final double kTolerance      = 0.5; // rotations

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
        SparkMaxConfig leaderCfg = new SparkMaxConfig();
        leaderCfg.idleMode(IdleMode.kBrake).smartCurrentLimit(50);
        leaderCfg.closedLoop.pid(kP, kI, kD).velocityFF(kFF).outputRange(-kMaxOutput, kMaxOutput);
        m_leader.configure(leaderCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig followerCfg = new SparkMaxConfig();
        followerCfg.follow(m_leader, true);
        m_follower.configure(followerCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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

    public boolean atTarget() {
        return Math.abs(m_encoder.getPosition() - m_setpoint) < kTolerance;
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

// ─── climb ───────────────────────────────────────────────────────────────────

function tplClimb(): string { return `package frc.robot.subsystems;

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
        SparkMaxConfig leftCfg = new SparkMaxConfig();
        leftCfg.idleMode(IdleMode.kBrake).smartCurrentLimit(60);
        m_left.configure(leftCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig rightCfg = new SparkMaxConfig();
        rightCfg.idleMode(IdleMode.kBrake).smartCurrentLimit(60).follow(m_left, true);
        m_right.configure(rightCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climb/Position", m_encoder.getPosition());
    }

    public void extend()  { m_left.set(kSpeed);  }
    public void retract() { m_left.set(-kSpeed); }
    public void stop()    { m_left.stopMotor(); }

    public boolean isExtended()  { return Math.abs(m_encoder.getPosition() - kExtendPos) < kTolerance; }
    public boolean isRetracted() { return m_encoder.getPosition() < kTolerance; }

    public Command extendCommand()  { return startEnd(this::extend,  this::stop).until(this::isExtended); }
    public Command retractCommand() { return startEnd(this::retract, this::stop).until(this::isRetracted); }
}
`; }

// ─── generic mechanism ────────────────────────────────────────────────────────

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

    private static final int    kMotorCanId = 0;   // TODO: set CAN ID
    private static final double kP          = 0.1;
    private static final double kI          = 0.0;
    private static final double kD          = 0.0;
    private static final double kFF         = 0.0;
    private static final double kTolerance  = 0.5; // rotations

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

${presetMethods}

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

    private static final int kPort   = 9;  // TODO: set PWM port
    private static final int kLength = 60; // TODO: set strip length

    private final AddressableLED       m_led    = new AddressableLED(kPort);
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

    public Command solidCommand(Color color)     { return runOnce(() -> setSolid(color)); }
    public Command rainbowCommand()              { return run(this::setRainbow); }
    public Command breatheCommand(Color color)   { return run(() -> setBreathe(color)); }
    public Command offCommand()                  { return runOnce(this::setOff); }
    public Command allianceCommand(boolean isRed){ return runOnce(() -> setAllianceColor(isRed)); }
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

    private static final double kYawP         = 0.04;
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

    public double alignRotationOutput() {
        m_yawPID.setSetpoint(0);
        m_yawPID.setTolerance(kYawTolerance);
        return -m_yawPID.calculate(getYawToTarget());
    }

    public boolean isAligned() { return m_yawPID.atSetpoint() && hasTargets(); }

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

    private final NetworkTable  m_table  =
        NetworkTableInstance.getDefault().getTable(kTableName);
    private final PIDController m_yawPID = new PIDController(kYawP, 0, 0);

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Vision/HasTargets", hasTargets());
        if (hasTargets()) {
            SmartDashboard.putNumber("Vision/TX",    getTx());
            SmartDashboard.putNumber("Vision/TY",    getTy());
            SmartDashboard.putNumber("Vision/TagId", getTagId());
        }
    }

    private double entry(String key) {
        return m_table.getEntry(key).getDouble(0.0);
    }

    public boolean hasTargets() { return entry("tv") == 1.0; }
    public double  getTx()      { return entry("tx"); }
    public double  getTy()      { return entry("ty"); }
    public double  getTa()      { return entry("ta"); }
    public int     getTagId()   { return (int) entry("tid"); }

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
 * Drop into RobotContainer and call new AutoConfig(m_drive) in the constructor.
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
                new PIDConstants(5.0, 0.0, 0.0),
                new PIDConstants(5.0, 0.0, 0.0)
            ),
            config,
            () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                      == DriverStation.Alliance.Red,
            drive
        );

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

    const motor = await qp(['REV (MAXSwerve / SparkMax)', 'CTRE (TalonFX + CANcoder)'], 'Swerve motor library');
    if (!motor) { return; }

    const gyro = await qp(['NavX (Studica)', 'Pigeon 2 (CTRE)'], 'Gyro');
    if (!gyro) { return; }

    const isRev    = motor.startsWith('REV');
    const isPigeon = gyro.startsWith('Pigeon');
    const subDir   = path.join(robotDir, 'subsystems');

    const modulePath = path.join(subDir, 'SwerveModule.java');
    const drivePath  = path.join(subDir, 'SwerveDriveSubsystem.java');

    const modOk   = await writeSafe(modulePath, isRev ? tplSwerveModuleRev() : tplSwerveModuleCtre());
    const driveOk = await writeSafe(drivePath, isPigeon ? tplSwerveDrivePigeon(isRev) : tplSwerveDriveNavx(isRev));

    if (driveOk) { await reveal(drivePath); }
    else if (modOk) { await reveal(modulePath); }

    vscode.window.showInformationMessage(
        `FRC Forge: Swerve written (${isRev ? 'REV' : 'CTRE'} modules, ${isPigeon ? 'Pigeon 2' : 'NavX'}). Fill in DriveConstants + ModuleConstants.`);
}

async function cmdTank() {
    const robotDir = await requireRobotDir();
    if (!robotDir) { return; }

    const motor = await qp(['REV (SparkMax)', 'CTRE (TalonFX)'], 'Tank drive motor library');
    if (!motor) { return; }

    const isRev = motor.startsWith('REV');
    const p     = path.join(robotDir, 'subsystems', 'TankDriveSubsystem.java');
    const ok    = await writeSafe(p, isRev ? tplTankRev() : tplTankCtre());
    if (ok) { await reveal(p); }
    vscode.window.showInformationMessage('FRC Forge: TankDriveSubsystem.java written. Fill in DriveConstants.');
}

async function cmdShooter() {
    const robotDir = await requireRobotDir();
    if (!robotDir) { return; }
    const p  = path.join(robotDir, 'subsystems', 'ShooterSubsystem.java');
    const ok = await writeSafe(p, tplShooter());
    if (ok) { await reveal(p); }
    vscode.window.showInformationMessage('FRC Forge: ShooterSubsystem.java written. Set CAN IDs and tune PID.');
}

async function cmdShooterHopper() {
    const robotDir = await requireRobotDir();
    if (!robotDir) { return; }
    const sp = path.join(robotDir, 'subsystems', 'ShooterSubsystem.java');
    const hp = path.join(robotDir, 'subsystems', 'HopperSubsystem.java');
    const s  = await writeSafe(sp, tplShooter());
    const h  = await writeSafe(hp, tplHopper());
    if (h)      { await reveal(hp); }
    else if (s) { await reveal(sp); }
    vscode.window.showInformationMessage('FRC Forge: ShooterSubsystem.java + HopperSubsystem.java written. Set CAN IDs and DIO ports.');
}

async function cmdIntake() {
    const robotDir = await requireRobotDir();
    if (!robotDir) { return; }
    const p  = path.join(robotDir, 'subsystems', 'IntakeSubsystem.java');
    const ok = await writeSafe(p, tplIntake());
    if (ok) { await reveal(p); }
    vscode.window.showInformationMessage('FRC Forge: IntakeSubsystem.java written. Set CAN ID and DIO port.');
}

async function cmdElevator() {
    const robotDir = await requireRobotDir();
    if (!robotDir) { return; }
    const p  = path.join(robotDir, 'subsystems', 'ElevatorSubsystem.java');
    const ok = await writeSafe(p, tplElevator());
    if (ok) { await reveal(p); }
    vscode.window.showInformationMessage('FRC Forge: ElevatorSubsystem.java written. Set CAN IDs and tune heights.');
}

async function cmdClimb() {
    const robotDir = await requireRobotDir();
    if (!robotDir) { return; }
    const p  = path.join(robotDir, 'subsystems', 'ClimbSubsystem.java');
    const ok = await writeSafe(p, tplClimb());
    if (ok) { await reveal(p); }
    vscode.window.showInformationMessage('FRC Forge: ClimbSubsystem.java written. Set CAN IDs and tune speed.');
}

async function cmdMechanism() {
    const robotDir = await requireRobotDir();
    if (!robotDir) { return; }

    const name = await vscode.window.showInputBox({
        prompt: 'Subsystem name (e.g. Arm, Wrist, Indexer)',
        validateInput: v => /^[A-Z][A-Za-z0-9]*$/.test(v) ? null : 'Must start with a capital letter, letters/digits only'
    });
    if (!name) { return; }

    const presetInput = await vscode.window.showInputBox({
        prompt: 'Preset positions (comma-separated)',
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
    vscode.window.showInformationMessage('FRC Forge: AutoConfig.java written. Instantiate in RobotContainer and register NamedCommands.');
}

async function cmdCombine() {
    const robotDir = await requireRobotDir();
    if (!robotDir) { return; }

    interface CombineItem extends vscode.QuickPickItem { id: string; }
    const items: CombineItem[] = [
        { label: 'Swerve Drive',      id: 'swerve',        description: 'SwerveDriveSubsystem + SwerveModule' },
        { label: 'Tank Drive',        id: 'tank',           description: 'TankDriveSubsystem' },
        { label: 'Shooter',           id: 'shooter',        description: 'ShooterSubsystem (dual flywheel, velocity PID)' },
        { label: 'Shooter + Hopper',  id: 'shooterhopper',  description: 'ShooterSubsystem + HopperSubsystem' },
        { label: 'Intake',            id: 'intake',         description: 'IntakeSubsystem (roller + beam break)' },
        { label: 'Elevator',          id: 'elevator',       description: 'ElevatorSubsystem (L1–L4 presets)' },
        { label: 'Climb',             id: 'climb',          description: 'ClimbSubsystem (extend/retract)' },
        { label: 'Generic Mechanism', id: 'mechanism',      description: 'Custom name + preset positions' },
        { label: 'Vision',            id: 'vision',         description: 'VisionSubsystem (PhotonVision or Limelight)' },
        { label: 'Addressable LEDs',  id: 'leds',           description: 'LEDSubsystem' },
        { label: 'PathPlanner Autos', id: 'pathplanner',    description: 'AutoConfig.java' },
    ];

    const selected = await vscode.window.showQuickPick(items, {
        canPickMany: true,
        placeHolder: 'Select templates to generate — use Space to check, Enter to confirm',
        title: 'FRC Forge — Combine Templates',
    }) as CombineItem[] | undefined;
    if (!selected?.length) { return; }

    for (const item of selected) {
        switch (item.id) {
            case 'swerve':        await cmdSwerve();        break;
            case 'tank':          await cmdTank();          break;
            case 'shooter':       await cmdShooter();       break;
            case 'shooterhopper': await cmdShooterHopper(); break;
            case 'intake':        await cmdIntake();        break;
            case 'elevator':      await cmdElevator();      break;
            case 'climb':         await cmdClimb();         break;
            case 'mechanism':     await cmdMechanism();     break;
            case 'vision':        await cmdVision();        break;
            case 'leds':          await cmdLeds();          break;
            case 'pathplanner':   await cmdPathPlanner();   break;
        }
    }
}

async function cmdOpen() {
    const choice = await qp([
        'Swerve Drive',
        'Tank Drive',
        'Shooter',
        'Shooter + Hopper',
        'Intake',
        'Elevator',
        'Climb',
        'Generic Mechanism',
        'Vision',
        'Addressable LEDs',
        'PathPlanner Autos',
        'Combine Templates',
    ], 'FRC Forge — choose a template');
    if (!choice) { return; }

    if      (choice === 'Swerve Drive')       { await cmdSwerve(); }
    else if (choice === 'Tank Drive')         { await cmdTank(); }
    else if (choice === 'Shooter')            { await cmdShooter(); }
    else if (choice === 'Shooter + Hopper')   { await cmdShooterHopper(); }
    else if (choice === 'Intake')             { await cmdIntake(); }
    else if (choice === 'Elevator')           { await cmdElevator(); }
    else if (choice === 'Climb')              { await cmdClimb(); }
    else if (choice === 'Generic Mechanism')  { await cmdMechanism(); }
    else if (choice === 'Vision')             { await cmdVision(); }
    else if (choice === 'Addressable LEDs')   { await cmdLeds(); }
    else if (choice === 'PathPlanner Autos')  { await cmdPathPlanner(); }
    else if (choice === 'Combine Templates')  { await cmdCombine(); }
}

// ─── side panel ──────────────────────────────────────────────────────────────

class FrcForgeViewProvider implements vscode.WebviewViewProvider {
    static readonly viewId = 'frcForge.panel';

    resolveWebviewView(view: vscode.WebviewView) {
        view.webview.options = { enableScripts: true };
        view.webview.html = this._html();

        view.webview.onDidReceiveMessage(msg => {
            switch (msg.command) {
                case 'swerve':        cmdSwerve();        break;
                case 'tank':          cmdTank();          break;
                case 'shooter':       cmdShooter();       break;
                case 'shooterhopper': cmdShooterHopper(); break;
                case 'intake':        cmdIntake();        break;
                case 'elevator':      cmdElevator();      break;
                case 'climb':         cmdClimb();         break;
                case 'mechanism':     cmdMechanism();     break;
                case 'leds':          cmdLeds();          break;
                case 'vision':        cmdVision();        break;
                case 'pathplanner':   cmdPathPlanner();   break;
                case 'combine':       cmdCombine();       break;
            }
        });
    }

    private _html(): string {
        const nonce = Math.random().toString(36).substring(2, 15);
        const row = (id: string, label: string) =>
            `<button class="row" data-cmd="${id}">${label}</button>`;

        return `<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta http-equiv="Content-Security-Policy" content="default-src 'none'; style-src 'unsafe-inline'; script-src 'nonce-${nonce}';">
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }

  body {
    font-family: var(--vscode-font-family);
    font-size: var(--vscode-font-size, 13px);
    color: var(--vscode-foreground);
    background: var(--vscode-sideBar-background);
    overflow-x: hidden;
  }

  .pane {
    border-top: 1px solid var(--vscode-sideBarSectionHeader-border, rgba(128,128,128,0.35));
  }

  .pane-header {
    display: flex;
    align-items: center;
    height: 22px;
    padding: 0 12px;
    font-size: 11px;
    font-weight: 700;
    letter-spacing: 0.08em;
    text-transform: uppercase;
    color: var(--vscode-sideBarSectionHeader-foreground, var(--vscode-foreground));
    background: var(--vscode-sideBarSectionHeader-background, transparent);
    user-select: none;
    overflow: hidden;
  }

  .row {
    display: block;
    width: 100%;
    padding: 0 0 0 28px;
    height: 22px;
    line-height: 22px;
    background: transparent;
    color: var(--vscode-foreground);
    border: none;
    cursor: pointer;
    font-size: var(--vscode-font-size, 13px);
    font-family: var(--vscode-font-family);
    text-align: left;
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
  }

  .row:hover {
    background: var(--vscode-list-hoverBackground);
    color: var(--vscode-list-hoverForeground, var(--vscode-foreground));
    outline: none;
  }

  .row:focus-visible {
    outline: 1px solid var(--vscode-focusBorder);
    outline-offset: -1px;
  }

  .combine-wrap {
    padding: 8px 12px 10px;
    border-top: 1px solid var(--vscode-sideBarSectionHeader-border, rgba(128,128,128,0.35));
  }

  .combine-btn {
    width: 100%;
    padding: 4px 0;
    background: var(--vscode-button-background, #0e639c);
    color: var(--vscode-button-foreground, #fff);
    border: none;
    border-radius: 2px;
    cursor: pointer;
    font-size: var(--vscode-font-size, 13px);
    font-family: var(--vscode-font-family);
  }

  .combine-btn:hover {
    background: var(--vscode-button-hoverBackground, #1177bb);
  }

  .combine-btn:focus-visible {
    outline: 1px solid var(--vscode-focusBorder);
    outline-offset: 2px;
  }
</style>
</head>
<body>

<div class="pane">
  <div class="pane-header">Drivetrains</div>
  ${row('swerve', 'Swerve Drive')}
  ${row('tank',   'Tank Drive')}
</div>

<div class="pane">
  <div class="pane-header">Mechanisms</div>
  ${row('shooter',       'Shooter')}
  ${row('shooterhopper', 'Shooter + Hopper')}
  ${row('intake',        'Intake')}
  ${row('elevator',      'Elevator')}
  ${row('climb',         'Climb')}
  ${row('mechanism',     'Generic Mechanism')}
</div>

<div class="pane">
  <div class="pane-header">Sensors &amp; Visualization</div>
  ${row('vision', 'Vision')}
  ${row('leds',   'Addressable LEDs')}
</div>

<div class="pane">
  <div class="pane-header">Autonomous</div>
  ${row('pathplanner', 'PathPlanner Autos')}
</div>

<div class="combine-wrap">
  <button class="combine-btn" data-cmd="combine">Combine Templates</button>
</div>

<script nonce="${nonce}">
  const vscode = acquireVsCodeApi();
  document.querySelectorAll('[data-cmd]').forEach(el => {
    el.addEventListener('click', () => vscode.postMessage({ command: el.dataset.cmd }));
  });
</script>
</body>
</html>`;
    }
}

// ─── azure auth ──────────────────────────────────────────────────────────────

export async function getAzureToken(scopes: string[] = ['https://management.azure.com/.default']): Promise<string | undefined> {
    try {
        const session = await vscode.authentication.getSession('microsoft', scopes, { createIfNone: true });
        return session.accessToken;
    } catch {
        vscode.window.showErrorMessage('FRC Forge: Azure sign-in failed or was cancelled.');
        return undefined;
    }
}

async function cmdSignInAzure() {
    const token = await getAzureToken();
    if (token) {
        vscode.window.showInformationMessage('FRC Forge: Signed in to Azure successfully.');
    }
}

// ─── activate / deactivate ───────────────────────────────────────────────────

export function activate(context: vscode.ExtensionContext) {
    context.subscriptions.push(
        vscode.commands.registerCommand('frcForge.open',              cmdOpen),
        vscode.commands.registerCommand('frcForge.newSwerve',         cmdSwerve),
        vscode.commands.registerCommand('frcForge.newTank',           cmdTank),
        vscode.commands.registerCommand('frcForge.newShooter',        cmdShooter),
        vscode.commands.registerCommand('frcForge.newShooterHopper',  cmdShooterHopper),
        vscode.commands.registerCommand('frcForge.newIntake',         cmdIntake),
        vscode.commands.registerCommand('frcForge.newElevator',       cmdElevator),
        vscode.commands.registerCommand('frcForge.newClimb',          cmdClimb),
        vscode.commands.registerCommand('frcForge.newMechanism',      cmdMechanism),
        vscode.commands.registerCommand('frcForge.newLeds',           cmdLeds),
        vscode.commands.registerCommand('frcForge.newVision',         cmdVision),
        vscode.commands.registerCommand('frcForge.newPathPlanner',    cmdPathPlanner),
        vscode.commands.registerCommand('frcForge.combine',           cmdCombine),
        vscode.commands.registerCommand('frcForge.signInAzure',       cmdSignInAzure),
        vscode.window.registerWebviewViewProvider(FrcForgeViewProvider.viewId, new FrcForgeViewProvider()),
    );
}

export function deactivate() {}
