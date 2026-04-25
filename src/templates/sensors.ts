// ─── Vision (PhotonVision) ───────────────────────────────────────────────────

export function tplVisionPhoton(): string { return `package frc.robot.subsystems;

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

    public boolean hasTargets() { return m_camera.getLatestResult().hasTargets(); }

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

    public Command alignToTargetCommand(java.util.function.DoubleConsumer rotOutput) {
        return run(() -> rotOutput.accept(hasTargets() ? alignRotationOutput() : 0))
            .until(this::isAligned)
            .withName("AlignToTarget");
    }
}
`; }

// ─── Vision (Limelight, NetworkTables 4) ─────────────────────────────────────

export function tplVisionLimelight(): string { return `package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    private static final String kTableName    = "limelight"; // TODO: match Limelight hostname
    private static final double kYawP         = 0.04;
    private static final double kYawTolerance = 1.0;         // degrees

    private final NetworkTable  m_table  = NetworkTableInstance.getDefault().getTable(kTableName);
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

    private double entry(String key) { return m_table.getEntry(key).getDouble(0.0); }

    public boolean hasTargets() { return entry("tv") == 1.0; }
    public double  getTx()      { return entry("tx"); }
    public double  getTy()      { return entry("ty"); }
    public double  getTa()      { return entry("ta"); }
    public int     getTagId()   { return (int) entry("tid"); }

    public void setPipelineIndex(int idx) { m_table.getEntry("pipeline").setNumber(idx); }

    public Pose2d getBotPose() {
        double[] raw = m_table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        if (raw.length < 6) { return new Pose2d(); }
        return new Pose2d(raw[0], raw[1], Rotation2d.fromDegrees(raw[5]));
    }

    public double getLatencySeconds() { return (entry("tl") + entry("cl")) / 1000.0; }

    public double alignRotationOutput() {
        m_yawPID.setSetpoint(0);
        m_yawPID.setTolerance(kYawTolerance);
        return -m_yawPID.calculate(getTx());
    }

    public boolean isAligned() { return m_yawPID.atSetpoint() && hasTargets(); }

    public Command alignToTargetCommand(java.util.function.DoubleConsumer rotOutput) {
        return run(() -> rotOutput.accept(hasTargets() ? alignRotationOutput() : 0))
            .until(this::isAligned)
            .withName("AlignToTarget");
    }
}
`; }

// ─── Addressable LEDs ────────────────────────────────────────────────────────

export function tplLeds(): string { return `package frc.robot.subsystems;

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
    public void periodic() { m_led.setData(m_buffer); }

    public void setSolid(Color color)                 { LEDPattern.solid(color).applyTo(m_buffer); }
    public void setRainbow()                          { LEDPattern.rainbow(255, 128).applyTo(m_buffer); }
    public void setBreathe(Color color)               { LEDPattern.solid(color).breathe(Units.Seconds.of(2)).applyTo(m_buffer); }
    public void setStrobe(Color color, double period) { LEDPattern.solid(color).blink(Units.Seconds.of(period)).applyTo(m_buffer); }
    public void setChase(Color color)                 { LEDPattern.solid(color).scrollAtRelativeSpeed(Units.Percent.of(25)).applyTo(m_buffer); }
    public void setOff()                              { LEDPattern.solid(Color.kBlack).applyTo(m_buffer); }

    public void setProgressBar(Color color, double fraction) {
        LEDPattern.solid(color).mask(LEDPattern.progressMaskLayer(() -> fraction)).applyTo(m_buffer);
    }

    public void setAllianceColor(boolean isRed) { setSolid(isRed ? Color.kRed : Color.kBlue); }
    public void setIndicator(boolean ok)        { setSolid(ok ? Color.kGreen : Color.kRed); }

    public Command solidCommand(Color color)        { return runOnce(() -> setSolid(color)); }
    public Command rainbowCommand()                 { return run(this::setRainbow); }
    public Command breatheCommand(Color color)      { return run(() -> setBreathe(color)); }
    public Command offCommand()                     { return runOnce(this::setOff); }
    public Command allianceCommand(boolean isRed)   { return runOnce(() -> setAllianceColor(isRed)); }
}
`; }
