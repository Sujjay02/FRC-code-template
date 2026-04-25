// ─── PathPlanner ─────────────────────────────────────────────────────────────

export function tplPathPlanner(): string { return `package frc.robot;

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
 * Drop into RobotContainer: new AutoConfig(m_drive) in the constructor,
 * then return autoConfig.getSelectedAuto() from getAutonomousCommand().
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
                new PIDConstants(5.0, 0.0, 0.0), // translation
                new PIDConstants(5.0, 0.0, 0.0)  // rotation
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
        // TODO: register commands that PathPlanner can trigger mid-path.
        // Example: NamedCommands.registerCommand("Shoot", shooter.shootCommand());
    }

    public Command getSelectedAuto() { return m_chooser.getSelected(); }
}
`; }

// ─── Choreo ───────────────────────────────────────────────────────────────────

export function tplChoreo(): string { return `package frc.robot;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoLoop;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * Drop into RobotContainer: new ChoreoAutoConfig(m_drive) in the constructor.
 * Trajectories are .traj files under src/main/deploy/choreo/.
 *
 * Add dependency in build.gradle:
 *   implementation 'com.choreo:choreolib:VERSION'
 * Replace VERSION with the latest from https://github.com/SleipnirGroup/ChoreoLib/releases
 */
public class ChoreoAutoConfig {

    private final AutoFactory m_factory;
    private final SendableChooser<Command> m_chooser = new SendableChooser<>();

    public ChoreoAutoConfig(SwerveDriveSubsystem drive) {
        // Feedback controllers that correct trajectory tracking error
        PIDController xController = new PIDController(5.0, 0, 0); // TODO: tune
        PIDController yController = new PIDController(5.0, 0, 0); // TODO: tune
        PIDController rController = new PIDController(5.0, 0, 0); // TODO: tune
        rController.enableContinuousInput(-Math.PI, Math.PI);

        m_factory = Choreo.createAutoFactory(
            drive,
            drive::getPose,
            (pose, sample) -> {
                SwerveSample s = (SwerveSample) sample;
                // Feed-forward from trajectory + feedback from pose error
                ChassisSpeeds ff = s.getChassisSpeeds();
                ChassisSpeeds fb = new ChassisSpeeds(
                    xController.calculate(pose.getX(),                    s.x),
                    yController.calculate(pose.getY(),                    s.y),
                    rController.calculate(pose.getRotation().getRadians(), s.heading)
                );
                drive.driveRobotRelative(
                    new ChassisSpeeds(ff.vxMetersPerSecond + fb.vxMetersPerSecond,
                                      ff.vyMetersPerSecond + fb.vyMetersPerSecond,
                                      ff.omegaRadiansPerSecond + fb.omegaRadiansPerSecond));
            },
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
            new AutoFactory.AutoBindings()
        );

        m_chooser.setDefaultOption("None", Commands.none());
        // TODO: add your autos below
        // m_chooser.addOption("Example Auto", exampleAuto());

        SmartDashboard.putData("Auto", m_chooser);
    }

    /**
     * Example: reset odometry to path start, then follow a single trajectory.
     * Name must match the .traj filename (without extension) in deploy/choreo/.
     */
    private Command exampleAuto() {
        AutoLoop loop = m_factory.newLoop("ExampleAuto");
        AutoTrajectory traj = m_factory.trajectory("ExamplePath", loop);
        loop.enabled().onTrue(traj.resetOdometry().andThen(traj.cmd()));
        return loop.cmd();
    }

    /**
     * Multi-segment auto: chain trajectories together with actions in between.
     */
    private Command multiSegmentAuto() {
        AutoLoop loop = m_factory.newLoop("MultiSegment");
        AutoTrajectory seg1 = m_factory.trajectory("Segment1", loop);
        AutoTrajectory seg2 = m_factory.trajectory("Segment2", loop);

        loop.enabled().onTrue(seg1.resetOdometry().andThen(seg1.cmd()));
        // When seg1 ends, run an action then start seg2
        seg1.done().onTrue(
            Commands.print("Between segments") // TODO: replace with real action
                .andThen(seg2.cmd())
        );
        return loop.cmd();
    }

    public Command getSelectedAuto() { return m_chooser.getSelected(); }
}
`; }
