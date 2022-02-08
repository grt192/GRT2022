package frc.robot.commands.tank;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

import frc.robot.subsystems.TankSubsystem;

/**
 * A command to drive the robot from some start to end point, passing through given waypoints in the path. 
 * The trajectory can be given outright, or can be generated from a start and end position given as a `Pose2d` 
 * and waypoints given as a List of `Translation2d`. All units are in meters.
 * 
 * https://docs.wpilib.org/en/stable/docs/software/examples-tutorials/trajectory-tutorial/trajectory-tutorial-overview.html
 * 
 * TODO: remeasure constants for new robot
 */
public class FollowPathCommand extends RamseteCommand {
    // Robot constants
    private static final double TRACK_WIDTH = 0.7805022504396351;
    private static final DifferentialDriveKinematics KINEMATICS = 
        new DifferentialDriveKinematics(TRACK_WIDTH);

    // Drive constants
    // https://docs.wpilib.org/en/stable/docs/software/examples-tutorials/trajectory-tutorial/characterizing-drive.html
    private static final double Ks = 0.15964; // V
    private static final double Kv = 2.3796; // Vs/m
    private static final double Ka = 0.31372; // Vs^2/m

    // Velocity PID constants
    private static final double Kp = 0.018101;
    private static final double Ki = 0;
    private static final double Kd = 0;

    // Velocity / Acceleration constants
    private static final double MAX_VEL = 3; // m/s
    private static final double MAX_ACCEL = 3; // m/s^2

    // Ramsete constants
    private static final double RAMSETE_B = 2;
    private static final double RAMSETE_ZETA = 0.7;

    /**
     * Creates a FollowPathCommand from a given trajectory.
     * 
     * @param tankSubsystem The tank subsystem.
     * @param trajectory The trajectory to follow.
     */
    public FollowPathCommand(TankSubsystem tankSubsystem, Trajectory trajectory) {
        super(
            trajectory,
            tankSubsystem::getRobotPosition, // Position supplier
            new RamseteController(RAMSETE_B, RAMSETE_ZETA),
            new SimpleMotorFeedforward(Ks, Kv, Ka),
            KINEMATICS,
            tankSubsystem::getWheelSpeeds, // Wheel speed supplier
            // PID controllers
            new PIDController(Kp, Ki, Kd),
            new PIDController(Kp, Ki, Kd),
            tankSubsystem::setTankDriveVoltages, // Wheel voltage consumer
            tankSubsystem
        );

        tankSubsystem.resetPosition(trajectory.getInitialPose());
    }

    /**
     * Creates a FollowPathCommand from a given start point, list of waypoints, and end point.
     * 
     * @param tankSubsystem The tank subsystem.
     * @param start The start point of the trajectory as a Pose2d.
     * @param waypoints A list of waypoints the robot must pass through as a List<Translation2d>.
     * @param end The end point of the trajectory as a Pose2d.
     */
    public FollowPathCommand(TankSubsystem tankSubsystem, Pose2d start, List<Translation2d> waypoints, Pose2d end) {
        this(
            tankSubsystem,
            // Target trajectory
            TrajectoryGenerator.generateTrajectory(
                start, waypoints, end, 
                new TrajectoryConfig(MAX_VEL, MAX_ACCEL)
                    .setKinematics(KINEMATICS)
                    .addConstraint(new DifferentialDriveVoltageConstraint(
                        new SimpleMotorFeedforward(Ks, Kv, Ka), 
                        KINEMATICS, 
                        10))
            )
        );
    }
}
