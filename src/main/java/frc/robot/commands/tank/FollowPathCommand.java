package frc.robot.commands.tank;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

import frc.robot.odometry.Odometry;
import frc.robot.subsystems.tank.TankSubsystem;

/**
 * A command to drive the robot from some start to end point, passing through given waypoints
 * in the path. Start and end are represented as `Pose2d`s while waypoints are given as a List of
 * `Translation2d`. All units are given in meters.
 * 
 * https://docs.wpilib.org/en/stable/docs/software/examples-tutorials/trajectory-tutorial/trajectory-tutorial-overview.html
 */
public class FollowPathCommand extends RamseteCommand {

  // Robot constants
  // TODO: measure this
  private static final double ROBOT_WIDTH_METERS = 0.7;
  private static final DifferentialDriveKinematics KINEMATICS = 
    new DifferentialDriveKinematics(ROBOT_WIDTH_METERS);

  // Drive constants
  // TODO: measure these with the Robot Characterization Toolsuite
  // https://docs.wpilib.org/en/stable/docs/software/examples-tutorials/trajectory-tutorial/characterizing-drive.html
  private static final double Ks = 0;
  private static final double Kv = 0;
  private static final double Ka = 0;
  private static final double Kp = 0;

  // Velocity / Acceleration constants (m/s, m/s^2)
  private static final double MAX_VEL = 3;
  private static final double MAX_ACCEL = 3;

  // Ramsete constants
  private static final double RAMSETE_B = 2;
  private static final double RAMSETE_ZETA = 0.7;
  
  /**
   * Creates a FollowPathCommand from a given trajectory.
   * 
   * @param tankSubsystem The tank subsystem.
   * @param odometry The odometry object.
   * @param trajectory The trajectory to follow.
   */
  public FollowPathCommand(TankSubsystem tankSubsystem, Odometry odometry, Trajectory trajectory) {
    super(
      trajectory,
      odometry::getRobotPosition, // Position supplier
      new RamseteController(RAMSETE_B, RAMSETE_ZETA),
      new SimpleMotorFeedforward(Ks, Kv, Ka),
      KINEMATICS,
      odometry::getWheelSpeeds, // Wheel speed supplier
      // PID controllers
      new PIDController(Kp, 0, 0),
      new PIDController(Kp, 0, 0),
      tankSubsystem::setTankDriveVoltages, // Wheel voltage consumer
      tankSubsystem
    );

    odometry.resetPosition(trajectory.getInitialPose());
  }

  /**
   * Creates a FollowPathCommand from a given start point, list of waypoints, and end point.
   * 
   * @param tankSubsystem The tank subsystem.
   * @param odometry The odometry object.
   * @param start The start point of the trajectory as a Pose2d.
   * @param waypoints A list of waypoints the robot must pass through as a List<Translation2d>.
   * @param end The end point of the trajectory as a Pose2d.
   */
  public FollowPathCommand(TankSubsystem tankSubsystem, Odometry odometry, Pose2d start, List<Translation2d> waypoints, Pose2d end) {
    this(
      tankSubsystem,
      odometry,
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
