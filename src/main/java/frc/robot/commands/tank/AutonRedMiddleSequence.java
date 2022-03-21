package frc.robot.commands.tank;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.internals.InternalSubsystem;
import frc.robot.subsystems.tank.TankSubsystem;

import static frc.robot.Constants.BallCoordinates.*;

/**
 * The middle red autonomous sequence. This assumes we start in the middle position, facing away from the hub.
 * For all autonomous sequences, we drive forward and intake a ball, then shoot both cargo into the hub.
 * For the middle sequence specifically, after shooting both balls off the tarmac, drive to and shoot
 * the ball from the human player terminal.
 */
public class AutonRedMiddleSequence extends GRTAutonSequence {
    // Assuming the robot is aligned to the far right edge of the top red tarmac,
    // with the robot's bottom right corner touching the hub.
    private static Pose2d initialPose = new Pose2d(
        Units.inchesToMeters(44.575), 
        Units.inchesToMeters(41.328), 
        Rotation2d.fromDegrees(24));
    private static Pose2d ballOnePose = localizeBallCoordinate(RIGHT_MID_RED, 35.2482302738);
    private static Pose2d ballTwoPose = localizeBallCoordinate(TERMINAL_RED, 45);

    public AutonRedMiddleSequence(RobotContainer robotContainer, TankSubsystem tankSubsystem, InternalSubsystem internalSubsystem, IntakeSubsystem intakeSubsystem) {
        super(robotContainer, tankSubsystem, internalSubsystem, intakeSubsystem, initialPose, ballOnePose, ballTwoPose);
    }
}
