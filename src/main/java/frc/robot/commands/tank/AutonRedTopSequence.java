package frc.robot.commands.tank;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import frc.robot.RobotContainer;

import static frc.robot.Constants.BallCoordinates.*;

/**
 * The top red autonomous sequence. This assumes we start in the top position, facing away from the hub.
 * For all autonomous sequences, we drive forward and intake a ball, then shoot both cargo into the hub.
 */
public class AutonRedTopSequence extends GRTAutonSequence {
    // Assuming the robot is aligned to the far left edge of the top red tarmac,
    // with the robot's bottom left corner touching the hub.
    private static final Pose2d initialPose = new Pose2d(
        Units.inchesToMeters(-5.472), 
        Units.inchesToMeters(60.540), 
        Rotation2d.fromDegrees(114));
    private static final Pose2d ballOnePose = localizeBallCoordinate(RIGHT_TOP_RED, 90);

    public AutonRedTopSequence(RobotContainer robotContainer) {
        super(robotContainer, initialPose, ballOnePose);
    }
}
