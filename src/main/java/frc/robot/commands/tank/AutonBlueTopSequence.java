package frc.robot.commands.tank;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import frc.robot.RobotContainer;

import static frc.robot.Constants.BallCoordinates.*;

/**
 * The top blue autonomous sequence. This assumes we start in the top position, facing away from the hub.
 * For all autonomous sequences, we drive forward and intake a ball, then shoot both cargo into the hub.
 */
public class AutonBlueTopSequence extends GRTAutonSequence {
    // Assuming the robot is aligned to the far right edge of the top blue tarmac,
    // with the robot's bottom right corner touching the hub.
    private static final Pose2d initialPose = new Pose2d(
        Units.inchesToMeters(-41.328), 
        Units.inchesToMeters(44.575), 
        Rotation2d.fromDegrees(114));
    private static final Pose2d ballOnePose = localizeToRobotFront(LEFT_TOP_BLUE, 147.74823);

    public AutonBlueTopSequence(RobotContainer robotContainer) {
        super(robotContainer, initialPose, ballOnePose);
    }
}
