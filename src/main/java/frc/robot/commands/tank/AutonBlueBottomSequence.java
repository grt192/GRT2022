package frc.robot.commands.tank;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import frc.robot.RobotContainer;

import static frc.robot.Constants.BallCoordinates.*;

/**
 * The bottom blue autonomous sequence. This assumes we start in the top position, facing away from the hub.
 * For all autonomous sequences, we drive forward and intake a ball, then shoot both cargo into the hub.
 */
public class AutonBlueBottomSequence extends GRTAutonSequence {
    // Assuming the robot is aligned to the far left (right) edge of the bottom blue tarmac,
    // with the robot's bottom left corner touching the hub.
    private static final Pose2d initialPose = new Pose2d(
        Units.inchesToMeters(5.472), 
        Units.inchesToMeters(-60.540), 
        Rotation2d.fromDegrees(-66)); // TODO: are negative angles ok?
    private static final Pose2d ballOnePose = localizeToIntakeFront(LEFT_BOTTOM_BLUE, 260.251594482);

    public AutonBlueBottomSequence(RobotContainer robotContainer) {
        super(robotContainer, initialPose, ballOnePose);
    }
}
