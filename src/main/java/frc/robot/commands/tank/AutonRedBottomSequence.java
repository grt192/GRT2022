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
 * The bottom red autonomous sequence. This assumes we start in the top position, facing away from the hub.
 * For all autonomous sequences, we drive forward and intake a ball, then shoot both cargo into the hub.
 */
public class AutonRedBottomSequence extends GRTAutonSequence {
    // Assuming the robot is aligned to the far left (bottom) edge of the bottom red tarmac,
    // with the robot's bottom right corner touching the hub.
    private static final Pose2d initialPose = new Pose2d(
        Units.inchesToMeters(41.328), 
        Units.inchesToMeters(-44.575), 
        Rotation2d.fromDegrees(-66)); // TODO: are negative angles ok?
    private static final Pose2d ballOnePose = localizeBallCoordinate(RIGHT_BOTTOM_RED, -32.2517702051);

    public AutonRedBottomSequence(RobotContainer robotContainer, TankSubsystem tankSubsystem, InternalSubsystem internalSubsystem, IntakeSubsystem intakeSubsystem) {
        super(robotContainer, tankSubsystem, internalSubsystem, intakeSubsystem, initialPose, ballOnePose);
    }
}
