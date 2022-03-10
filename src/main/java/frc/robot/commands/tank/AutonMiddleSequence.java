package frc.robot.commands.tank;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.internals.ShootCommand;
import frc.robot.subsystems.internals.InternalSubsystem;
import frc.robot.subsystems.tank.TankSubsystem;

import static frc.robot.Constants.BallCoordinates.*;

/**
 * The middle autonomous sequence. This assumes we start in the middle position, facing away from the hub.
 * For all autonomous sequences, we drive forward and intake a ball, then shoot both cargo into the hub.
 * For the middle sequence specifically, after shooting both balls off the tarmac, drive to and shoot
 * the ball from the human player terminal.
 */
public class AutonMiddleSequence extends SequentialCommandGroup {
    // TODO: decide on these
    private static Pose2d initialPose = new Pose2d(0, 0, Rotation2d.fromDegrees(-66));
    private static Pose2d ballOnePose = new Pose2d(RIGHT_MID_RED, Rotation2d.fromDegrees(-54.7514582647));

    public AutonMiddleSequence(TankSubsystem tankSubsystem, InternalSubsystem internalSubsystem) {
        addRequirements(tankSubsystem, internalSubsystem);
        addCommands(
            new FollowPathCommand(tankSubsystem, initialPose, List.of(), ballOnePose),
            new ShootCommand(internalSubsystem),
            new ShootCommand(internalSubsystem),
            new FollowPathCommand(tankSubsystem, ballOnePose, List.of(), TERMINAL_RED),
            new ShootCommand(internalSubsystem)
        );
    }
}
