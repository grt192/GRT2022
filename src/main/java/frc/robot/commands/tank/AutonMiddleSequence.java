package frc.robot.commands.tank;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.internals.ShootCommand;
import frc.robot.subsystems.internals.InternalSubsystem;
import frc.robot.subsystems.tank.TankSubsystem;

/**
 * The middle autonomous sequence. This assumes we start in the middle position, facing away from the hub.
 * For all autonomous sequences, we drive forward and intake a ball, then shoot both cargo into the hub.
 * For the middle sequence specifically, after shooting both balls off the tarmac, drive to and shoot
 * the ball from the human player terminal.
 */
public class AutonMiddleSequence extends SequentialCommandGroup {
    // TODO: measure these
    private static Pose2d initialPose = new Pose2d(0, 0, Rotation2d.fromDegrees(-66));
    private static Pose2d ballOnePose = new Pose2d(
        Units.inchesToMeters(88.3000339775), 
        Units.inchesToMeters(124.94840535), 
        Rotation2d.fromDegrees(-54.7514582647));
    private static Pose2d ballTwoPose = new Pose2d(0, 0, Rotation2d.fromDegrees(-45));

    public AutonMiddleSequence(TankSubsystem tankSubsystem, InternalSubsystem internalSubsystem) {
        addRequirements(tankSubsystem, internalSubsystem);
        addCommands(
            new FollowPathCommand(tankSubsystem, initialPose, List.of(), ballOnePose),
            new ShootCommand(internalSubsystem),
            new ShootCommand(internalSubsystem),
            new FollowPathCommand(tankSubsystem, ballOnePose, List.of(), ballTwoPose),
            new ShootCommand(internalSubsystem)
        );
    }
}
