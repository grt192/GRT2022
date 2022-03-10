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
 * The top autonomous sequence. This assumes we start in the top position, facing away from the hub.
 * For all autonomous sequences, we drive forward and intake a ball, then shoot both cargo into the hub.
 */
public class AutonTopSequence extends SequentialCommandGroup {
    // TODO: decide on this
    public static final Pose2d initialPose = new Pose2d(0, 0, new Rotation2d());

    public AutonTopSequence(TankSubsystem tankSubsystem, InternalSubsystem internalSubsystem) {
        addRequirements(tankSubsystem, internalSubsystem);
        addCommands(
            new FollowPathCommand(tankSubsystem, initialPose, List.of(), new Pose2d(RIGHT_TOP_RED, new Rotation2d())),
            new ShootCommand(internalSubsystem),
            new ShootCommand(internalSubsystem)
        );
    }
}
