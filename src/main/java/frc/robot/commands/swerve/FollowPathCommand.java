package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class FollowPathCommand extends CommandBase {
    private final SwerveSubsystem swerveSubsystem;

    public FollowPathCommand(SwerveSubsystem swerveSubsystem) { // TODO add parameters
        this.swerveSubsystem = swerveSubsystem;

        addRequirements(swerveSubsystem);
    }

    // TODO
}
