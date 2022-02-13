package frc.robot.commands.internals;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.internals.InternalSubsystem;

public class RequestShotCommand extends InstantCommand {
    private final InternalSubsystem internals;

    public RequestShotCommand(InternalSubsystem internals) {
        this.internals = internals;
        addRequirements(internals);
    }

    @Override
    public void initialize() {
        internals.requestShot();
    }
}
