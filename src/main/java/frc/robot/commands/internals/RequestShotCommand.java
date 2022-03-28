package frc.robot.commands.internals;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.internals.InternalSubsystem;

/**
 * Asynchronously requests a shot be fired. The shot will *actually* be fired when a ball is in staging and
 * the turret is lined up. Calling requestShot repeatedly has no effect; this merely requests that the next 
 * opportunity internals has to shoot a ball is taken.
 */
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
