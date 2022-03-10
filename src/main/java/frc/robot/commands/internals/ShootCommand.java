package frc.robot.commands.internals;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.internals.InternalSubsystem;

/**
 * Requests a shot to be fired, and waits until it is. This is used in autonomous command groups to
 * make sure shots are fired when they are supposed to.
 */
public class ShootCommand extends CommandBase {
    private final InternalSubsystem internals;

    private boolean shotRequested = false;

    public ShootCommand(InternalSubsystem internals) {
        this.internals = internals;
        addRequirements(internals);
    }

    @Override
    public void initialize() {
        internals.requestShot();
        shotRequested = true;
    }

    @Override
    public boolean isFinished() {
        // If the internals shot has finished (the subsystem no longer counts a shot as requested),
        // mark this command as finished.
        // TODO: if initialize() is always called before this, we can remove `shotRequested` entirely
        return shotRequested && !internals.getShotRequested();
    }
}
