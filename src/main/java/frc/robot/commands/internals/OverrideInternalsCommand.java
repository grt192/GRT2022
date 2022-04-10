package frc.robot.commands.internals;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.internals.InternalSubsystem;

/**
 * A continuously running command to manage driver overriding of the flywheel (on the left bumper)
 * and internals (on the right bumper). When the right bumper is pressed, start the flywheel before
 * internals to give it time to spin up.
 */
public class OverrideInternalsCommand extends RunCommand {
    // TODO: will this semi-hacky `static` modifier have adverse consequences?
    private static final Timer internalTimer = new Timer();

    public OverrideInternalsCommand(InternalSubsystem internalSubsystem, TurretSubsystem turretSubsystem, XboxController xboxController) {
        super(() -> {
            boolean leftBumper = xboxController.getLeftBumper();
            boolean rightBumper = xboxController.getRightBumper();

            // Override the flywheel if either bumper is pressed
            turretSubsystem.setDriverOverrideFlywheel(leftBumper || rightBumper);

            if (rightBumper) {
                // If the driver is overriding internals and the required spinup time for the flywheel
                // has passed, override internals
                if (internalTimer.hasElapsed(0.25)) {
                    internalSubsystem.setDriverOverride(true);
                } else {
                    // If the time hasn't elapsed, start the timer
                    internalTimer.start();
                }
            } else {
                internalSubsystem.setDriverOverride(false);
                internalTimer.stop();
                internalTimer.reset();
            }
        }, internalSubsystem);
    }
}
