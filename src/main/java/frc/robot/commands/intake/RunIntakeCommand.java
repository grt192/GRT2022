package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.subsystems.IntakeSubsystem;

/**
 * A continuously running command to manage driver overriding of intake's automatic roller behavior.
 * When the driver supplies manual input to the intake via the controller triggers, it will override the intake power
 * and relinquish control after 2 seconds.
 */
public class RunIntakeCommand extends RunCommand {
    // TODO: will this semi-hacky `static` modifier have adverse consequences?
    private static final Timer overrideTimer = new Timer();

    public RunIntakeCommand(IntakeSubsystem intakeSubsystem, XboxController xboxController) {
        super(() -> {
            double leftTrigger = xboxController.getLeftTriggerAxis();
            double rightTrigger = xboxController.getRightTriggerAxis();

            // If the driver is supplying intake power with the triggers, override intake control
            if (leftTrigger > 0 || rightTrigger > 0) {
                intakeSubsystem.setDriverOverride(true);
                intakeSubsystem.setIntakePower(rightTrigger - leftTrigger);

                overrideTimer.reset();
                overrideTimer.start();
            } else if (overrideTimer.hasElapsed(2)) {
                // Otherwise, if 2 seconds have passed without driver input, resume automatic intake control
                intakeSubsystem.setDriverOverride(false);
            }
        }, intakeSubsystem);
    }
}
