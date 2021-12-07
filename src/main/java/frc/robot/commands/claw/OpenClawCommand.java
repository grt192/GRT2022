package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.claw.ClawSubsystem;

public class OpenClawCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final ClawSubsystem clawSubsystem;
    private JoystickButton button;

    public OpenClawCommand(ClawSubsystem clawSubsystem, JoystickButton button) {
        this.clawSubsystem = clawSubsystem;
        this.button = button;

        addRequirements(clawSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // Set motor powers to open the claw
        clawSubsystem.setOpenPowers();
    }

    @Override
    public boolean isFinished() {
        // If button is still pressed, the command is not finished
        return !button.get();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop motors
        clawSubsystem.setNeutralPowers();
    }

}
