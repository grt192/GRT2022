package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.claw.ClawSubsystem;

public class CloseAndLiftClawCommand extends CommandBase {

    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final ClawSubsystem clawSubsystem;

    public CloseAndLiftClawCommand(ClawSubsystem clawSubsystem) {
        this.clawSubsystem = clawSubsystem;

        addRequirements(clawSubsystem);
    }

    @Override
    public void initialize() {
        // Once this command is triggered, close the claw
        clawSubsystem.clawIsOpen = false;
    }

    @Override
    public void execute() {
        // Repeatedly check if the claw-closing is finished
        if (clawSubsystem.isMotorStalled()) {
            // If finished closing claw, lift the claw
            clawSubsystem.clawIsLifted = true;
        }
    }

    @Override
    public boolean isFinished() {
        // If command has completed operation
        if (clawSubsystem.clawIsLifted) {
            return true;
        }
        return false;
    }
}
