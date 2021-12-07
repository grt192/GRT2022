package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.claw.ClawSubsystem;

public class CloseClawCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final ClawSubsystem clawSubsystem;

    public CloseClawCommand(ClawSubsystem clawSubsystem) {
        this.clawSubsystem = clawSubsystem;

        addRequirements(clawSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // Set motor powers to close the claw
        clawSubsystem.setClosePowers();
    }

    @Override
    public boolean isFinished() {
        // If both motors are stalled, command is finished
        return clawSubsystem.areBothMotorsStalled();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop motors
        clawSubsystem.setNeutralPowers();
    }

}
