package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.IntakeSubsystem;

public class DeployIntakeCommand extends InstantCommand {
    private final IntakeSubsystem intake;

    public DeployIntakeCommand(IntakeSubsystem intake) {
        this.intake = intake;
    }

    @Override
    public void initialize() {
        intake.setPosition(IntakeSubsystem.IntakePosition.DEPLOYED);
    }
}
