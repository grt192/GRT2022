package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakePosition;

/**
 * Raises the intake and stops the intake rollers.
 */
public class RaiseIntakeCommand extends InstantCommand {
    private final IntakeSubsystem intakeSubsystem;

    public RaiseIntakeCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.setPosition(IntakePosition.RAISED);
        intakeSubsystem.setDriverOverride(false);
    }
}
