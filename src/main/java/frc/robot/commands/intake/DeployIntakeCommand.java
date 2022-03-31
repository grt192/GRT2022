package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakePosition;

/**
 * Deploys and runs the intake at a supplied power (defaults to 1.0).
 */
public class DeployIntakeCommand extends InstantCommand {
    private final IntakeSubsystem intakeSubsystem;
    private final double intakePow;

    public DeployIntakeCommand(IntakeSubsystem intakeSubsystem, double intakePow) {
        this.intakeSubsystem = intakeSubsystem;
        this.intakePow = intakePow;
        addRequirements(intakeSubsystem);
    }

    public DeployIntakeCommand(IntakeSubsystem intakeSubsystem) {
        this(intakeSubsystem, 1.0);
    }

    @Override
    public void initialize() {
        intakeSubsystem.setPosition(IntakePosition.DEPLOYED);
        intakeSubsystem.setAutoOverride(true);
        intakeSubsystem.setAutoPower(intakePow);
    }
}
