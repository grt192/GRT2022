// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tank;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TankSubsystem;

/** An example command that uses an example subsystem. */
public class DriveTankCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final TankSubsystem tankSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param tankSubsystem The subsystem used by this command.
   */
  public DriveTankCommand(TankSubsystem tankSubsystem) { // TODO: add parameters whenever swerveSubsystem.setDrivePowers() is finished.
    this.tankSubsystem = tankSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(tankSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void initialize() {
    tankSubsystem.setDrivePowers(); // TODO: add parameters whenever the method is fixed
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
