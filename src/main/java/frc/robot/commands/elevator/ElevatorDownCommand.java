// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ElevatorDownCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final ElevatorSubsystem elevatorSubsystem;

  public ElevatorDownCommand(ElevatorSubsystem elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;

    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    elevatorSubsystem.setElevatorPower(-1);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
