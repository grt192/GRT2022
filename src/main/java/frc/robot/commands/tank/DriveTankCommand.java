// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tank;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.tank.TankSubsystem;

/** An example command that uses an example subsystem. */
public class DriveTankCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final TankSubsystem tankSubsystem;

  private double yScale;
  private double angularScale;
  private boolean squareInput;


 //TODO can we concentrate this into one
  public DriveTankCommand(TankSubsystem tankSubsystem, double yScale, double angularScale) {
    this(tankSubsystem, yScale, angularScale, true);
  }

  public DriveTankCommand(TankSubsystem tankSubsystem, double yScale, double angularScale, boolean squareInput) {
    super();

    this.tankSubsystem = tankSubsystem;
    
    addRequirements(tankSubsystem);

    this.yScale = yScale;
    this.angularScale = angularScale;
    this.squareInput = squareInput;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void initialize() {
    tankSubsystem.setDrivePowers(yScale, angularScale, squareInput);
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  public void end() {
    tankSubsystem.setDrivePowers(0,0, false);
  }
}
