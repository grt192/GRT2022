// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tank;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.tank.TankSubsystem;

/** A command to drive the robot straight for x inches */
// Just a rough implementation for testing;
// TODO: make actually useful commands
public class DriveTankCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final TankSubsystem tankSubsystem;

  private double distance;

  public DriveTankCommand(TankSubsystem tankSubsystem, double inches) {
    this.tankSubsystem = tankSubsystem;
    
    addRequirements(tankSubsystem);

    this.distance = inches;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void initialize() {
    // TODO: the zeroing of the sensors takes a non-zero amount of time and `isFinished()` returns 
    // true before the sensors are zeroed.
    // Every other run of this command will fail, with the second run serving only to zero the sensors 
    // for the third.
    tankSubsystem.zeroPosition();

    tankSubsystem.setCarDrivePowers(0.5, 0.0);
  }

  @Override
  public boolean isFinished() {
    return true;
    //return tankSubsystem.getLeftPosition() >= distance
    //  && tankSubsystem.getRightPosition() >= distance;
  }

  @Override
  public void end(boolean interrupted) {
    tankSubsystem.setCarDrivePowers(0, 0, false);
  }
}
