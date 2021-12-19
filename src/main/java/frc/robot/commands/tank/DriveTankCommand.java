// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tank;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.odometry.Odometry;
import frc.robot.subsystems.tank.TankSubsystem;

/**
 * A command to drive the robot straight for x inches
 * Currently a rough implementation for testing; 
 * 
 * TODO: use closed loop to prevent turning
 * TODO: is this necessary with FollowPathCommand existing?
 */
public class DriveTankCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final TankSubsystem tankSubsystem;
  private final Odometry odometry;

  private double distance;

  public DriveTankCommand(TankSubsystem tankSubsystem, Odometry odometry, double inches) {
    this.tankSubsystem = tankSubsystem;
    this.odometry = odometry;

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
    odometry.resetPosition();

    tankSubsystem.setCarDrivePowers(0.5, 0.0);
  }

  @Override
  public boolean isFinished() {
    // Finish if the x component of the robot's translation is greater than or equal to the desired distance
    // x direction -> forward displacement when angle = 0
    return odometry.getRobotPosition().getX() >= distance;
  }

  @Override
  public void end(boolean interrupted) {
    tankSubsystem.setCarDrivePowers(0, 0, false);
  }
}
