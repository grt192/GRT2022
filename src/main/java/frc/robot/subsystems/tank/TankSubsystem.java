// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.tank;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.TankConstants.*;

public class TankSubsystem extends SubsystemBase {
  private final CANSparkMax leftMain;
  private final CANSparkMax leftFollow;

  private final CANSparkMax rightMain;
  private final CANSparkMax rightFollow;

  public TankSubsystem() {
    // Init left main and follower motors
    leftMain = new CANSparkMax(fLeftMotorPort, MotorType.kBrushless);
    leftMain.setIdleMode(IdleMode.kBrake);

    leftFollow = new CANSparkMax(bLeftMotorPort, MotorType.kBrushless);
    leftFollow.follow(leftMain);
    leftFollow.setIdleMode(IdleMode.kBrake);

    // Init right main and follower motors
    rightMain = new CANSparkMax(fRightMotorPort, MotorType.kBrushless);
    rightMain.setInverted(true);
    rightMain.setIdleMode(IdleMode.kBrake);

    rightFollow = new CANSparkMax(bRightMotorPort, MotorType.kBrushless);
    rightFollow.follow(rightMain);
    rightFollow.setIdleMode(IdleMode.kBrake);
  }

  /**
   * Drive the system with the given power scales using the car system.
   * 
   * @param yScale       Scale in the forward/backward direction, from 1 to -1.
   * @param angularScale Scale in the rotational direction, from 1 to -1,
   *                     clockwise to counterclockwise.
   */
  public void setCarDrivePowers(double yScale, double angularScale) {
    setCarDrivePowers(yScale, angularScale, true);
  }

  public void setCarDrivePowers(double yScale, double angularScale, boolean squareInput) {
    // Square the input if needed for finer control
    if (squareInput) {
      yScale = squareInput(yScale);
      angularScale = squareInput(angularScale);
    }

    // Set motor output state
    double leftPowerTemp = yScale + angularScale;
    double rightPowerTemp = yScale - angularScale;

    // Scale powers greater than 1 back to 1 if needed
    double largest_power = Math.max(Math.abs(leftPowerTemp), Math.abs(rightPowerTemp));
    if (largest_power > 1.0) {
      double scale = 1.0 / largest_power;

      leftPowerTemp *= scale;
      rightPowerTemp *= scale;
    }

    // Set motor output state
    leftMain.set(leftPowerTemp);
    rightMain.set(rightPowerTemp);
  }

  /**
   * Drive the system with the given power scales using the tank system.
   * 
   * @param leftScale  Scale in the forward/backward direction of the left motor,
   *                   from -1 to 1.
   * @param rightScale Scale in the forward/backward direction of the right motor,
   *                   from -1 to 1.
   */
  public void setTankDrivePowers(double leftScale, double rightScale, boolean squareInput) {
    if (squareInput) {
      leftScale = squareInput(leftScale);
      rightScale = squareInput(rightScale);
    }

    // Set motor output state
    leftMain.set(leftScale);
    rightMain.set(rightScale);
  }

  public void setTankDrivePowers(double leftScale, double rightScale) {
    setTankDrivePowers(leftScale, rightScale, false);
  }

  /**
   * Drive the system with the given voltage values for each side of the drivetrain.
   *  
   * @param leftVoltage left motor voltages
   * @param rightVoltage right motor voltages
   */
  public void setTankDriveVoltages(double leftVoltage, double rightVoltage) {
    leftMain.setVoltage(leftVoltage);
    rightMain.setVoltage(rightVoltage);
  }

  /**
   * Squares input value while retaining original sign.
   * 
   * @param value the value to square
   * @return the squared value
   */
  private double squareInput(double value) {
    return Math.copySign(value * value, value);
  }
}
