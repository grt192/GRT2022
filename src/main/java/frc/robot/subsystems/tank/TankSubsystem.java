// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.tank;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.TankConstants.*;

public class TankSubsystem extends SubsystemBase {
  private final WPI_TalonSRX leftMain;
  private final WPI_TalonSRX leftFollow;

  private final WPI_TalonSRX rightMain;
  private final WPI_TalonSRX rightFollow;

  // Motor power output states
  private double leftPower;
  private double rightPower;

  public TankSubsystem() {
    // Init left main and follower motors
    leftMain = new WPI_TalonSRX(fLeftMotorPort);
    leftMain.setNeutralMode(NeutralMode.Brake);

    leftFollow = new WPI_TalonSRX(bLeftMotorPort);
    leftFollow.follow(leftMain);
    leftFollow.setInverted(InvertType.FollowMaster);
    leftFollow.setNeutralMode(NeutralMode.Brake);

    // Init right main and follower motors
    rightMain = new WPI_TalonSRX(fRightMotorPort);
    rightMain.setInverted(true);
    rightMain.setNeutralMode(NeutralMode.Brake);

    rightFollow = new WPI_TalonSRX(bRightMotorPort);
    rightFollow.follow(rightMain);
    rightFollow.setInverted(InvertType.FollowMaster);
    rightFollow.setNeutralMode(NeutralMode.Brake);

    // Initialize Talon sensors
    // https://docs.ctre-phoenix.com/en/latest/ch14_MCSensor.html#cross-the-road-electronics-magnetic-encoder-absolute-and-relative
    // https://docs.ctre-phoenix.com/en/latest/ch14_MCSensor.html#software-select-sensor
    leftMain.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    rightMain.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);

    // https://docs.ctre-phoenix.com/en/latest/ch14_MCSensor.html#sensor-phase
    leftMain.setSensorPhase(false);
    rightMain.setSensorPhase(false);

    // Initialize power values
    leftPower = 0;
    rightPower = 0;
  }

  @Override
  public void periodic() {

    leftMain.set(ControlMode.PercentOutput, leftPower);
    rightMain.set(ControlMode.PercentOutput, rightPower);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
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
    this.leftPower = leftPowerTemp;
    this.rightPower = rightPowerTemp;
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
    this.leftPower = leftScale;
    this.rightPower = rightScale;
  }

  public void setTankDrivePowers(double leftScale, double rightScale) {
    setTankDrivePowers(leftScale, rightScale, false);
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
