// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.tank;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.TankConstants.*;

public class TankSubsystem extends SubsystemBase {
  private final WPI_TalonSRX leftMain;
  private final WPI_TalonSRX leftFollow;

  private final WPI_TalonSRX rightMain;
  private final WPI_TalonSRX rightFollow;

  private final DifferentialDrive differentialDrive;

  // Motor power output states
  private double yScale;
  private double angularScale;

  public TankSubsystem() {
    // Init left main and follower motors
    leftMain = new WPI_TalonSRX(fLeftMotorPort);
    leftMain.setNeutralMode(NeutralMode.Brake);

    leftFollow = new WPI_TalonSRX(bLeftMotorPort);
    leftFollow.follow(leftMain);
    leftFollow.setInverted(InvertType.FollowMaster);
    leftFollow.setNeutralMode(NeutralMode.Brake);

    // Init right main and follower motors
    // Right motors are default inverted; see https://docs.wpilib.org/en/stable/docs/software/actuators/wpi-drive-classes.html#motor-inversion
    rightMain = new WPI_TalonSRX(fRightMotorPort);
    rightMain.setNeutralMode(NeutralMode.Brake);

    rightFollow = new WPI_TalonSRX(bRightMotorPort);
    rightFollow.follow(rightMain);
    rightFollow.setInverted(InvertType.FollowMaster);
    rightFollow.setNeutralMode(NeutralMode.Brake);

    // Class for driving differential (tank) drive systems
    // See https://first.wpi.edu/FRC/roborio/release/docs/java/edu/wpi/first/wpilibj/drive/DifferentialDrive.html
    differentialDrive = new DifferentialDrive(leftMain, rightMain);

    yScale = 0;
    angularScale = 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // TODO: odometry
    differentialDrive.tankDrive(yScale, angularScale);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /**
   * Drive the system with the given power scales.
   * 
   * @param yScale       Scale in the forward/backward direction, from 1 to -1.
   * @param angularScale Scale in the rotational direction, from 1 to -1,
   *                     clockwise to counterclockwise.
   */
  public void setDrivePowers(double yScale, double angularScale) {
    setDrivePowers(yScale, angularScale, true);
  }

  public void setDrivePowers(double yScale, double angularScale, boolean squareInput) {

    // Square the input if needed for finer control
    if (squareInput) {
      yScale = Math.copySign(yScale * yScale, yScale);
      angularScale = Math.copySign(angularScale * angularScale, angularScale);
    }

    double leftPower = yScale + angularScale;
    double rightPower = yScale - angularScale;

    // Scale powers greater than 1 back to 1 if needed
    double largest_power = Math.max(Math.abs(leftPower), Math.abs(rightPower));
    if (largest_power > 1.0) {
      double scale = 1.0 / largest_power;

      leftPower *= scale;
      rightPower *= scale;
    }

    // set state with new motor powers
    this.yScale = leftPower;
    this.angularScale = rightPower;
  }
}
