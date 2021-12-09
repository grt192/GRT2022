// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.tank;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MatBuilder;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpilibj.SPI;

import static frc.robot.Constants.TankConstants.*;

public class TankSubsystem extends SubsystemBase {
  private final WPI_TalonSRX leftMain;
  private final WPI_TalonSRX leftFollow;

  private final WPI_TalonSRX rightMain;
  private final WPI_TalonSRX rightFollow;

  private final AHRS ahrs;

  private final DifferentialDrivePoseEstimator odometry;

  public static final double ENCODER_TICKS_TO_INCHES = 32 / 142.40;

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

    // Initialize navX AHRS
    // https://www.kauailabs.com/public_files/navx-mxp/apidocs/java/com/kauailabs/navx/frc/AHRS.html
    ahrs = new AHRS(SPI.Port.kMXP); 

    // Initialize odometry class
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose_state-estimators.html
    odometry = new DifferentialDrivePoseEstimator(new Rotation2d(), new Pose2d(),
      new MatBuilder<>(Nat.N5(), Nat.N1()).fill(0.02, 0.02, 0.01, 0.02, 0.02), // State measurement standard deviations. X, Y, theta.
      new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01), // Local measurement standard deviations. Left encoder, right encoder, gyro.
      new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01)); // Global measurement standard deviations. X, Y, and theta. 

    // Initialize power values
    leftPower = 0;
    rightPower = 0;

    zeroPosition();
  }

  @Override
  public void periodic() {

    leftMain.set(ControlMode.PercentOutput, leftPower);
    rightMain.set(ControlMode.PercentOutput, rightPower);

    // Update odometry readings
    Rotation2d gyroAngle = Rotation2d.fromDegrees(ahrs.getAngle());
    DifferentialDriveWheelSpeeds wheelVelocities = new DifferentialDriveWheelSpeeds(
      leftMain.getSelectedSensorVelocity() * ENCODER_TICKS_TO_INCHES, 
      rightMain.getSelectedSensorVelocity() * ENCODER_TICKS_TO_INCHES);
    double leftDistance = leftMain.getSelectedSensorPosition() * ENCODER_TICKS_TO_INCHES;
    double rightDistance = rightMain.getSelectedSensorPosition() * ENCODER_TICKS_TO_INCHES;

    odometry.update(gyroAngle, wheelVelocities, leftDistance, rightDistance);

    System.out.println("Odometry readings: " + odometry.getEstimatedPosition());
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

  /**
   * Gets the estimated current position of the robot.
   * @return the estimated position of the robot as a Pose2d.
   */
  public Pose2d getRobotPosition() {
    return odometry.getEstimatedPosition();
  }

  /**
   * Zeros the robot's position.
   * This method zeros both the robot's translation *and* rotation.
   */
  public void zeroPosition() {
    leftMain.setSelectedSensorPosition(0);
    rightMain.setSelectedSensorPosition(0);

    // https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/wpilibj/estimator/DifferentialDrivePoseEstimator.html#resetPosition(edu.wpi.first.wpilibj.geometry.Pose2d,edu.wpi.first.wpilibj.geometry.Rotation2d)
    odometry.resetPosition(new Pose2d(), Rotation2d.fromDegrees(ahrs.getAngle()));
  }
}
