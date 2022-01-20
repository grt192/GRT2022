// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.tank;

import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.TankConstants.*;

public class TankSubsystem extends SubsystemBase {
  private final CANSparkMax leftMain;
  private final CANSparkMax leftFollow;

  private final CANSparkMax rightMain;
  private final CANSparkMax rightFollow;

  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;

  private final AHRS ahrs;
  private final DifferentialDrivePoseEstimator poseEstimator;

  private final ShuffleboardTab shuffleboardTab;
  private final NetworkTableEntry shuffleboardXEntry;
  private final NetworkTableEntry shuffleboardYEntry;
  private final NetworkTableEntry shuffleboardAngleEntry;
  private final Field2d shuffleboardField;

  public static final double ENCODER_ROTATIONS_TO_METERS = 5 / 92.08;

  public TankSubsystem() {
    // Init left main and follower motors and encoders
    leftMain = new CANSparkMax(fLeftMotorPort, MotorType.kBrushless);
    leftMain.restoreFactoryDefaults();
    leftMain.setInverted(true);
    leftMain.setIdleMode(IdleMode.kBrake);

    // Position conversion: Rotations -> m
    // Velocity conversion: RPM -> m/s
    leftEncoder = leftMain.getEncoder();
    leftEncoder.setPositionConversionFactor(ENCODER_ROTATIONS_TO_METERS);
    leftEncoder.setVelocityConversionFactor(ENCODER_ROTATIONS_TO_METERS / 60.0);

    leftFollow = new CANSparkMax(bLeftMotorPort, MotorType.kBrushless);
    leftFollow.restoreFactoryDefaults();
    leftFollow.follow(leftMain);
    leftFollow.setIdleMode(IdleMode.kBrake);

    // Init right main and follower motors
    rightMain = new CANSparkMax(fRightMotorPort, MotorType.kBrushless);
    rightMain.restoreFactoryDefaults();
    //rightMain.setInverted(true);
    rightMain.setIdleMode(IdleMode.kBrake);

    rightEncoder = rightMain.getEncoder();
    rightEncoder.setPositionConversionFactor(ENCODER_ROTATIONS_TO_METERS);
    rightEncoder.setVelocityConversionFactor(ENCODER_ROTATIONS_TO_METERS / 60.0);

    rightFollow = new CANSparkMax(bRightMotorPort, MotorType.kBrushless);
    rightFollow.restoreFactoryDefaults();
    rightFollow.follow(rightMain);
    rightFollow.setIdleMode(IdleMode.kBrake);

    // Initialize navX AHRS
    // https://www.kauailabs.com/public_files/navx-mxp/apidocs/java/com/kauailabs/navx/frc/AHRS.html
    ahrs = new AHRS(SPI.Port.kMXP); 

    // Initialize odometry class
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose_state-estimators.html
    poseEstimator = new DifferentialDrivePoseEstimator(new Rotation2d(), new Pose2d(),
      new MatBuilder<>(Nat.N5(), Nat.N1()).fill(0.02, 0.02, 0.01, 0.02, 0.02), // State measurement standard deviations. X, Y, theta.
      new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01), // Local measurement standard deviations. Left encoder, right encoder, gyro.
      new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01)); // Global measurement standard deviations. X, Y, and theta.

    // Initialize Shuffleboard entries
    shuffleboardTab = Shuffleboard.getTab("Drivetrain");
    shuffleboardXEntry = shuffleboardTab.add("Robot x", 0).getEntry();
    shuffleboardYEntry = shuffleboardTab.add("Robot y", 0).getEntry();
    shuffleboardAngleEntry = shuffleboardTab.add("Gyro angle", 0).getEntry();
    shuffleboardField = new Field2d();
    shuffleboardTab.add("Field", shuffleboardField);
  }

  /**
   * Drive the system with the given power scales using the car system.
   * 
   * @param yScale       Scale in the forward/backward direction, from 1 to -1.
   * @param angularScale Scale in the rotational direction, from 1 to -1, clockwise to counterclockwise.
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
   * @param leftScale  Scale in the forward/backward direction of the left motor, from -1 to 1.
   * @param rightScale Scale in the forward/backward direction of the right motor, from -1 to 1.
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
   * Drive the system with the given voltage values for each side of the
   * drivetrain.
   * 
   * @param leftVoltage  Left motor voltages.
   * @param rightVoltage Right motor voltages.
   */
  public void setTankDriveVoltages(double leftVoltage, double rightVoltage) {
    leftMain.setVoltage(leftVoltage);
    rightMain.setVoltage(rightVoltage);
  }

  @Override
  public void periodic() {
    // Update odometry readings
    Rotation2d gyroAngle = getRobotHeading();
    DifferentialDriveWheelSpeeds wheelVelocities = getWheelSpeeds();
    double leftDistance = leftEncoder.getPosition();
    double rightDistance = rightEncoder.getPosition();

    poseEstimator.update(gyroAngle, wheelVelocities, leftDistance, rightDistance);

    // Update Shuffleboard entries
    Pose2d pose = poseEstimator.getEstimatedPosition();
    shuffleboardXEntry.setDouble(pose.getX());
    shuffleboardYEntry.setDouble(pose.getY());
    shuffleboardAngleEntry.setDouble(pose.getRotation().getDegrees());
    shuffleboardField.setRobotPose(pose);

    System.out.println("Odometry readings: " + poseEstimator.getEstimatedPosition());
  }

    /**
   * Gets the estimated current position of the robot.
   * @return The estimated position of the robot as a Pose2d.
   */
  public Pose2d getRobotPosition() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Gets the gyro angle given by the NavX AHRS, inverted to be counterclockwise positive.
   * @return The robot heading as a Rotation2d.
   */
  public Rotation2d getRobotHeading() {
    return Rotation2d.fromDegrees(-ahrs.getAngle());
  }

  /**
   * Gets the wheel speeds of the drivetrain.
   * @return The drivetrain wheel speeds as a DifferentialDriveWheelSpeeds object.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      leftEncoder.getVelocity(), 
      rightEncoder.getVelocity());
  }

  /**
   * Reset the robot's position to a given Pose2d.
   * @param position The position to reset the pose estimator to.
   */
  public void resetPosition(Pose2d position) {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    // https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/wpilibj/estimator/DifferentialDrivePoseEstimator.html#resetPosition(edu.wpi.first.wpilibj.geometry.Pose2d,edu.wpi.first.wpilibj.geometry.Rotation2d)
    poseEstimator.resetPosition(position, getRobotHeading());
  }

  /**
   * Zeros the robot's position.
   * This method zeros both the robot's translation *and* rotation.
   */
  public void resetPosition() {
    resetPosition(new Pose2d());
  }

  /**
   * Squares input value while retaining original sign.
   * 
   * @param value The value to square.
   * @return The squared value.
   */
  private double squareInput(double value) {
    return Math.copySign(value * value, value);
  }
}
