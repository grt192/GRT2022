package frc.robot.odometry;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpiutil.math.MatBuilder;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpilibj.SPI;

import static frc.robot.Constants.TankConstants.*;

/**
 * A localization class which stores the robot's position and contains helper methods to
 * read and manipulate said position data.
 */
public class Odometry {
  private final WPI_TalonSRX leftMain;
  private final WPI_TalonSRX rightMain;

  private final AHRS ahrs;
  private final DifferentialDrivePoseEstimator poseEstimator;

  public static final double ENCODER_TICKS_TO_INCHES = 32 / 142.40;

  public Odometry() {
    // Ititialize motors
    // Talon sensor configuration is handled already in tankSubsystem so don't do them again
    leftMain = new WPI_TalonSRX(fLeftMotorPort);
    rightMain = new WPI_TalonSRX(fRightMotorPort);

    // Initialize navX AHRS
    // https://www.kauailabs.com/public_files/navx-mxp/apidocs/java/com/kauailabs/navx/frc/AHRS.html
    ahrs = new AHRS(SPI.Port.kMXP); 

    // Initialize odometry class
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose_state-estimators.html
    poseEstimator = new DifferentialDrivePoseEstimator(new Rotation2d(), new Pose2d(),
      new MatBuilder<>(Nat.N5(), Nat.N1()).fill(0.02, 0.02, 0.01, 0.02, 0.02), // State measurement standard deviations. X, Y, theta.
      new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01), // Local measurement standard deviations. Left encoder, right encoder, gyro.
      new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01)); // Global measurement standard deviations. X, Y, and theta. 

    // Launch odometry thread
    OdometryThread odoThread = new OdometryThread(this);
    new Thread(odoThread).start();
  }

  /**
   * Updates the `PoseEstimator`'s current position with new readings.
   * This method is called every millisecond by `OdometryThread`.
   */
  public void updateCurrentPosition() {
    // Update odometry readings
    Rotation2d gyroAngle = Rotation2d.fromDegrees(ahrs.getAngle());
    DifferentialDriveWheelSpeeds wheelVelocities = new DifferentialDriveWheelSpeeds(
      leftMain.getSelectedSensorVelocity() * ENCODER_TICKS_TO_INCHES, 
      rightMain.getSelectedSensorVelocity() * ENCODER_TICKS_TO_INCHES);
    double leftDistance = leftMain.getSelectedSensorPosition() * ENCODER_TICKS_TO_INCHES;
    double rightDistance = rightMain.getSelectedSensorPosition() * ENCODER_TICKS_TO_INCHES;

    poseEstimator.update(gyroAngle, wheelVelocities, leftDistance, rightDistance);

    System.out.println("Odometry readings: " + poseEstimator.getEstimatedPosition());
  }

  /**
   * Gets the estimated current position of the robot.
   * @return the estimated position of the robot as a Pose2d.
   */
  public Pose2d getRobotPosition() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Zeros the robot's position.
   * This method zeros both the robot's translation *and* rotation.
   */
  public void zeroPosition() {
    leftMain.setSelectedSensorPosition(0);
    rightMain.setSelectedSensorPosition(0);

    // https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/wpilibj/estimator/DifferentialDrivePoseEstimator.html#resetPosition(edu.wpi.first.wpilibj.geometry.Pose2d,edu.wpi.first.wpilibj.geometry.Rotation2d)
    poseEstimator.resetPosition(new Pose2d(), Rotation2d.fromDegrees(ahrs.getAngle()));
  }
}
