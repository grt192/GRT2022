package frc.robot.odometry;

import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
 * read and manipulate said position data. Measurements are given in meters.
 */
public class Odometry {
  private final CANSparkMax leftMain;
  private final CANSparkMax rightMain;

  private final AHRS ahrs;
  private final DifferentialDrivePoseEstimator poseEstimator;

  // For characterization:
  // Rotations -> meters = ENCODER_TICKS_TO_METERS * 42 (hall sensor ticks per revolution)
  public static final double ENCODER_TICKS_TO_METERS = 3 / 3274.70;

  public Odometry() {
    // Ititialize motors and encoder position/velocity scaling
    leftMain = new CANSparkMax(fLeftMotorPort, MotorType.kBrushless);
    leftMain.getEncoder().setPositionConversionFactor(ENCODER_TICKS_TO_METERS);
    leftMain.getEncoder().setVelocityConversionFactor(ENCODER_TICKS_TO_METERS);

    rightMain = new CANSparkMax(fRightMotorPort, MotorType.kBrushless);
    rightMain.getEncoder().setPositionConversionFactor(ENCODER_TICKS_TO_METERS);
    rightMain.getEncoder().setVelocityConversionFactor(ENCODER_TICKS_TO_METERS);

    // Initialize navX AHRS
    // https://www.kauailabs.com/public_files/navx-mxp/apidocs/java/com/kauailabs/navx/frc/AHRS.html
    ahrs = new AHRS(SPI.Port.kMXP); 

    // Initialize odometry class
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose_state-estimators.html
    poseEstimator = new DifferentialDrivePoseEstimator(new Rotation2d(), new Pose2d(),
      new MatBuilder<>(Nat.N5(), Nat.N1()).fill(0.02, 0.02, 0.01, 0.02, 0.02), // State measurement standard deviations. X, Y, theta.
      new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01), // Local measurement standard deviations. Left encoder, right encoder, gyro.
      new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01), // Global measurement standard deviations. X, Y, and theta.
      0.001); // Seconds between each time `update()` is called

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
    Rotation2d gyroAngle = Rotation2d.fromDegrees(-ahrs.getAngle()); // Invert to be ccw positive
    DifferentialDriveWheelSpeeds wheelVelocities = getWheelSpeeds();
    double leftDistance = leftMain.getEncoder().getPosition();
    double rightDistance = rightMain.getEncoder().getPosition();

    poseEstimator.update(gyroAngle, wheelVelocities, leftDistance, rightDistance);

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
   * Gets the wheel speeds of the drivetrain.
   * @return The drivetrain wheel speeds as a DifferentialDriveWheelSpeeds object.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      leftMain.getEncoder().getVelocity(), 
      rightMain.getEncoder().getVelocity());
  }

  /**
   * Reset the robot's position to a given Pose2d.
   * @param position The position to reset the pose estimator to.
   */
  public void resetPosition(Pose2d position) {
    leftMain.getEncoder().setPosition(0);
    rightMain.getEncoder().setPosition(0);

    // https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/wpilibj/estimator/DifferentialDrivePoseEstimator.html#resetPosition(edu.wpi.first.wpilibj.geometry.Pose2d,edu.wpi.first.wpilibj.geometry.Rotation2d)
    poseEstimator.resetPosition(position, Rotation2d.fromDegrees(-ahrs.getAngle()));
  }

  /**
   * Zeros the robot's position.
   * This method zeros both the robot's translation *and* rotation.
   */
  public void resetPosition() {
    resetPosition(new Pose2d());
  }
}
