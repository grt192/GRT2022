package frc.robot.subsystems.tank;

import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

public class PoseEstimatorThread {

    private final PoseEstimatorRunnable runnable;

    public PoseEstimatorThread(AHRS ahrs, RelativeEncoder leftEncoder, RelativeEncoder rightEncoder) {
        runnable = new PoseEstimatorRunnable(ahrs, leftEncoder, rightEncoder);

        Thread thread = new Thread(runnable);
        thread.setDaemon(true);
        thread.start();
    }

    /**
     * Gets the estimated current position of the robot.
     * @return The estimated position of the robot as a Pose2d.
     */
    public Pose2d getPosition() {
        return runnable.getPosition();
    }

    /**
     * Resets the robot's position to a given Pose2D.
     * @param pose The pose to reset the pose estimator to.
     */
    public void setPosition(Pose2d pose) {
        runnable.setPosition(pose);
    }

    /**
     * Gets the last measured wheel speeds of the drivetrain.
     * @return The last measured drivetrain wheel speeds as a DifferentialDriveWheelSpeeds object.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return runnable.getLastWheelSpeeds();
    }

    class PoseEstimatorRunnable implements Runnable {
        private final DifferentialDrivePoseEstimator poseEstimator;

        private final AHRS ahrs;
        private final RelativeEncoder leftEncoder;
        private final RelativeEncoder rightEncoder;

        private DifferentialDriveWheelSpeeds lastWheelSpeeds;

        public PoseEstimatorRunnable(AHRS ahrs, RelativeEncoder leftEncoder, RelativeEncoder rightEncoder) {
            this.ahrs = ahrs;
            this.leftEncoder = leftEncoder;
            this.rightEncoder = rightEncoder;

            // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose_state-estimators.html
            poseEstimator = new DifferentialDrivePoseEstimator(new Rotation2d(), new Pose2d(),
                // State measurement standard deviations. X, Y, theta, dist_l, dist_r.
                new MatBuilder<>(Nat.N5(), Nat.N1()).fill(0.02, 0.02, 0.01, 0.02, 0.02),
                // Odometry measurement standard deviations. Left encoder, right encoder, gyro.
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01),
                // Vision measurement standard deviations. X, Y, and theta.
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01));
        }

        @Override
        public void run() {
            while (true) {
                // Update odometry readings
                Rotation2d gyroAngle = getGyroHeading();
                lastWheelSpeeds = getWheelSpeeds();
                double leftDistance = leftEncoder.getPosition();
                double rightDistance = rightEncoder.getPosition();

                poseEstimator.update(gyroAngle, lastWheelSpeeds, leftDistance, rightDistance);

                System.out.println(getPosition());

                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                }
            }
        }

        /**
         * Gets the estimated current position of the robot.
         * @return The estimated position of the robot as a Pose2d.
         */
        public Pose2d getPosition() {
            return poseEstimator.getEstimatedPosition();
        }

        /**
         * Resets the robot's position to a given Pose2D.
         * @param pose The pose to reset the pose estimator to.
         */
        public void setPosition(Pose2d pose) {
            leftEncoder.setPosition(0);
            rightEncoder.setPosition(0);

            // https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/wpilibj/estimator/DifferentialDrivePoseEstimator.html#resetPosition(edu.wpi.first.wpilibj.geometry.Pose2d,edu.wpi.first.wpilibj.geometry.Rotation2d)
            poseEstimator.resetPosition(pose, getGyroHeading());
        }

        /**
         * Gets the gyro angle given by the NavX AHRS, inverted to be counterclockwise positive. 
         * @return The robot heading as a Rotation2d.
         */
        private Rotation2d getGyroHeading() {
            return Rotation2d.fromDegrees(-ahrs.getAngle());
        }

        /**
         * Gets the wheel speeds of the drivetrain. 
         * @return The drivetrain wheel speeds as a DifferentialDriveWheelSpeeds object.
         */
        private DifferentialDriveWheelSpeeds getWheelSpeeds() {
            return new DifferentialDriveWheelSpeeds(
                leftEncoder.getVelocity(),
                rightEncoder.getVelocity());
        }

        /**
         * Gets the last measured wheel speeds of the drivetrain.
         * @return The last measured drivetrain wheel speeds as a DifferentialDriveWheelSpeeds object.
         */
        public DifferentialDriveWheelSpeeds getLastWheelSpeeds() {
            return this.lastWheelSpeeds;
        }
    }
}
