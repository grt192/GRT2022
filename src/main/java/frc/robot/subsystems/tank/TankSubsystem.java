package frc.robot.subsystems.tank;

import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import frc.robot.GRTSubsystem;
import frc.robot.brownout.PowerController;

import static frc.robot.Constants.TankConstants.*;

/**
 * A subsystem which controls the robot's drivetrain. This subsystem handles both driving and odometry.
 */
public class TankSubsystem extends GRTSubsystem {
    private final CANSparkMax leftMain;
    private final CANSparkMax leftMiddle;
    private final CANSparkMax leftBack;

    private final CANSparkMax rightMain;
    private final CANSparkMax rightMiddle;
    private final CANSparkMax rightBack;

    private final PoseEstimatorThread poseEstimatorThread;

    private final ShuffleboardTab shuffleboardTab;
    private final NetworkTableEntry shuffleboardXEntry;
    private final NetworkTableEntry shuffleboardYEntry;
    private final Field2d shuffleboardField;

    // TODO: measure this for new robot
    public static final double ENCODER_ROTATIONS_TO_METERS = 5 / 92.08;

    public TankSubsystem() {
        // TODO: measure this
        super(50);

        // Init left main and follower motors and encoders
        leftMain = new CANSparkMax(fLeftMotorPort, MotorType.kBrushless);
        leftMain.restoreFactoryDefaults();
        leftMain.setIdleMode(IdleMode.kBrake);

        // Position conversion: Rotations -> m
        // Velocity conversion: RPM -> m/s
        RelativeEncoder leftEncoder = leftMain.getEncoder();
        leftEncoder.setPositionConversionFactor(ENCODER_ROTATIONS_TO_METERS);
        leftEncoder.setVelocityConversionFactor(ENCODER_ROTATIONS_TO_METERS / 60.0);

        leftMiddle = new CANSparkMax(mLeftMotorPort, MotorType.kBrushless);
        leftMiddle.restoreFactoryDefaults();
        leftMiddle.follow(leftMain);
        leftMiddle.setIdleMode(IdleMode.kBrake);

        leftBack = new CANSparkMax(bLeftMotorPort, MotorType.kBrushless);
        leftBack.restoreFactoryDefaults();
        leftBack.follow(leftMain);
        leftBack.setIdleMode(IdleMode.kBrake);

        // Init right main and follower motors
        rightMain = new CANSparkMax(fRightMotorPort, MotorType.kBrushless);
        rightMain.restoreFactoryDefaults();
        rightMain.setInverted(true);
        rightMain.setIdleMode(IdleMode.kBrake);

        RelativeEncoder rightEncoder = rightMain.getEncoder();
        rightEncoder.setPositionConversionFactor(ENCODER_ROTATIONS_TO_METERS);
        rightEncoder.setVelocityConversionFactor(ENCODER_ROTATIONS_TO_METERS / 60.0);

        rightMiddle = new CANSparkMax(mRightMotorPort, MotorType.kBrushless);
        rightMiddle.restoreFactoryDefaults();
        rightMiddle.follow(rightMain);
        rightMiddle.setIdleMode(IdleMode.kBrake);

        rightBack = new CANSparkMax(bRightMotorPort, MotorType.kBrushless);
        rightBack.restoreFactoryDefaults();
        rightBack.follow(rightMain);
        rightBack.setIdleMode(IdleMode.kBrake);

        // Initialize navX AHRS
        // https://www.kauailabs.com/public_files/navx-mxp/apidocs/java/com/kauailabs/navx/frc/AHRS.html
        AHRS ahrs = new AHRS(SPI.Port.kMXP);

        // Start pose estimator thread
        poseEstimatorThread = new PoseEstimatorThread(ahrs, leftEncoder, rightEncoder);

        // Initialize Shuffleboard entries
        shuffleboardTab = Shuffleboard.getTab("Drivetrain");
        shuffleboardXEntry = shuffleboardTab.add("Robot x", 0).getEntry();
        shuffleboardYEntry = shuffleboardTab.add("Robot y", 0).getEntry();
        shuffleboardTab.add("Gyro", ahrs);
        shuffleboardField = new Field2d();
        shuffleboardTab.add("Field", shuffleboardField);
    }

    /**
     * Drive the system with the given power scales using the car system.
     * @param yScale Scale in the forward/backward direction, from 1 to -1.
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
     * @param leftScale Scale in the forward/backward direction of the left motor, from -1 to 1.
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
     * Drive the system with the given voltage values for each side of the drivetrain.
     * @param leftVoltage Left motor voltages.
     * @param rightVoltage Right motor voltages.
     */
    public void setTankDriveVoltages(double leftVoltage, double rightVoltage) {
        leftMain.setVoltage(leftVoltage);
        rightMain.setVoltage(rightVoltage);
    }

    @Override
    public void periodic() {
        // Update Shuffleboard entries
        Pose2d pose = getRobotPosition();

        shuffleboardXEntry.setDouble(pose.getX());
        shuffleboardYEntry.setDouble(pose.getY());
        shuffleboardField.setRobotPose(pose);
    }

    /**
     * Gets the estimated current position of the robot.
     * @return The estimated position of the robot as a Pose2d.
     */
    public Pose2d getRobotPosition() {
        return poseEstimatorThread.getPosition();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return poseEstimatorThread.getWheelSpeeds();
    }

    /**
     * Reset the robot's position to a given Pose2d.
     * @param position The position to reset the pose estimator to.
     */
    public void resetPosition(Pose2d position) {
        poseEstimatorThread.setPosition(position);
    }

    /**
     * Zeros the robot's position.
     * This method zeros both the robot's translation *and* rotation.
     */
    public void resetPosition() {
        resetPosition(new Pose2d());
    }

    /**
     * Squares an input value while retaining the original sign.
     * @param value The value to square.
     * @return The squared value.
     */
    private double squareInput(double value) {
        return Math.copySign(value * value, value);
    }

    @Override
    public double getTotalCurrentDrawn() {
        return PowerController.getCurrentDrawnFromPDH(fRightMotorPort, fLeftMotorPort, bRightMotorPort, bLeftMotorPort);
    }

    @Override
    public void setCurrentLimit(double limit) {
        int motorLimit = (int) Math.floor(limit / 4);

        leftMain.setSmartCurrentLimit(motorLimit);
        rightMain.setSmartCurrentLimit(motorLimit);
    }
}
