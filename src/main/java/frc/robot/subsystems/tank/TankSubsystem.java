package frc.robot.subsystems.tank;

import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import frc.robot.GRTSubsystem;
import frc.robot.brownout.PowerController;
import frc.robot.shuffleboard.GRTNetworkTableEntry;
import frc.robot.shuffleboard.GRTShuffleboardTab;

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

    private final AHRS ahrs;
    private final PoseEstimator poseEstimator;

    private final GRTShuffleboardTab shuffleboardTab;
    private final GRTNetworkTableEntry shuffleX;
    private final GRTNetworkTableEntry shuffleY;
    private final GRTNetworkTableEntry shuffleHeading;
    private final Field2d shuffleboardField;

    public static final double ENCODER_ROTATIONS_TO_METERS = 4 / 71.11351407691836;

    public TankSubsystem() {
        // TODO: measure this
        super(40, fRightMotorPort, fLeftMotorPort, mRightMotorPort, mLeftMotorPort, bRightMotorPort, bLeftMotorPort);

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
        ahrs = new AHRS(SPI.Port.kMXP);

        // Start pose estimator thread
        poseEstimator = new PoseEstimator(ahrs, leftEncoder, rightEncoder);
        resetPosition();

        // Initialize Shuffleboard entries
        shuffleboardTab = new GRTShuffleboardTab("Drivetrain");
        shuffleX = shuffleboardTab.addEntry("Robot x", 0);
        shuffleY = shuffleboardTab.addEntry("Robot y", 0);
        shuffleHeading = shuffleboardTab.addEntry("Robot heading", 0);
        shuffleboardField = new Field2d();
        shuffleboardTab.addWidget("Field", shuffleboardField);
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
        poseEstimator.update();
        //System.out.println(getRobotPosition());

        // Update Shuffleboard entries
        Pose2d pose = getRobotPosition();
        
        shuffleX.setValue(pose.getX());
        shuffleY.setValue(pose.getY());
        shuffleHeading.setValue(pose.getRotation().getDegrees());
        shuffleboardField.setRobotPose(pose);
    }

    /**
     * Gets the estimated current position of the robot.
     * @return The estimated position of the robot as a Pose2d.
     */
    public Pose2d getRobotPosition() {
        return poseEstimator.getPosition();
    }

    /**
     * Gets the wheel speeds of the robot as a DifferentialDriveWheelSpeeds.
     * @return The wheel speeds of the robot as a DifferentialDriveWheelSpeeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        // TODO: is this ok?
        return poseEstimator.getLastWheelSpeeds();
    }

    /**
     * Reset the robot's position to a given Pose2d.
     * @param position The position to reset the pose estimator to.
     */
    public void resetPosition(Pose2d position) {
        poseEstimator.setPosition(position);
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
    public void setCurrentLimit(double limit) {
        int motorLimit = (int) Math.floor(limit / 4);

        leftMain.setSmartCurrentLimit(motorLimit);
        rightMain.setSmartCurrentLimit(motorLimit);
    }

    public double distance(Translation2d other) {
        Pose2d curr = getRobotPosition();

        return Math.hypot(curr.getX() - other.getX(), curr.getY() - other.getY());
    }
}
