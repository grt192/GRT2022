package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.jetson.JetsonConnection;
import static frc.robot.Constants.TurretConstants.*;

/**
 * A subsystem which controls the turret mechanism on the robot. 
 * Note that the act of shooting is handled by internals; this subsystem only handles aiming (calculations and shot readiness).
 */
public class TurretSubsystem extends SubsystemBase {
    private final JetsonConnection jetson;

    private final WPI_TalonSRX turntable;

    private final CANSparkMax hood;
    private final RelativeEncoder hoodEncoder;
    private final SparkMaxPIDController hoodPidController;

    private final CANSparkMax flywheel;
    private final RelativeEncoder flywheelEncoder;
    private final SparkMaxPIDController flywheelPidController;

    // Turntable position PID constants
    private static final double turntableP = 0.125;
    private static final double turntableI = 0;
    private static final double turntableD = 0;

    // Hood position PID constants
    private static final double hoodP = 0.125;
    private static final double hoodI = 0;
    private static final double hoodD = 0;

    // Flywheel velocity PID constants
    private static final double flywheelP = 0.125;
    private static final double flywheelI = 0;
    private static final double flywheelD = 0;

    // State variables
    // TODO: measure these, add constants
    private double flywheelSpeed = 30.0;
    private double turntablePosition = 0.0;
    private double hoodAngle = 60.0;

    private TurretMode mode = TurretMode.SHOOTING;

    private final ShuffleboardTab shuffleboardTab;
    private final NetworkTableEntry shuffleboardTurntablePEntry;
    private final NetworkTableEntry shuffleboardTurntableIEntry;
    private final NetworkTableEntry shuffleboardTurntableDEntry;

    private final NetworkTableEntry shuffleboardHoodPEntry;
    private final NetworkTableEntry shuffleboardHoodIEntry;
    private final NetworkTableEntry shuffleboardHoodDEntry;

    private final NetworkTableEntry shuffleboardFlywheelPEntry;
    private final NetworkTableEntry shuffleboardFlywheelIEntry;
    private final NetworkTableEntry shuffleboardFlywheelDEntry;

    public TurretSubsystem(JetsonConnection connection) {
        // Initialize turntable Talon and encoder PID
        turntable = new WPI_TalonSRX(turntablePort);
        turntable.configFactoryDefault();
        turntable.setNeutralMode(NeutralMode.Brake);

        turntable.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        turntable.setSelectedSensorPosition(0);
        turntable.setSensorPhase(false);
        turntable.config_kP(0, turntableP);
        turntable.config_kI(0, turntableI);
        turntable.config_kD(0, turntableD);

        // Initialize hood SparkMax and encoder PID
        hood = new CANSparkMax(hoodPort, MotorType.kBrushless);
        hood.restoreFactoryDefaults();
        hood.setIdleMode(IdleMode.kBrake);

        hoodEncoder = hood.getEncoder();
        hoodEncoder.setPosition(0);

        hoodPidController = hood.getPIDController();
        hoodPidController.setP(hoodP);
        hoodPidController.setI(hoodI);
        hoodPidController.setD(hoodD);

        // Initialize flywheel SparkMax and encoder PID
        flywheel = new CANSparkMax(flywheelPort, MotorType.kBrushless);
        flywheel.restoreFactoryDefaults();
        //flywheel.setIdleMode(IdleMode.kBrake);
        flywheel.setInverted(true);

        flywheelEncoder = flywheel.getEncoder();
        flywheelEncoder.setPosition(0);

        flywheelPidController = flywheel.getPIDController();
        flywheelPidController.setP(flywheelP);
        flywheelPidController.setI(flywheelI);
        flywheelPidController.setD(flywheelD);

        this.jetson = connection;

        // Initialize Shuffleboard entries
        shuffleboardTab = Shuffleboard.getTab("Shooter");
        shuffleboardTurntablePEntry = shuffleboardTab.add("Turntable kP", turntableP).getEntry();
        shuffleboardTurntableIEntry = shuffleboardTab.add("Turntable kI", turntableI).getEntry();
        shuffleboardTurntableDEntry = shuffleboardTab.add("Turntable kD", turntableD).getEntry();

        shuffleboardHoodPEntry = shuffleboardTab.add("Hood kP", hoodP).getEntry();
        shuffleboardHoodIEntry = shuffleboardTab.add("Hood kI", hoodI).getEntry();
        shuffleboardHoodDEntry = shuffleboardTab.add("Hood kD", hoodD).getEntry();

        shuffleboardFlywheelPEntry = shuffleboardTab.add("Flywheel kP", flywheelP).getEntry();
        shuffleboardFlywheelIEntry = shuffleboardTab.add("Flywheel kI", flywheelI).getEntry();
        shuffleboardFlywheelDEntry = shuffleboardTab.add("Flywheel kD", flywheelD).getEntry();
    }

    @Override
    public void periodic() {
        // Get PID constants from Shuffleboard for testing
        turntable.config_kP(0, shuffleboardTurntablePEntry.getDouble(turntableP));
        turntable.config_kI(0, shuffleboardTurntableIEntry.getDouble(turntableI));
        turntable.config_kD(0, shuffleboardTurntableDEntry.getDouble(turntableD));

        hoodPidController.setP(shuffleboardHoodPEntry.getDouble(hoodP));
        hoodPidController.setI(shuffleboardHoodIEntry.getDouble(hoodI));
        hoodPidController.setD(shuffleboardHoodDEntry.getDouble(hoodD));

        flywheelPidController.setP(shuffleboardFlywheelPEntry.getDouble(flywheelP));
        flywheelPidController.setI(shuffleboardFlywheelIEntry.getDouble(flywheelI));
        flywheelPidController.setD(shuffleboardFlywheelDEntry.getDouble(flywheelD));

        // If retracted, skip jetson logic and calculations
        if (mode == TurretMode.RETRACTED) {
            turntablePosition = 0;
            hoodAngle = 0;
            flywheelSpeed = 0;
        } else {
            // TODO: implement vision tracking and turntable
            turntablePosition = jetson.getTurretTheta();

            double distance = jetson.getHubDistance();
            // TODO: constants, interpolation
            flywheelSpeed = 30;

            // If rejecting, scale down flywheel speed
            if (mode == TurretMode.REJECTING) flywheelSpeed *= 0.5;
        }

        flywheelPidController.setReference(flywheelSpeed, ControlType.kVelocity);
        hoodPidController.setReference(hoodAngle, ControlType.kPosition);
        turntable.set(ControlMode.Position, turntablePosition);
    }

    /**
     * Set whether the turret should reject the current ball.
     * @param reject Whether to reject the ball.
     */
    public void setReject(boolean reject) {
        // Don't do anything if the turret is retracted
        // TODO: is this necessary? will internals still be executing periodic logic during climb?
        if (mode == TurretMode.RETRACTED) return;
        mode = reject ? TurretMode.REJECTING : TurretMode.SHOOTING;
    }

    /**
     * Checks whether the flywheel is ready to shoot.
     * @return Whether the flywheel is up to speed.
     */
    public boolean flywheelReady() {
        // TODO: test thresholding value
        return Math.abs(flywheelEncoder.getVelocity() - flywheelSpeed) < 10;
    }

    /**
     * Checks whether the turntable is aligned to the hub.
     * @return Whether the turntable is aligned and ready to shoot.
     */
    public boolean turntableAligned() {
        // TODO: test thresholding value
        return Math.abs(turntable.getSelectedSensorPosition() - turntablePosition) < 10;
    }

    /**
     * Checks whether the hood is at its desired angle.
     * @return Whether the hood is in position.
     */
    public boolean hoodReady() {
        // TODO: test thresholding value
        return Math.abs(hoodEncoder.getPosition() - hoodAngle) < 10;
    }

    /**
     * Cleans up the subsystem for climb (sets the Turret's mode to RETRACTED).
     * In RETRACTED, the turntable faces the same direction as the robot, the hood is retracted, and the flywheel is off.
     */
    public void climbInit() {
        mode = TurretMode.RETRACTED;
    }

    /**
     * An enum representing the mode the shooter is currently in.
     * In SHOOTING, the turret will aim to score balls in the upper hub.
     * In REJECTING, the turret will scale down its flywheel speed to reject wrong-colored balls.
     * In RETRACTED, the turret will retract itself for climb.
     * TODO: split SHOOTING into UPPER and LOWER modes?
     */
    public enum TurretMode {
        SHOOTING, REJECTING, RETRACTED
    }
}
