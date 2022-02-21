package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.GRTSubsystem;
import frc.robot.brownout.PowerController;
import frc.robot.jetson.JetsonConnection;
import frc.robot.shuffleboard.GRTNetworkTableEntry;
import frc.robot.subsystems.tank.TankSubsystem;

/**
 * A subsystem which controls the turret mechanism on the robot. 
 * Note that the act of shooting is handled by internals; this subsystem only handles aiming (calculations and shot readiness).
 */
public class TurretSubsystem extends GRTSubsystem {

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

    /**
     * Represents the state of a turret module (flywheel, turntable, hood).
     * If a module is in HIGH_TOLERANCE, it is completely ready to shoot.
     * If a module is in LOW_TOLERANCE, it is nearly ready and will become ready after a robot stop.
     * This state is for rejecting, which doesn't need a perfectly lined up shot but needs better than completely
     * unaligned. It is also for drivers to know when the turret is almost ready
     * If a module is UNALIGNED, it is not ready; it will require more than a second to get it to READY status. 
     * For turntable, this includes being in the blind spot.
     */
    public enum ModuleState {
        HIGH_TOLERANCE, LOW_TOLERANCE, UNALIGNED
    }

    private final TankSubsystem tankSubsystem;
    private final JetsonConnection jetson;

    private final CANSparkMax turntable;
    private final RelativeEncoder turntableEncoder;
    private final SparkMaxPIDController turntablePidController;
    private final WPI_TalonSRX hood;

    private final CANSparkMax flywheel;
    private final RelativeEncoder flywheelEncoder;
    private final SparkMaxPIDController flywheelPidController;

    // constants
    // Turntable position PID constants
    private static final double turntableFF = 0.0022;
    private static final double turntableP = 0;
    private static final double turntableI = 0;
    private static final double turntableD = 0;
    private static final double maxVel = 9000;
    private static final double maxAccel = 220;

    // Hood position PID constants
    private static final double hoodP = 0.125;
    private static final double hoodI = 0;
    private static final double hoodD = 0;

    // Flywheel velocity PID constants
    private static final double flywheelP = 0.125;
    private static final double flywheelI = 0;
    private static final double flywheelD = 0;

    // Desired state variables
    // TODO: measure these, add constants
    private double theta;
    private double r;
    private Pose2d previousPosition = new Pose2d();

    private double desiredFlywheelSpeed = 30.0; // TODO: units? RPM?
    private double desiredTurntableRadians = 0.0;
    private double desiredHoodAngle = 0.0; // TODO: radians or degrees?

    private TurretMode mode = TurretMode.SHOOTING;

    // Encoder constants
    private static final double ENCODER_ROTATIONS_TO_RADIANS = (Math.PI / 2.) / 3.8095201253890992;
    private static final double TURNTABLE_MIN_POS = Math.toRadians(60);
    private static final double TURNTABLE_MAX_POS = Math.toRadians(300);

    private static final double HOOD_MAX_POS = 50.0;
    private static final double HOOD_MIN_POS = 0.0;

    // shuffleboard
    private final ShuffleboardTab shuffleboardTab;
    // private final NetworkTableEntry shuffleboardTurntablePEntry;
    // private final NetworkTableEntry shuffleboardTurntableIEntry;
    // private final NetworkTableEntry shuffleboardTurntableDEntry;
    // private final NetworkTableEntry shuffleboardTurntableFFEntry;
    // private final NetworkTableEntry shuffleboardTurntableMinVelEntry;
    // private final NetworkTableEntry shuffleboardTurntableMaxVelEntry;
    // private final NetworkTableEntry shuffleboardTurntableMaxAccelEntry;
    // private final NetworkTableEntry shuffleboardTarget;
    // private final NetworkTableEntry shuffleboardTurretVelo;
    private final NetworkTableEntry shuffleboardTurret;

    // private final NetworkTableEntry shuffleboardHoodPEntry;
    // private final NetworkTableEntry shuffleboardHoodIEntry;
    // private final NetworkTableEntry shuffleboardHoodDEntry;

    // private final NetworkTableEntry shuffleboardFlywheelPEntry;
    // private final NetworkTableEntry shuffleboardFlywheelIEntry;
    // private final NetworkTableEntry shuffleboardFlywheelDEntry;

    public TurretSubsystem(TankSubsystem tankSubsystem, JetsonConnection connection) {
        // TODO: measure this
        super(50);

        this.tankSubsystem = tankSubsystem;
        this.jetson = connection;

        // Initialize turntable SparkMax and encoder PID
        turntable = new CANSparkMax(turntablePort, MotorType.kBrushless);
        turntable.restoreFactoryDefaults();
        turntable.setIdleMode(IdleMode.kBrake);
        turntable.setInverted(false);
 
        // Position conversion: Rotations -> radians
        // Velocity conversion: RPM -> radians / minute
        turntableEncoder = turntable.getEncoder();
        turntableEncoder.setPositionConversionFactor((Math.PI/2.) / 3.8095201253890992);
        turntableEncoder.setVelocityConversionFactor((Math.PI/2.) / 3.8095201253890992);
        turntableEncoder.setPosition(Math.toRadians(180));
 
        turntablePidController = turntable.getPIDController();
        turntablePidController.setP(turntableP);
        turntablePidController.setI(turntableI);
        turntablePidController.setD(turntableD);
        turntablePidController.setIZone(0);
        turntablePidController.setFF(turntableFF);
        turntablePidController.setOutputRange(-0.5, 0.5);
        turntablePidController.setSmartMotionMaxVelocity(maxVel, 0);
        turntablePidController.setSmartMotionMaxAccel(maxAccel, 0);

        turntable.setSoftLimit(com.revrobotics.CANSparkMax.SoftLimitDirection.kForward, (float) TURNTABLE_MAX_POS);
        turntable.setSoftLimit(com.revrobotics.CANSparkMax.SoftLimitDirection.kReverse, (float) TURNTABLE_MIN_POS);
        turntable.enableSoftLimit(SoftLimitDirection.kForward, true);
        turntable.enableSoftLimit(SoftLimitDirection.kReverse, true);

        // Initialize hood 775 and encoder PID
        hood = new WPI_TalonSRX(hoodPort);
        hood.configFactoryDefault();
        hood.setNeutralMode(NeutralMode.Brake);

        hood.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        hood.setSelectedSensorPosition(0);
        hood.setSensorPhase(false);
        hood.config_kP(0, hoodP);
        hood.config_kI(0, hoodI);
        hood.config_kD(0, hoodD);

        /*
        hood.configForwardSoftLimitEnable(true);
        hood.configReverseSoftLimitEnable(true);
        hood.configForwardSoftLimitThreshold(HOOD_MAX_POS);
        hood.configReverseSoftLimitThreshold(HOOD_MIN_POS);
        */

        // Initialize flywheel NEO and encoder PID
        flywheel = new CANSparkMax(flywheelPort, MotorType.kBrushless);
        flywheel.restoreFactoryDefaults();
        //flywheel.setIdleMode(IdleMode.kBrake);
        flywheel.setInverted(false);

        flywheelEncoder = flywheel.getEncoder();
        flywheelEncoder.setVelocityConversionFactor(FLYWHEEL_RATIO);
        flywheelEncoder.setPosition(0);

        flywheelPidController = flywheel.getPIDController();
        flywheelPidController.setP(flywheelP);
        flywheelPidController.setI(flywheelI);
        flywheelPidController.setD(flywheelD);

        // Initialize Shuffleboard entries
        shuffleboardTab = Shuffleboard.getTab("Turret");
        shuffleboardTurret = shuffleboardTab.add("turret pos", 0).getEntry();
        // shuffleboardTurntablePEntry = shuffleboardTab.add("Turntable kP", 0).getEntry();
        // shuffleboardTurntableIEntry = shuffleboardTab.add("Turntable kI", 0).getEntry();
        // shuffleboardTurntableDEntry = shuffleboardTab.add("Turntable kD", 0).getEntry();
        // shuffleboardTurntableFFEntry = shuffleboardTab.add("Turntable FF", .002).getEntry();
        // shuffleboardTarget = shuffleboardTab.add("target pos", 90).getEntry();
        // shuffleboardTurretVelo = shuffleboardTab.add("turret velo", 0).getEntry();
        
        // shuffleboardTurntableMinVelEntry = shuffleboardTab.add("minvel", 0).getEntry();
        // shuffleboardTurntableMaxVelEntry = shuffleboardTab.add("maxvel", 10000).getEntry();
        // shuffleboardTurntableMaxAccelEntry = shuffleboardTab.add("maxaccel", 400).getEntry();

        // shuffleboardHoodPEntry = shuffleboardTab.add("Hood kP", hoodP).getEntry();
        // shuffleboardHoodIEntry = shuffleboardTab.add("Hood kI", hoodI).getEntry();
        // shuffleboardHoodDEntry = shuffleboardTab.add("Hood kD", hoodD).getEntry();

        // shuffleboardFlywheelPEntry = shuffleboardTab.add("Flywheel kP", flywheelP).getEntry();
        // shuffleboardFlywheelIEntry = shuffleboardTab.add("Flywheel kI", flywheelI).getEntry();
        // shuffleboardFlywheelDEntry = shuffleboardTab.add("Flywheel kD", flywheelD).getEntry();
    }

    @Override
    public void periodic() {
        // Get PID constants from Shuffleboard for testing
        // turntablePidController.setP(shuffleboardTurntablePEntry.getDouble(turntableP));
        // turntablePidController.setI(shuffleboardTurntableIEntry.getDouble(turntableI));
        // turntablePidController.setD(shuffleboardTurntableDEntry.getDouble(turntableD));
        // turntablePidController.setFF(shuffleboardTurntableFFEntry.getDouble(0));
        // turntablePidController.setSmartMotionMinOutputVelocity(shuffleboardTurntableMinVelEntry.getDouble(0), 0);
        // turntablePidController.setSmartMotionMaxVelocity(shuffleboardTurntableMaxVelEntry.getDouble(0), 0);
        // turntablePidController.setSmartMotionMaxAccel(shuffleboardTurntableMaxAccelEntry.getDouble(0), 0);

        // hood.config_kP(0, shuffleboardHoodPEntry.getDouble(hoodP));
        // hood.config_kI(0, shuffleboardHoodIEntry.getDouble(hoodI));
        // hood.config_kD(0, shuffleboardHoodDEntry.getDouble(hoodD));

        // flywheelPidController.setP(shuffleboardFlywheelPEntry.getDouble(flywheelP));
        // flywheelPidController.setI(shuffleboardFlywheelIEntry.getDouble(flywheelI));
        // flywheelPidController.setD(shuffleboardFlywheelDEntry.getDouble(flywheelD));

        // If retracted, skip jetson logic and calculations
        if (mode == TurretMode.RETRACTED) {
            desiredTurntableRadians = 0;
            desiredHoodAngle = 0;
            desiredFlywheelSpeed = 0;
        } else {
            Pose2d currentPosition = tankSubsystem.getRobotPosition();

            if (jetson != null) {
                // If the hub is in vision range, use vision's `r` and `theta` as ground truth
                if (jetson.turretVisionWorking()) {
                    r = jetson.getHubDistance();
                    theta = jetson.getTurretTheta();
                } else {
                    // Otherwise, update our `r` and `theta` state system from the previous `r` and `theta` values
                    // and the delta X and Y since our last position. We do this instead of using raw odometry coordinates
                    // to prevent error accumulation over time; `r` and `theta` are reset to vision values when vision is in
                    // range, so our states only accumulate error while vision is out of range (as opposed to odometry, which
                    // accumulates error throughout the match).

                    double deltaX = currentPosition.getX() - previousPosition.getX();
                    double deltaY = currentPosition.getY() - previousPosition.getY();
                    double deltaThetaRadians = currentPosition.getRotation().getRadians() - previousPosition.getRotation().getRadians();

                    double thetaRadians = Units.degreesToRadians(theta);
                    double thetaComplementRadians = Units.degreesToRadians(90 - theta);

                    // Localize deltas to Y-axis position
                    double localizedDeltaX = deltaX * Math.sin(thetaRadians) + deltaY * Math.sin(thetaComplementRadians);
                    double localizedDeltaY = deltaX * Math.cos(thetaRadians) + deltaY * Math.cos(thetaComplementRadians);

                    // Pythagorean theorem to calculate R'
                    double rPrime = Math.sqrt(Math.pow(r + localizedDeltaY, 2) + Math.pow(localizedDeltaX, 2));

                    // Calculate theta' from theta, delta theta, and phi (the change in turret angle between P and P', given
                    // by arccos(R / R'))
                    double phi = Math.acos(r / rPrime);
                    r = rPrime;
                    theta += Units.radiansToDegrees(deltaThetaRadians + phi);
                }

                // Set the turntable position from the relative theta given by vision
                desiredTurntableRadians = turntableEncoder.getPosition() + theta;

                // TODO: constants, interpolation
                desiredFlywheelSpeed = 30;

                previousPosition = currentPosition;
            }

            // Turntable position: heading + theta to maintain desired angle
            desiredTurntableRadians = Math.toRadians(shuffleboardTarget.getDouble(0)) - currentPosition.getRotation().getRadians();

            // If rejecting, scale down flywheel speed
            if (mode == TurretMode.REJECTING) desiredFlywheelSpeed *= 0.5;
        }

        // flywheelPidController.setReference(desiredFlywheelSpeed, ControlType.kVelocity);
        // hood.set(ControlMode.Position, desiredHoodAngle);
        // turntable.set(ControlMode.Position, Math.max(Math.min(desiredTurntablePosition, TURNTABLE_MAX_POS), TURNTABLE_MIN_POS));

        // System.out.println("Turret RPM: " + flywheelEncoder.getVelocity());
        shuffleboardTurret.setDouble(Math.toDegrees(turntableEncoder.getPosition()));
        // shuffleboardTurretVelo.setDouble(Math.toDegrees(turntableEncoder.getVelocity()));
        turntablePidController.setReference(desiredTurntableRadians, ControlType.kSmartMotion);
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
     * Gets the state of the flywheel (whether it is up to speed).
     * @return The state of the flywheel.
     */
    private ModuleState flywheelReady() {
        // TODO: test thresholding values
        double diff = Math.abs(flywheelEncoder.getVelocity() - desiredFlywheelSpeed);
        return diff < 10 ? ModuleState.HIGH_TOLERANCE
            : diff < 20 ? ModuleState.LOW_TOLERANCE
            : ModuleState.UNALIGNED;
    }

    /**
     * Gets the state of the turntable (whether it is aligned to the hub).
     * @return The state of the turntable.
     */
    private ModuleState turntableAligned() {
        // If the calculated theta is in the blind spot, return UNALIGNED
        if (desiredTurntableRadians < TURNTABLE_MIN_POS || desiredTurntableRadians > TURNTABLE_MAX_POS)
            return ModuleState.UNALIGNED;

        // TODO: test thresholding values
        double diff = Math.abs(turntableEncoder.getPosition() - desiredTurntableRadians);
        return diff < 10 ? ModuleState.HIGH_TOLERANCE
            : diff < 20 ? ModuleState.LOW_TOLERANCE
            : ModuleState.UNALIGNED;
    }

    /**
     * Gets the state of the hood (whether it is at its desired angle).
     * @return The state of the hood.
     */
    private ModuleState hoodReady() {
        // TODO: test thresholding values
        double diff = Math.abs(hood.getSelectedSensorPosition() - desiredHoodAngle);
        return diff < 5 ? ModuleState.HIGH_TOLERANCE
            : diff < 10 ? ModuleState.LOW_TOLERANCE
            : ModuleState.UNALIGNED;
    }

    /**
     * Gets the current state of the turret by taking the lowest state of all of its modules.
     * Ex: UNALIGNED, LOW_TOLERANCE, HIGH_TOLERANCE -> UNALIGNED
     * Ex. LOW_TOLERANCE, LOW_TOLERANCE, HIGH_TOLERANCE -> LOW_TOLERANCE
     * @return The state of the turret.
     */
    public ModuleState getState() {
        ModuleState flywheelState = flywheelReady();
        ModuleState turntableState = turntableAligned();
        ModuleState hoodState = hoodReady();

        // TODO: is there a better way to implement this?
        return flywheelState == ModuleState.UNALIGNED || turntableState == ModuleState.UNALIGNED || hoodState == ModuleState.UNALIGNED ? ModuleState.UNALIGNED
            : flywheelState == ModuleState.LOW_TOLERANCE || turntableState == ModuleState.LOW_TOLERANCE || hoodState == ModuleState.LOW_TOLERANCE ? ModuleState.LOW_TOLERANCE
            : ModuleState.HIGH_TOLERANCE;
    }

    /**
     * Cleans up the subsystem for climb (sets the Turret's mode to RETRACTED).
     * In RETRACTED, the turntable faces the same direction as the robot, the hood is retracted, and the flywheel is off.
     */
    @Override
    public void climbInit() {
        mode = TurretMode.RETRACTED;
    }

    @Override
    public double getTotalCurrentDrawn() {
        return PowerController.getCurrentDrawnFromPDH(turntablePort, hoodPort, flywheelPort);
    }

    @Override
    public void setCurrentLimit(double limit) {
        int motorLimit = (int) Math.floor(limit / 3);

        turntable.setSmartCurrentLimit(motorLimit);
        hood.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, motorLimit, 0, 0));
        flywheel.setSmartCurrentLimit(motorLimit);
    }

    /**
     * Test function to run the turntable at a set power.
     * @param pow The power to run the turntable at.
     */
    public void setTurntablePower(double pow) {
        turntable.set(pow);
    }

    /**
     * Test function to run the hood at a set power.
     * @param pow The power to run the hood at.
     */
    public void setHoodPower(double pow) {
        hood.set(pow);
    }

    /**
     * Test function to run the flywheel at a set power.
     * @param pow The power to run the flywheel at.
     */
    public void setFlywheelPower(double pow) {
        flywheel.set(pow);
    }
}
