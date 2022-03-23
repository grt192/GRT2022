package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import frc.robot.GRTSubsystem;
import frc.robot.brownout.PowerController;
import frc.robot.jetson.JetsonConnection;
import frc.robot.shuffleboard.GRTNetworkTableEntry;
import frc.robot.subsystems.tank.TankSubsystem;

import static frc.robot.Constants.TurretConstants.*;

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

    // private final DigitalInput leftLimitSwitch;
    // private final DigitalInput rightLimitSwitch;

    // Turntable position PID constants
    private static final double turntableP = 0.00015;
    private static final double turntableI = 0;
    private static final double turntableD = 0;
    private static final double turntableFF = 0.0009;
    private static final double maxVel = 150;
    private static final double maxAccel = 300;

    // Hood position PID constants
    private static final double hoodP = 0.125;
    private static final double hoodI = 0;
    private static final double hoodD = 0;

    // Flywheel velocity PID constants
    private static final double flywheelP = 0.00015;
    private static final double flywheelI = 0;
    private static final double flywheelD = 0;
    private static final double flywheelFF = 0.000092;

    // Reference variables for MANUAL_CONTROL tuning
    private double turntableRefPos = 180.0;
    private double flywheelRefVel = 0;
    private double hoodRefPos = 0;

    // Temporary offset states for manual correction of rtheta drift
    private double turntableOffset = 0;
    private double distanceOffset = 0;

    // The interpolation table from shooter testing.
    // Every entry can be thought of as a tuple representing [hub distance (in), flywheel speed (RPM), hood angle (degs)];
    // for any given hub distance, the desired flywheel speed and hood angle can be linearly interpolated 
    // from the immediately higher and lower points.
    // IMPORTANT: entries are assumed to be in order by hub distance. Without this assumption the array would have to be 
    // sorted before interpolation.
    private static final double[][] INTERPOLATION_TABLE = {
        {69, 5000, 7},
        {88, 5100, 13.5},
        {112, 5300, 17},
        {139, 5600, 20},
        {175, 6000, 25},
        {202, 6400, 30},
        {221, 6800, 36}
    };

    // System state variables
    private double theta;
    private double r;
    private Pose2d previousPosition;

    // Motor state variables
    private double desiredFlywheelRPM;
    private double desiredTurntableRadians;
    private double desiredHoodRadians;

    private boolean ballReady = false;

    private TurretMode mode = TurretMode.RETRACTED;

    private static final double TURNTABLE_ROTATIONS_TO_RADIANS = (Math.PI / 2.) / 3.142854928970337;
    private static final double TURNTABLE_MIN_RADIANS = Math.toRadians(47);
    private static final double TURNTABLE_MAX_RADIANS = Math.toRadians(320);
    private double TURNTABLE_THETA_FF = 4; // TODO: tune

    private static final double HOOD_RADIANS_TO_TICKS = 243732.0 / Math.toRadians(35.8029900116);
    public static final double HOOD_MIN_POS = 0.0;
    public static final double HOOD_MAX_POS = Math.toRadians(36) * HOOD_RADIANS_TO_TICKS; // 243758

    private static final double FLYWHEEL_GEAR_RATIO = 36.0 / 18.0;

    // Shuffleboard
    private final ShuffleboardTab shuffleboardTab;
    private final GRTNetworkTableEntry shuffleboardTurntablePosEntry;
    private GRTNetworkTableEntry shuffleboardFlywheelVeloEntry;
    private GRTNetworkTableEntry shuffleboardHoodPosEntry;
    private final GRTNetworkTableEntry shuffleboardOffset;
    private final GRTNetworkTableEntry rEntry;
    private final GRTNetworkTableEntry thetaEntry;

    // Debug flags
    // Whether interpolation (`r`, hood ref, flywheel ref) and rtheta (`r`, `theta`, `dx`, `dy`, `dtheta`, 
    // `alpha`, `beta`, `x`, `y`, `h`) system states should be printed.
    private static boolean PRINT_STATES = false; 
    // Whether PID tuning shuffleboard entries should be displayed.
    private static boolean DEBUG_PID = false;
    // Whether the turntable, hood, and flywheel references should be manually set through shuffleboard.
    private static boolean MANUAL_CONTROL = false;
    // Whether the turret should fire at full speed regardless of rejection logic.
    private static boolean SKIP_REJECTION = true;

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
        turntableEncoder.setPositionConversionFactor(TURNTABLE_ROTATIONS_TO_RADIANS);
        turntableEncoder.setVelocityConversionFactor(TURNTABLE_ROTATIONS_TO_RADIANS);
        turntableEncoder.setPosition(Math.toRadians(180));

        turntablePidController = turntable.getPIDController();
        turntablePidController.setP(turntableP);
        turntablePidController.setI(turntableI);
        turntablePidController.setD(turntableD);
        turntablePidController.setFF(turntableFF);
        turntablePidController.setIZone(0);
        turntablePidController.setOutputRange(-0.5, 0.5);
        turntablePidController.setSmartMotionMaxVelocity(maxVel, 0);
        turntablePidController.setSmartMotionMaxAccel(maxAccel, 0);

        turntable.setSoftLimit(SoftLimitDirection.kForward, (float) TURNTABLE_MAX_RADIANS);
        turntable.setSoftLimit(SoftLimitDirection.kReverse, (float) TURNTABLE_MIN_RADIANS);
        turntable.enableSoftLimit(SoftLimitDirection.kForward, true);
        turntable.enableSoftLimit(SoftLimitDirection.kReverse, true);

        // Initialize hood 775 and encoder PID
        hood = new WPI_TalonSRX(hoodPort);
        hood.configFactoryDefault();
        hood.setInverted(true);
        hood.setNeutralMode(NeutralMode.Brake);

        hood.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        hood.setSelectedSensorPosition(0);
        hood.setSensorPhase(true);
        hood.config_kP(0, hoodP);
        hood.config_kI(0, hoodI);
        hood.config_kD(0, hoodD);

        hood.configForwardSoftLimitEnable(true);
        hood.configReverseSoftLimitEnable(true);
        hood.configForwardSoftLimitThreshold(HOOD_MAX_POS);
        hood.configReverseSoftLimitThreshold(HOOD_MIN_POS);

        // Initialize flywheel NEO and encoder PID
        flywheel = new CANSparkMax(flywheelPort, MotorType.kBrushless);
        flywheel.restoreFactoryDefaults();
        flywheel.setInverted(false);

        // Position conversion: Motor rotations -> flywheel rotations
        // Velocity conversion: Motor RPM -> flywheel RPM
        flywheelEncoder = flywheel.getEncoder();
        flywheelEncoder.setPositionConversionFactor(FLYWHEEL_GEAR_RATIO);
        flywheelEncoder.setVelocityConversionFactor(FLYWHEEL_GEAR_RATIO);
        flywheelEncoder.setPosition(0);

        flywheelPidController = flywheel.getPIDController();
        flywheelPidController.setP(flywheelP);
        flywheelPidController.setI(flywheelI);
        flywheelPidController.setD(flywheelD);
        flywheelPidController.setFF(flywheelFF);

        // Initialize limit switches
        // leftLimitSwitch = new DigitalInput(lLimitSwitchPort);
        // rightLimitSwitch = new DigitalInput(rLimitSwitchPort);

        // Initialize Shuffleboard entries
        shuffleboardTab = Shuffleboard.getTab("Turret");
        shuffleboardTurntablePosEntry = new GRTNetworkTableEntry(shuffleboardTab.add("Turntable pos", Math.toDegrees(turntableEncoder.getPosition())).getEntry());
        rEntry = new GRTNetworkTableEntry(shuffleboardTab.add("r", 0).getEntry());
        thetaEntry = new GRTNetworkTableEntry(shuffleboardTab.add("theta", 0).getEntry());
        // shuffleboardTurntableVeloEntry = new GRTNetworkTableEntry(shuffleboardTab.add("Turntable vel", 0).getEntry());
        // shuffleboardFlywheelVeloEntry = new GRTNetworkTableEntry(shuffleboardTab.add("Flywheel vel", flywheelEncoder.getVelocity()).getEntry());
        // shuffleboardHoodPosEntry = new GRTNetworkTableEntry(shuffleboardTab.add("Hood pos", 0).getEntry());
        shuffleboardOffset = new GRTNetworkTableEntry(shuffleboardTab.add("Turntable Offset", 0).getEntry());

        // If DEBUG_PID is set, allow for PID tuning on shuffleboard
        if (DEBUG_PID) {
            shuffleboardTab.add("Turntable kP", turntableP).getEntry()
                .addListener(this::setTurntableP, EntryListenerFlags.kUpdate);
            shuffleboardTab.add("Turntable kI", turntableI).getEntry()
                .addListener(this::setTurntableI, EntryListenerFlags.kUpdate);
            shuffleboardTab.add("Turntable kD", turntableD).getEntry()
                .addListener(this::setTurntableD, EntryListenerFlags.kUpdate);
            shuffleboardTab.add("Turntable kFF", turntableFF).getEntry()
                .addListener(this::setTurntableFF, EntryListenerFlags.kUpdate);
            shuffleboardTab.add("Turntable maxVel", maxVel).getEntry()
                .addListener(this::setTurntableMaxVel, EntryListenerFlags.kUpdate);
            shuffleboardTab.add("Turntable minVel", maxAccel).getEntry()
                .addListener(this::setTurnMinVel, EntryListenerFlags.kUpdate);
            shuffleboardTab.add("Turntable maxAcc", maxAccel).getEntry()
                .addListener(this::setTurntableMaxAcc, EntryListenerFlags.kUpdate);
            shuffleboardTab.add("Turntable theta FF", TURNTABLE_THETA_FF).getEntry()
                .addListener(this::setTurntableThetaFF, EntryListenerFlags.kUpdate); 

            shuffleboardTab.add("Flywheel kP", flywheelP).getEntry()
                .addListener(this::setFlywheelP, EntryListenerFlags.kUpdate);
            shuffleboardTab.add("Flywheel kI", flywheelI).getEntry()
                .addListener(this::setFlywheelI, EntryListenerFlags.kUpdate);
            shuffleboardTab.add("Flywheel kD", flywheelD).getEntry()
                .addListener(this::setFlywheelD, EntryListenerFlags.kUpdate);
            shuffleboardTab.add("Flywheel kFF", flywheelFF).getEntry()
                .addListener(this::setFlywheelFF, EntryListenerFlags.kUpdate);

            shuffleboardTab.add("Hood kP", hoodP).getEntry()
                .addListener(this::setHoodP, EntryListenerFlags.kUpdate);
            shuffleboardTab.add("Hood kI", hoodI).getEntry()
                .addListener(this::setHoodI, EntryListenerFlags.kUpdate);
            shuffleboardTab.add("Hood kD", hoodD).getEntry()
                .addListener(this::setHoodD, EntryListenerFlags.kUpdate);
        }

        // If MANUAL_CONTROL is enabled, allow for reference setting on shuffleboard
        if (MANUAL_CONTROL) {
            shuffleboardTab.add("Turntable ref pos", Math.toDegrees(turntableEncoder.getPosition())).getEntry()
                .addListener(this::setTurntableRefPos, EntryListenerFlags.kUpdate);
            shuffleboardTab.add("Flywheel ref vel", flywheelRefVel).getEntry()
                .addListener(this::setFlywheelRefVel, EntryListenerFlags.kUpdate);
            shuffleboardTab.add("Hood ref degs", hoodRefPos).getEntry()
                .addListener(this::setHoodRefPos, EntryListenerFlags.kUpdate);
        }
    }

    @Override
    public void periodic() {
        shuffleboardTurntablePosEntry.setValue(Math.toDegrees(turntableEncoder.getPosition()));
        // shuffleboardFlywheelVeloEntry.setValue(flywheelEncoder.getVelocity());
        // shuffleboardHoodPosEntry.setValue(Math.toDegrees(hood.getSelectedSensorPosition() / HOOD_RADIANS_TO_TICKS));

        // Check limit switches and reset encoders if detected
        // if (leftLimitSwitch.get()) turntableEncoder.setPosition(TURNTABLE_MIN_RADIANS);
        // if (rightLimitSwitch.get()) turntableEncoder.setPosition(TURNTABLE_MAX_RADIANS);

        Pose2d currentPosition = tankSubsystem.getRobotPosition();
        // boolean runFlywheel = !tankSubsystem.isMoving() && ballReady;
        boolean runFlywheel = ballReady;

        // Set turntable lazy tracking if a ball isn't ready
        double pow = !ballReady ? 0.25 : 0.5;
        turntablePidController.setOutputRange(-pow, pow);

        // If the hub is in vision range, use vision's `r` and `theta` as ground truth.
        // While the flywheel is running, use the manual system values instead to prevent camera issues while the flywheel shakes
        // the turntable.
        if (jetson.turretVisionWorking() && !runFlywheel) {
            r = jetson.getHubDistance();
            theta = jetson.getTurretTheta();

            // Reset offsets to accomodate for rtheta system reset
            turntableOffset = 0;
            distanceOffset = 0;

            // System.out.println("JETSON r: " + r + " theta: " + theta);
        } else {
            // Otherwise, update our `r` and `theta` state system from the previous `r` and `theta` values
            // and the delta X and Y since our last position. We do this instead of using raw odometry coordinates
            // to prevent error accumulation over time; `r` and `theta` are reset to vision values when vision is in
            // range, so our states only accumulate error while vision is out of range (as opposed to odometry, which
            // accumulates error throughout the match).

            // TODO: this assumes that the last r, theta we got is always 'clean' data.
            // we need to do filtering of some sort, probably on the jetson, to ensure this is true.
            manualUpdateRTheta(previousPosition, currentPosition);
        }

        previousPosition = currentPosition;

        rEntry.setValue(this.r);
        thetaEntry.setValue(this.theta);

        // If retracted, skip interpolation calculations
        if (mode == TurretMode.RETRACTED) {
            desiredTurntableRadians = Math.toRadians(180);
            desiredHoodRadians = 0;
            desiredFlywheelRPM = 0;

            turntablePidController.setReference(desiredTurntableRadians, ControlType.kSmartMotion);
            hood.set(ControlMode.Position, desiredHoodRadians);
            flywheelPidController.setReference(desiredFlywheelRPM, ControlType.kVelocity);
        } else {
            // If MANUAL_CONTROL is enabled, set the turntable reference from shuffleboard.
            // Otherwise, use the theta given by `rtheta`.
            // TODO: threshold for wrapping to prevent excessive swing-between
            double newTurntableRadians = MANUAL_CONTROL 
                ? (Math.toRadians(turntableRefPos) - currentPosition.getRotation().getRadians()) % (2 * Math.PI)
                : angleWrap(Math.PI - theta + turntableOffset);

            // Apply feedforward and constrain within max and min angle
            double deltaTurntableRadians = newTurntableRadians - desiredTurntableRadians;
            double turntableReference = Math.min(Math.max(
                newTurntableRadians + deltaTurntableRadians * TURNTABLE_THETA_FF, 
                TURNTABLE_MIN_RADIANS), TURNTABLE_MAX_RADIANS);

            interpolateFlywheelHoodRefs();

            if (PRINT_STATES)
                System.out.println("r: " + r + ", hood ref: " + Math.toDegrees(desiredHoodRadians) + ", flywheel ref: " + desiredFlywheelRPM);

            turntablePidController.setReference(turntableReference, ControlType.kSmartMotion);

            // If MANUAL_CONTROL is enabled, set the references from shuffleboard
            if (MANUAL_CONTROL) {
                hood.set(ControlMode.Position, Math.toRadians(hoodRefPos) * HOOD_RADIANS_TO_TICKS);
                flywheelPidController.setReference(flywheelRefVel, ControlType.kVelocity);
            } else {
                // Otherwise, use the interpolated values from hub distance
                hood.set(ControlMode.Position, desiredHoodRadians * HOOD_RADIANS_TO_TICKS);
                flywheelPidController.setReference(runFlywheel ? desiredFlywheelRPM : 0, ControlType.kVelocity);
            }
            desiredTurntableRadians = newTurntableRadians;
        }
    }

    /**
     * Sets the initial `r` and `theta` of the subsystem from an initial pose.
     * @param initial The pose to calculate `r` and `theta` from.
     */
    public void setInitialPose(Pose2d initial) {
        double x = Units.metersToInches(initial.getX());
        double y = Units.metersToInches(initial.getY());

        double theta = initial.getRotation().getRadians();
        double phi = Math.atan2(y, x);

        this.r = Math.hypot(x, y);
        this.theta = theta - phi;
        this.previousPosition = initial;
    }

    /**
     * Updates the state of the turntable's `r` and `theta` coordinate system.
     * @param lastPosition The previous odometry position.
     * @param currentPosition The current odometry position.
     */
    private void manualUpdateRTheta(Pose2d lastPosition, Pose2d currentPosition) {
        Pose2d deltas = currentPosition.relativeTo(lastPosition);

        double dx = Units.metersToInches(deltas.getX());
        double dy = Units.metersToInches(deltas.getY());
        double dTheta = deltas.getRotation().getRadians();

        double h = Math.hypot(dx, dy);
        double alpha = Math.atan2(-dy, dx);
        double beta = alpha + this.theta;

        double x = r + h * Math.cos(beta);
        double y = h * Math.sin(beta);

        if (PRINT_STATES) System.out.println(
            "dx: " + dx + " dy: " + dy + " dtheta: " + dTheta
            + "\npose: " + currentPosition + " last: " + lastPosition 
            + "\nh: " + h + " alpha: " + alpha + " beta: " + beta
            + "\nx: " + x + " y: " + y
            + "\nr: " + r + ", theta: " + Math.toDegrees(theta)
        );

        this.r = Math.hypot(x, y);
        this.theta = (theta + dTheta) - Math.atan2(y, x);
    }

    /**
     * Interpolate the hood angle and flywheel RPM from the hub distance and interpolation
     * table. If rejecting, scale down flywheel speed to prevent scoring.
     */
    private void interpolateFlywheelHoodRefs() {
        double hubDistance = r + distanceOffset;

        for (int i = 1; i < INTERPOLATION_TABLE.length; i++) {
            double[] above = INTERPOLATION_TABLE[i];
            double[] below = INTERPOLATION_TABLE[i - 1];

            double rTop = above[0], flywheelTop = above[1], hoodAngleTop = above[2];
            double rBottom = below[0], flywheelBottom = below[1], hoodAngleBottom = below[2];

            // If the entry's distance is above the current distance, the current distance is between the entry 
            // and the previous entry.
            if (rTop > hubDistance) {
                // Where x axis is distance d, y axis is flywheel RPM f, the top and bottom points are A and B, 
                // and the interpolated point is C:
                // m = (f_A - f_B) / (d_A - d_B)
                // C = (d_B + (d_C - d_B), mx) = (d_C, md_C)
                double flywheelSlope = (flywheelTop - flywheelBottom) / (rTop - rBottom);
                desiredFlywheelRPM = flywheelBottom + flywheelSlope * (hubDistance - rBottom);
                if (mode == TurretMode.REJECTING && !SKIP_REJECTION) desiredFlywheelRPM *= 0.5;

                double hoodSlope = (hoodAngleTop - hoodAngleBottom) / (rTop - rBottom);
                desiredHoodRadians = Math.toRadians(hoodAngleBottom + hoodSlope * (hubDistance - rBottom));
                return;
            }
        }
    }

    /**
     * Set whether the turret should reject the current ball.
     * @param reject Whether to reject the ball.
     */
    public void setReject(boolean reject) {
        // Don't do anything if the turret is retracted
        if (mode == TurretMode.RETRACTED) return;
        mode = reject ? TurretMode.REJECTING : TurretMode.SHOOTING;
    }

    /**
     * Sets whether there is a ball in internals. A ready ball disengages lazy tracking and starts
     * powering the flywheel when the robot is stationary.
     * @param ballReady Whether there is a ball in internals.
     */
    public void setBallReady(boolean ballReady) {
        this.ballReady = ballReady;
    }

    /**
     * Changes the distance offset of the `rtheta` system.
     * @param offset The amount to change the offet by, in inches.
     */
    public void changeDistanceOffset(double offset) {
        this.distanceOffset += offset;
    }

    /**
     * Changes the turntable (angle) offset of the `rtheta` system.
     * @param offset The amount to change the offset by, in radians.
     */
    public void changeTurntableOffset(double offset) {
        this.turntableOffset += offset;
    }

    public void resetOffsets() {
        this.turntableOffset = 0;
        this.distanceOffset = 0;
    }

    /**
     * Gets the state of the flywheel (whether it is up to speed).
     * @return The state of the flywheel.
     */
    private ModuleState flywheelReady() {
        // Thresholding in units of RPM
        // TODO: test thresholding values
        double diffRPM = Math.abs(flywheelEncoder.getVelocity() - desiredFlywheelRPM);
        return diffRPM < 10 ? ModuleState.HIGH_TOLERANCE
            : diffRPM < 20 ? ModuleState.LOW_TOLERANCE
            : ModuleState.UNALIGNED;
    }

    /**
     * Gets the state of the turntable (whether it is aligned to the hub).
     * @return The state of the turntable.
     */
    private ModuleState turntableAligned() {
        // If the calculated theta is in the blind spot, return UNALIGNED
        if (desiredTurntableRadians < TURNTABLE_MIN_RADIANS || desiredTurntableRadians > TURNTABLE_MAX_RADIANS)
            return ModuleState.UNALIGNED;

        // Thresholding in units of radians
        // TODO: test values
        double diffRads = Math.abs(turntableEncoder.getPosition() - desiredTurntableRadians);
        return diffRads < Math.toRadians(0.5) ? ModuleState.HIGH_TOLERANCE
            : diffRads < Math.toRadians(10) ? ModuleState.LOW_TOLERANCE
            : ModuleState.UNALIGNED;
    }

    /**
     * Gets the state of the hood (whether it is at its desired angle).
     * @return The state of the hood.
     */
    private ModuleState hoodReady() {
        // Thesholding in units of encoder ticks
        // TODO: test thresholding values
        double diffTicks = Math.abs(hood.getSelectedSensorPosition() - desiredHoodRadians * HOOD_RADIANS_TO_TICKS);
        return diffTicks < Math.toRadians(0.5) * HOOD_RADIANS_TO_TICKS ? ModuleState.HIGH_TOLERANCE
            : diffTicks < Math.toRadians(10) * HOOD_RADIANS_TO_TICKS ? ModuleState.LOW_TOLERANCE
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
     * Contrains an angle between [center - pi, center + pi]. Defaults to [0, 2pi].
     * @param angle The angle to wrap.
     * @param center The center of constraint.
     * @return The wrapped angle.
     */
    public double angleWrap(double angle, double center) {
        angle -= center;
        double wrapped = angle - (2 * Math.PI) * Math.round(angle / (2 * Math.PI));
        return wrapped + center;
    }

    public double angleWrap(double angle) {
        return angleWrap(angle, Math.PI);
    }

    /**
     * Cleans up the subsystem for climb (sets the Turret's mode to RETRACTED).
     * In RETRACTED, the turntable faces the same direction as the robot, the hood is retracted, and the flywheel is off.
     */
    @Override
    public void climbInit() {
        mode = TurretMode.RETRACTED;
    }

    public void toggleClimb() {
        if (mode == TurretMode.RETRACTED) {
            mode = TurretMode.SHOOTING;
        } else {
            mode = TurretMode.RETRACTED;
        }
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
     * Sets the velocity of the flywheel. Used exclusively during Test Mode.
     * 
     * @param vel desired flywheel velocity 
     */
    public void setFlywheelVel(double desiredVel) {
        flywheelPidController.setReference(desiredVel, ControlType.kVelocity);
    }
    
    /**
     * Sets the position (in ticks) of the hood. Used exclusively during Test Mode.
     * 
     * @param vel desired position (in ticks) 
     */
    public void setHoodPos(double ticks) { 
        hood.set(ControlMode.Position, ticks);
    }

    /**
     * Turret PID tuning NetworkTable callbacks.
     * @param change The `EntryNotification` representing the NetworkTable entry change.
     */
    private void setTurntableRefPos(EntryNotification change) {
        turntableRefPos = change.value.getDouble();
    }

    private void setTurntableP(EntryNotification change) {
        turntablePidController.setP(change.value.getDouble());
    }

    private void setTurntableI(EntryNotification change) {
        turntablePidController.setI(change.value.getDouble());
    }

    private void setTurntableD(EntryNotification change) {
        turntablePidController.setD(change.value.getDouble());
    }

    private void setTurntableFF(EntryNotification change) {
        turntablePidController.setFF(change.value.getDouble());
    }

    private void setTurntableMaxVel(EntryNotification change) {
        turntablePidController.setSmartMotionMaxVelocity(change.value.getDouble(), 0);
    }

    private void setTurnMinVel(EntryNotification change) {
        turntablePidController.setSmartMotionMinOutputVelocity(change.value.getDouble(), 0);
    }

    private void setTurntableMaxAcc(EntryNotification change) {
        turntablePidController.setSmartMotionMaxAccel(change.value.getDouble(), 0);
    }

    private void setTurntableThetaFF(EntryNotification change) {
        TURNTABLE_THETA_FF = change.value.getDouble();
    }

    private void setFlywheelRefVel(EntryNotification change) {
        flywheelRefVel = change.value.getDouble();
    }

    private void setFlywheelP(EntryNotification change) {
        flywheelPidController.setP(change.value.getDouble());
    }

    private void setFlywheelI(EntryNotification change) {
        flywheelPidController.setI(change.value.getDouble());
    }

    private void setFlywheelD(EntryNotification change) {
        flywheelPidController.setD(change.value.getDouble());
    }

    private void setFlywheelFF(EntryNotification change) {
        flywheelPidController.setFF(change.value.getDouble());
    }

    private void setHoodRefPos(EntryNotification change) {
        hoodRefPos = change.value.getDouble();
    }

    private void setHoodP(EntryNotification change) {
        hood.config_kP(0, change.value.getDouble());
    }

    private void setHoodI(EntryNotification change) {
        hood.config_kI(0, change.value.getDouble());
    }

    private void setHoodD(EntryNotification change) {
        hood.config_kD(0, change.value.getDouble());
    }
}
