package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import frc.robot.GRTSubsystem;
import frc.robot.brownout.PowerController;
import frc.robot.commands.intake.DeployIntakeCommand;
import frc.robot.commands.intake.RaiseIntakeCommand;
import frc.robot.jetson.JetsonConnection;
import frc.robot.shuffleboard.GRTNetworkTableEntry;
import frc.robot.subsystems.internals.InternalSubsystem;

import static frc.robot.Constants.IntakeConstants.*;

public class IntakeSubsystem extends GRTSubsystem {
    /**
     * An enum representing the position of the intake, with `IntakePosition.value`
     * representing the counterclockwise angle from straight upwards.
     */
    public enum IntakePosition {
        START(0), RAISED(17214), DEPLOYED(486209);

        public final double value;

        private IntakePosition(double value) {
            this.value = value;
        }
    }

    private final InternalSubsystem internalSubsystem;
    private final JetsonConnection jetson;

    private final CANSparkMax intake;
    private final WPI_TalonSRX deploy;
    private final DigitalInput limitSwitch;

    private double intakePower = 0;
    private boolean driverOverride = false;

    public static final double DELAY_LIMIT_RESET = 0.3;
    private Double switchPressed = 0.0;

    public boolean autoDeployIntake = false;
    private IntakePosition targetPosition = IntakePosition.START;

    // Deploy position PID constants
    private static final double kP = 0.1;
    private static final double kI = 0;
    private static final double kD = 0;
    private static final double kFF = 0.016;
    private static final double cruiseVel = 20000;
    private static final double accel = 40000;
    private static final int sCurveStrength = 3;

    private final ShuffleboardTab shuffleboardTab;
    private final GRTNetworkTableEntry shuffleboardDeployPosition;
    private final GRTNetworkTableEntry shuffleboardVeloEntry;

    // Debug flags
    // Whether PID tuning shuffleboard entries should be displayed
    private static boolean DEBUG_PID = true;

    public IntakeSubsystem(InternalSubsystem internalSubsystem, JetsonConnection jetson) {
        // TODO: measure this
        super(50);

        this.internalSubsystem = internalSubsystem;
        this.jetson = jetson;

        // Initialize the intake (roller) motor
        intake = new CANSparkMax(intakePort, MotorType.kBrushless);
        intake.restoreFactoryDefaults();

        // Initialize the deploy (intake position) motor
        deploy = new WPI_TalonSRX(deploymentPort);
        deploy.configFactoryDefault();
        deploy.setInverted(true);
        deploy.setNeutralMode(NeutralMode.Brake);

        deploy.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        deploy.setSelectedSensorPosition(IntakePosition.START.value);
        deploy.setSensorPhase(true);
        deploy.config_kP(0, kP);
        deploy.config_kI(0, kI);
        deploy.config_kD(0, kD);
        deploy.config_kF(0, kFF);
        deploy.configMotionCruiseVelocity(cruiseVel);
        deploy.configMotionAcceleration(accel);
        deploy.configMotionSCurveStrength(sCurveStrength);

        // Soft limit deploy to the START position. The soft limit in the other direction is not
        // needed because of the limit switch and hard stop.
        deploy.configReverseSoftLimitEnable(true);
        deploy.configReverseSoftLimitThreshold(IntakePosition.START.value);

        limitSwitch = new DigitalInput(limitSwitchPort);

        // Initialize Shuffleboard entries
        shuffleboardTab = Shuffleboard.getTab("Intake");
        shuffleboardVeloEntry = new GRTNetworkTableEntry(
            shuffleboardTab.add("velo", deploy.getSelectedSensorVelocity()).getEntry());
        shuffleboardDeployPosition = new GRTNetworkTableEntry(shuffleboardTab.add("Deploy position", 0).getEntry());

        // If DEBUG_PID is set, allow for PID tuning on shuffleboard
        if (DEBUG_PID) {
            shuffleboardTab.add("kP", kP).getEntry()
                .addListener(this::setDeployP, EntryListenerFlags.kUpdate);
            shuffleboardTab.add("kI", kI).getEntry()
                .addListener(this::setDeployI, EntryListenerFlags.kUpdate);
            shuffleboardTab.add("kD", kD).getEntry()
                .addListener(this::setDeployD, EntryListenerFlags.kUpdate);
            shuffleboardTab.add("kFF", kFF).getEntry()
                .addListener(this::setDeployFF, EntryListenerFlags.kUpdate);
            shuffleboardTab.add("Cruise Vel", cruiseVel).getEntry()
                .addListener(this::setDeployCruiseVel, EntryListenerFlags.kUpdate);
            shuffleboardTab.add("Accel", accel).getEntry()
                .addListener(this::setDeployAccel, EntryListenerFlags.kUpdate);
        }

        shuffleboardTab.add("Raise", new RaiseIntakeCommand(this));
        shuffleboardTab.add("Deploy", new DeployIntakeCommand(this));
    }

    @Override
    public void periodic() {
        limitSwitchReset();

        double power;
        // If the ball count is greater than 2, don't run intake
        if (internalSubsystem.getBallCount() > 2) {
            power = 0;
        } else {
            // Otherwise, use driver input if they're overriding and default to running
            // intake automatically from vision
            power = driverOverride ? intakePower
                    : jetson.ballDetected() ? 0.5 : 0;
        }

        intake.set(power);

        if (autoDeployIntake && power > 0.1) {
            deploy.set(ControlMode.MotionMagic, IntakePosition.DEPLOYED.value);
        } else {
            deploy.set(ControlMode.MotionMagic, targetPosition.value);
        }

        shuffleboardDeployPosition.setValue(deploy.getSelectedSensorPosition());
        shuffleboardVeloEntry.setValue(deploy.getSelectedSensorVelocity());
    }

    private void limitSwitchReset() {
        // Check limit switch and reset encoder if detected
        // If the limit switch returns `false`, it's being pressed and the encoder should be reset
        if (!limitSwitch.get())
            if (switchPressed == null) switchPressed = Timer.getFPGATimestamp();
        else switchPressed = null;

        if (switchPressed != null && Timer.getFPGATimestamp() > switchPressed + DELAY_LIMIT_RESET)
            deploy.setSelectedSensorPosition(IntakePosition.DEPLOYED.value);
    }

    /**
     * Sets the power of the intake rollers.
     * @param intakePower The power (in percent output) to run the motors at.
     */
    public void setIntakePower(double intakePower) {
        this.intakePower = intakePower;
    }

    /**
     * Sets the position of the intake mechanism.
     * @param position The position to set the intake to.
     */
    public void setPosition(IntakePosition position) {
        targetPosition = position;
    }

    public void togglePosition() {
        if (this.targetPosition == IntakePosition.DEPLOYED) {
            this.targetPosition = IntakePosition.RAISED;
        } else {
            this.targetPosition = IntakePosition.DEPLOYED;
        }
    }

    /**
     * Sets whether the driver is overriding the intake's automatic run procedure.
     * @param override Whether to use driver input as power.
     */
    public void setDriverOverride(boolean override) {
        driverOverride = override;
    }

    @Override
    public double getTotalCurrentDrawn() {
        return PowerController.getCurrentDrawnFromPDH(intakePort, deploymentPort);
    }

    @Override
    public void setCurrentLimit(double limit) {
        int motorLimit = (int) Math.floor(limit / 2);

        deploy.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, motorLimit, 0, 0));
        intake.setSmartCurrentLimit(motorLimit);
    }

    /**
     * Intake PID tuning NetworkTable callbacks.
     * @param change The `EntryNotification` representing the NetworkTable entry change.
     */
    private void setDeployP(EntryNotification change) {
        deploy.config_kP(0, change.value.getDouble());
    }

    private void setDeployI(EntryNotification change) {
        deploy.config_kI(0, change.value.getDouble());
    }

    private void setDeployD(EntryNotification change) {
        deploy.config_kD(0, change.value.getDouble());
    }

    private void setDeployFF(EntryNotification change) {
        deploy.config_kF(0, change.value.getDouble());
    }

    private void setDeployCruiseVel(EntryNotification change) {
        deploy.configMotionCruiseVelocity(change.value.getDouble());
    }

    private void setDeployAccel(EntryNotification change) {
        deploy.configMotionAcceleration(change.value.getDouble());
    }
}
