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
import frc.robot.shuffleboard.GRTShuffleboardTab;
import frc.robot.subsystems.internals.InternalSubsystem;

import static frc.robot.Constants.IntakeConstants.*;

public class IntakeSubsystem extends GRTSubsystem {
    /**
     * An enum representing the position of the intake, with `IntakePosition.value`
     * representing the counterclockwise angle from straight upwards.
     */
    public enum IntakePosition {
        START(0), RAISED(10355), DEPLOYED(105248);

        public final double value;

        private IntakePosition(double value) {
            this.value = value;
        }
    }

    private final InternalSubsystem internalSubsystem;
    private final JetsonConnection jetson;

    private final CANSparkMax intake;
    private final WPI_TalonSRX deploy;

    private final DigitalInput topLimitSwitch;
    private final Timer topTimer = new Timer();
    private final DigitalInput bottomLimitSwitch;
    private final Timer bottomTimer = new Timer();

    private static final double TOP_LIMIT_DELAY = 0.01;
    private static final double BOTTOM_LIMIT_DELAY = 0.01;

    private double driverPower = 0;
    private boolean driverOverride = false;
    private double autoPower = 0;
    private boolean autoOverride = false;

    private boolean skipInternalsCheck = false;

    private boolean autoDeployIntake = false;
    private IntakePosition targetPosition = IntakePosition.START;

    // Deploy position PID constants
    private static final double kP = 0.05;
    private static final double kI = 0;
    private static final double kD = 0;
    private static final double kFF = 0.03;
    private static final double cruiseVel = 7000;
    private static final double accel = 12000;
    private static final int sCurveStrength = 3;

    // Shuffleboard
    private final GRTShuffleboardTab shuffleboardTab;
    private final GRTNetworkTableEntry deployPosEntry, deployVelEntry, targetPosEntry;
    private final GRTNetworkTableEntry driverPowerEntry, driverOverrideEntry, autoPowerEntry, autoOverrideEntry;

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
        //deploy.setInverted(true);
        deploy.setNeutralMode(NeutralMode.Brake);

        deploy.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        deploy.setSelectedSensorPosition(IntakePosition.START.value);
        //deploy.setSensorPhase(true);
        deploy.config_kP(0, kP);
        deploy.config_kI(0, kI);
        deploy.config_kD(0, kD);
        deploy.config_kF(0, kFF);
        deploy.configMotionCruiseVelocity(cruiseVel);
        deploy.configMotionAcceleration(accel);
        deploy.configMotionSCurveStrength(sCurveStrength);

        // Soft limit deploy to the START position. The soft limit in the other direction is not
        // needed because of the limit switch and hard stop.
        deploy.configReverseSoftLimitEnable(false);
        deploy.configReverseSoftLimitThreshold(IntakePosition.START.value);

        topLimitSwitch = new DigitalInput(tLimitSwitchPort);
        bottomLimitSwitch = new DigitalInput(bLimitSwitchPort);

        // Initialize Shuffleboard entries
        shuffleboardTab = new GRTShuffleboardTab("Intake");
        deployPosEntry = shuffleboardTab.addEntry("Deploy pos", deploy.getSelectedSensorPosition()).at(3, 0);
        deployVelEntry = shuffleboardTab.addEntry("Deploy vel", deploy.getSelectedSensorVelocity()).at(3, 1);
        targetPosEntry = shuffleboardTab.addEntry("Target pos", targetPosition.value).at(3, 2);

        driverOverrideEntry = shuffleboardTab.addEntry("Driver override", driverOverride).at(0, 0);
        driverPowerEntry = shuffleboardTab.addEntry("Driver power", driverPower).at(0, 1);
        autoOverrideEntry = shuffleboardTab.addEntry("Auto override", autoOverride).at(1, 0);
        autoPowerEntry = shuffleboardTab.addEntry("Auto power", autoPower).at(1, 1);

        shuffleboardTab.addToggle("Skip internals check", skipInternalsCheck, this::setSkipInternalsCheck, 0, 2);

        // If DEBUG_PID is set, allow for PID tuning on shuffleboard
        if (DEBUG_PID) {
            shuffleboardTab
                .list("Deploy PID")
                .at(4, 0)
                .withSize(1, 4)
                .addListener("kP", kP, this::setDeployP)
                .addListener("kI", kI, this::setDeployI)
                .addListener("kD", kD, this::setDeployD)
                .addListener("kFF", kFF, this::setDeployFF)
                .addListener("Cruise Vel", cruiseVel, this::setDeployCruiseVel)
                .addListener("Accel", accel, this::setDeployAccel);

            shuffleboardTab.addEntry("IMPORTANT REMINDER", "Remember to disable phoenix tuner control and the robot before changing these values!")
                .at(5, 0).withSize(2, 2);
        }

        shuffleboardTab
            .list("Commands")
            .at(2, 0)
            .withSize(1, 2)
            .addWidget("Raise", new RaiseIntakeCommand(this))
            .addWidget("Deploy", new DeployIntakeCommand(this, 0));
    }

    @Override
    public void periodic() {
        //delayLimitSwitchReset(bottomLimitSwitch, bottomTimer, BOTTOM_LIMIT_DELAY, IntakePosition.DEPLOYED.value);
        delayLimitSwitchReset(topLimitSwitch, topTimer, TOP_LIMIT_DELAY, IntakePosition.START.value);

        // If the ball count is greater than 2, don't run intake.
        // Skip this check if disabled on shuffleboard.
        double power;
        if (internalSubsystem.getBallCount() > 2 && !skipInternalsCheck) {
            power = 0;
        } else {
            // Otherwise, use auto input if it's overriding, driver input if they're overriding 
            // and default to running intake automatically from vision.
            power = autoOverride ? autoPower
                : driverOverride ? driverPower
                : jetson.ballDetected() ? 0.5 : 0;
        }

        intake.set(power);
        /*
        moveDeployTo(autoDeployIntake && power > 0.1
            ? IntakePosition.DEPLOYED.value
            : targetPosition.value);
        */
        /*
        deploy.set(ControlMode.MotionMagic, autoDeployIntake && power > 0.1
            ? IntakePosition.DEPLOYED.value
            : targetPosition.value);
        */

        deployPosEntry.setValue(deploy.getSelectedSensorPosition());
        deployVelEntry.setValue(deploy.getSelectedSensorVelocity());
        targetPosEntry.setValue(targetPosition.value);

        driverPowerEntry.setValue(driverPower);
        driverOverrideEntry.setValue(driverOverride);
        autoPowerEntry.setValue(autoPower);
        autoOverrideEntry.setValue(autoOverride);
    }

    private void moveDeployTo(double targPos) {
        double currentPos = deploy.getSelectedSensorPosition();

        //System.out.println("current: " + currentPos + " targ: " + targPos);
        if (Math.abs(targPos - currentPos) < 2000) {
            deploy.set(0);
        } else if (targPos > currentPos || targPos == IntakePosition.DEPLOYED.value) { // going down
            if (currentPos < 50000) {
                deploy.set(0.55);
            } else {
                deploy.set(0.25);
            }
        } else { // going up
            if (currentPos < 50000) {
                deploy.set(-0.1);
            } else {
                deploy.set(-0.6);
            }
        }
    }

    /**
     * Checks the target limit switch for whether the intake is touching it and resets the encoder on
     * the given delay to prevent uneven tilting on the limit switch side.
     * 
     * @param limitSwitch The limit switch to check.
     * @param timer The timer corresponding to that limit switch.
     * @param delay The delay to reset to the position after, in seconds.
     * @param resetPos The position to reset to, in encoder ticks.
     */
    private void delayLimitSwitchReset(DigitalInput limitSwitch, Timer timer, double delay, double resetPos) {
        if (!limitSwitch.get()) {
            timer.start();
        } else {
            timer.stop();
            timer.reset();
        }

        if (timer.hasElapsed(delay))
            deploy.setSelectedSensorPosition(resetPos);
    }

    /**
     * Sets the driver requested power of the intake rollers. This power will be set
     * when driver override is enabled.
     * @param driverPower The power (in percent output) to run the motors at.
     */
    public void setDriverPower(double driverPower) {
        this.driverPower = driverPower;
    }

    /**
     * Sets whether the driver is overriding the intake's automatic run procedure.
     * @param override Whether to use driver input as power.
     */
    public void setDriverOverride(boolean override) {
        driverOverride = override;
    }

    /**
     * Sets the auton requested power of the intake rollers. This power will be set
     * when auton override is enabled.
     * @param driverPower The power (in percent output) to run the motors at.
     */
    public void setAutoPower(double autoPower) {
        this.autoPower = autoPower;
    }

    /**
     * Sets whether auton is overriding the intake's automatic run procedure.
     * @param override Whether to use auton power as power.
     */
    public void setAutoOverride(boolean override) {
        autoOverride = override;
    }

    /**
     * Sets the position of the intake mechanism.
     * @param position The position to set the intake to.
     */
    public void setPosition(IntakePosition position) {
        targetPosition = position;
    }

    /**
     * Toggles the position of the intake between DEPLOYED and RAISED.
     */
    public void togglePosition() {
        this.targetPosition = this.targetPosition == IntakePosition.DEPLOYED
            ? IntakePosition.RAISED
            : IntakePosition.DEPLOYED;
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

    private void setSkipInternalsCheck(EntryNotification change) {
        skipInternalsCheck = change.value.getBoolean();
    }
}
