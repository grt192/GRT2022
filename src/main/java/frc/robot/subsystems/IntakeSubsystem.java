package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.deploymentPort;
import static frc.robot.Constants.IntakeConstants.intakePort;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.GRTSubsystem;
import frc.robot.brownout.PowerController;
import frc.robot.subsystems.internals.InternalSubsystem;

public class IntakeSubsystem extends GRTSubsystem {
    /**
     * An enum representing the position of the intake, with `IntakePosition.value`
     * representing the
     * counterclockwise angle from straight upwards. In degrees.
     */
    public enum IntakePosition {
        RAISED(0), DEPLOYED(200000);

        public final double value;

        private IntakePosition(double value) {
            this.value = value;
        }
    }

    private final InternalSubsystem internalSubsystem;
    // private final JetsonConnection jetson;

    private final CANSparkMax intake;
    private final WPI_TalonSRX deploy;

    private IntakePosition currentPosition = IntakePosition.DEPLOYED;

    private double intakePower = 0;

    // Deploy position PID constants
    private static final double kP = 0.125;
    private static final double kI = 0;
    private static final double kD = 0;

    private final ShuffleboardTab shuffleboardTab = null;
    private final NetworkTableEntry shuffleboardPEntry = null;
    private final NetworkTableEntry shuffleboardIEntry = null;
    private final NetworkTableEntry shuffleboardDEntry = null;
    private final NetworkTableEntry shuffleboardDeployPosition = null;

    // TODO: measure this
    // private static final double DEGREES_TO_ENCODER_TICKS = 1.0;

    public IntakeSubsystem(InternalSubsystem internalSubsystem /* , JetsonConnection jetson */) {
        // TODO: measure this
        super(50);

        this.internalSubsystem = internalSubsystem;
        // this.jetson = jetson;

        // Initialize the intake (roller) motor
        intake = new CANSparkMax(intakePort, MotorType.kBrushless);
        intake.restoreFactoryDefaults();

        // Initialize the deploy (intake position) motor
        deploy = new WPI_TalonSRX(deploymentPort);
        deploy.configFactoryDefault();
        deploy.setNeutralMode(NeutralMode.Brake);

        deploy.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        deploy.setSelectedSensorPosition(0);
        deploy.setSensorPhase(false);
        deploy.config_kP(0, kP);
        deploy.config_kI(0, kI);
        deploy.config_kD(0, kD);

        // Soft limit deploy between RAISED and DEPLOYED
        /*
        deploy.configForwardSoftLimitEnable(true);
        deploy.configReverseSoftLimitEnable(true);
        deploy.configForwardSoftLimitThreshold(IntakePosition.RAISED.value);
        deploy.configReverseSoftLimitThreshold(IntakePosition.DEPLOYED.value);
        */

        // Initialize Shuffleboard entries
        // shuffleboardTab = Shuffleboard.getTab("Intake");
        // shuffleboardPEntry = shuffleboardTab.add("kP", kP).getEntry();
        // shuffleboardIEntry = shuffleboardTab.add("kI", kI).getEntry();
        // shuffleboardDEntry = shuffleboardTab.add("kD", kD).getEntry();

        // shuffleboardDeployPosition = shuffleboardTab.add("Deploy Position", 0).getEntry();

        // shuffleboardTab.add("Raise", new RaiseIntakeCommand(this));
        // shuffleboardTab.add("Deploy", new DeployIntakeCommand(this));
    }

    @Override
    public void periodic() {
        // Get PID constants from Shuffleboard for testing
        // deploy.config_kP(0, shuffleboardPEntry.getDouble(kP));
        // deploy.config_kI(0, shuffleboardIEntry.getDouble(kI));
        // deploy.config_kD(0, shuffleboardDEntry.getDouble(kD));
        

        // If the jetson detects a ball or the driver is running the intake, the intake
        // is deployed,
        // and there are less than 2 balls in internals, run the intake motor
        // TODO: how should we work in the jetson ball detection code with variable
        // intake speeds from the driver?
        // Also, if running the intake in reverse is an option, how should internals
        // ball count be worked into this?
        boolean readyToIntake = /* internalSubsystem.getBallCount() < 2 && */
                currentPosition == IntakePosition.DEPLOYED;

        intake.set(readyToIntake ? intakePower : 0);

        // shuffleboardDeployPosition.setDouble(deploy.getSelectedSensorPosition());
    }

    /**
     * Temp testing function to supply raw power to the deploy motor.
     * 
     * @param power The percent power to supply.
     */
    public void setDeployPower(double power) {
        deploy.set(power);
    }

    /**
     * Sets the power of the intake rollers.
     * 
     * @param intakePower The power (in percent output) to run the motors at.
     */
    public void setIntakePower(double intakePower) {
        this.intakePower = intakePower;
    }

    /**
     * Sets the position of the intake mechanism.
     * 
     * @param position The position to set the intake to.
     */
    public void setPosition(IntakePosition position) {
        currentPosition = position;
        deploy.set(ControlMode.Position, position.value /* * DEGREES_TO_ENCODER_TICKS */);
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
}
