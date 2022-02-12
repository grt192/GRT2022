package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import frc.robot.GRTSubsystem;
import frc.robot.brownout.PowerController;
import frc.robot.commands.intake.DeployIntakeCommand;
import frc.robot.commands.intake.RaiseIntakeCommand;
import frc.robot.jetson.JetsonConnection;

import static frc.robot.Constants.IntakeConstants.*;

public class IntakeSubsystem extends GRTSubsystem {
    /**
     * An enum representing the position of the intake, with `IntakePosition.value` representing the 
     * counterclockwise angle from straight upwards.
     */
    public enum IntakePosition {
        RAISED(0), DEPLOYED(-489839);

        public final double value;

        private IntakePosition(double value) {
            this.value = value;
        }
    }

    //private final InternalSubsystem internalSubsystem;
    //private final JetsonConnection jetson;

    private final CANSparkMax intake;
    private final WPI_TalonSRX deploy;

    private IntakePosition currentPosition = IntakePosition.DEPLOYED;

    private double intakePower = 0;

    // Deploy position PID constants
    private static final double kP = 0.125;
    private static final double kI = 0;
    private static final double kD = 0;

    private final ShuffleboardTab shuffleboardTab;
    private final NetworkTableEntry shuffleboardPEntry;
    private final NetworkTableEntry shuffleboardIEntry;
    private final NetworkTableEntry shuffleboardDEntry;
    private final NetworkTableEntry shuffleboardDeployPosition;

    // TODO: measure this
    private static final double DEGREES_TO_ENCODER_TICKS = 1.0;

    public IntakeSubsystem(/*InternalSubsystem internalSubsystem, JetsonConnection jetson*/) {
        // TODO: measure this
        super(50);

        /*
        this.internalSubsystem = internalSubsystem;
        this.jetson = jetson;
        */

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

        // Initialize Shuffleboard entries
        shuffleboardTab = Shuffleboard.getTab("Intake");
        shuffleboardPEntry = shuffleboardTab.add("kP", kP).getEntry();
        shuffleboardIEntry = shuffleboardTab.add("kI", kI).getEntry();
        shuffleboardDEntry = shuffleboardTab.add("kD", kD).getEntry();

        shuffleboardDeployPosition = shuffleboardTab.add("Deploy Position", 0).getEntry();

        shuffleboardTab.add("Raise", new RaiseIntakeCommand(this));
        shuffleboardTab.add("Deploy", new DeployIntakeCommand(this));
    }

    @Override
    public void periodic() {
        // Get PID constants from Shuffleboard for testing
        deploy.config_kP(0, shuffleboardPEntry.getDouble(kP));
        deploy.config_kI(0, shuffleboardIEntry.getDouble(kI));
        deploy.config_kD(0, shuffleboardDEntry.getDouble(kD));

        boolean readyToIntake = /*(jetson.ballDetected() || driverRequesting) && */ 
            currentPosition == IntakePosition.DEPLOYED; 

        // If the jetson detects a ball or the driver is running the intake, the intake is deployed, 
        // and there are less than 2 balls in internals, run the intake motor
        intake.set(readyToIntake ? intakePower : 0);
            
        // System.out.println("Intake power " + intake.get());
        // System.out.println("Deploy power " + deploy.get());

        // System.out.println("deploy pos: " + deploy.getSelectedSensorPosition());
        shuffleboardDeployPosition.setDouble(deploy.getSelectedSensorPosition());
    }

    /**
     * Temp testing function to supply raw power to the deploy motor.
     * @param power The percent power to supply.
     */
    public void setDeployPower(double power) {
        deploy.set(power);
    }

    /**
     * Sets whether the driver is attempting to run the intake. Note that this will override `jetson.ballDetected()` but *not*
     * the other conditions; intake will not run if the mechanism is not deployed or there are too many balls in internals.
     * @param requesting Whether the driver is running the intake.
     */
    public void setIntakePower(double intakePower) {
        this.intakePower = intakePower;
    }

    /**
     * Sets the position of the intake mechanism.
     * @param position The position to set the intake to.
     */
    public void setPosition(IntakePosition position) {
        currentPosition = position;
        deploy.set(ControlMode.Position, position.value * DEGREES_TO_ENCODER_TICKS);
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
