package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.intake.DeployIntakeCommand;
import frc.robot.commands.intake.IntakePosition;
import frc.robot.commands.intake.RaiseIntakeCommand;
import frc.robot.jetson.JetsonConnection;
import static frc.robot.Constants.IntakeConstants.*;

public class IntakeSubsystem extends SubsystemBase {
    private final InternalSubsystem internalSubsystem;
    private final JetsonConnection jetson;

    private final CANSparkMax intake;
    private final WPI_TalonSRX deploy;

    private IntakePosition currentPosition;

    // Deploy position PID constants
    private static final double kP = 0.125;
    private static final double kI = 0;
    private static final double kD = 0;

    private final ShuffleboardTab shuffleboardTab;
    private final NetworkTableEntry shuffleboardPEntry;
    private final NetworkTableEntry shuffleboardIEntry;
    private final NetworkTableEntry shuffleboardDEntry;

    // TODO: measure this
    private static final double DEGREES_TO_ENCODER_TICKS = 1.0;

    public IntakeSubsystem(InternalSubsystem internalSubsystem, JetsonConnection jetson) {
        this.internalSubsystem = internalSubsystem;
        this.jetson = jetson;

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
        shuffleboardTab.add("Raise", new RaiseIntakeCommand(this));
        shuffleboardTab.add("Deploy", new DeployIntakeCommand(this));
    }

    @Override
    public void periodic() {
        // Get PID constants from Shuffleboard for testing
        deploy.config_kP(0, shuffleboardPEntry.getDouble(kP));
        deploy.config_kI(0, shuffleboardIEntry.getDouble(kI));
        deploy.config_kD(0, shuffleboardDEntry.getDouble(kD));

        // If the jetson detects a ball, the intake is deployed, and there are less than 2 balls in internals, 
        // run the motors
        intake.set(jetson.ballDetected() 
            && currentPosition == IntakePosition.DEPLOYED 
            && internalSubsystem.getBallCount() < 2
            ? 0.5 : 0);
    }

    /**
     * Sets the position of the intake mechanism.
     * @param position The position to set the intake to.
     */
    public void setPosition(IntakePosition position) {
        currentPosition = position;
        deploy.set(ControlMode.Position, position.value * DEGREES_TO_ENCODER_TICKS);
    }
}
