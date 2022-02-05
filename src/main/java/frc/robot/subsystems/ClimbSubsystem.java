package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
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
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import frc.robot.GRTSubsystem;
import frc.robot.brownout.PowerController;

import static frc.robot.Constants.ClimbConstants.*;

public class ClimbSubsystem extends GRTSubsystem {
    private final CANSparkMax six;
    private final RelativeEncoder sixEncoder;
    private final SparkMaxPIDController sixPidController;
    private final WPI_TalonSRX sixBrake;

    private final CANSparkMax ten;
    private final RelativeEncoder tenEncoder;
    private final SparkMaxPIDController tenPidController;
    private final WPI_TalonSRX tenBrake;

    private final WPI_TalonSRX tenSolenoidMain;
    private final WPI_TalonSRX tenSolenoidFollow;

    private final WPI_TalonSRX fifteenMain;
    private final WPI_TalonSRX fifteenFollow;

    // Six point arm position PID constants
    private static final double sixP = 0.125;
    private static final double sixI = 0;
    private static final double sixD = 0;

    // Ten point arm position PID constants
    private static final double tenP = 0.125;
    private static final double tenI = 0;
    private static final double tenD = 0;

    // Fifteen point arm position PID constants
    private static final double fifteenP = 0.125;
    private static final double fifteenI = 0;
    private static final double fifteenD = 0;

    private final ShuffleboardTab shuffleboardTab;
    private final NetworkTableEntry shuffleboardSixPEntry;
    private final NetworkTableEntry shuffleboardSixIEntry;
    private final NetworkTableEntry shuffleboardSixDEntry;

    private final NetworkTableEntry shuffleboardTenPEntry;
    private final NetworkTableEntry shuffleboardTenIEntry;
    private final NetworkTableEntry shuffleboardTenDEntry;

    private final NetworkTableEntry shuffleboardFifteenPEntry;
    private final NetworkTableEntry shuffleboardFifteenIEntry;
    private final NetworkTableEntry shuffleboardFifteenDEntry;

    public ClimbSubsystem() {
        // TODO: measure this
        super(20);

        // Initialize six point arm NEO, encoder PID, and solenoid brake
        six = new CANSparkMax(sixMotorPort, MotorType.kBrushless);
        six.restoreFactoryDefaults();
        six.setIdleMode(IdleMode.kBrake);

        sixEncoder = six.getEncoder();
        sixEncoder.setPosition(0);

        sixPidController = six.getPIDController();
        sixPidController.setP(sixP);
        sixPidController.setI(sixI);
        sixPidController.setD(sixD);

        sixBrake = new WPI_TalonSRX(sixBrakePort);
        sixBrake.configFactoryDefault();

        // Initialize ten point arm NEO, encoder PID, and solenoid brake
        ten = new CANSparkMax(tenMotorPort, MotorType.kBrushless);
        ten.restoreFactoryDefaults();
        ten.setIdleMode(IdleMode.kBrake);

        tenEncoder = ten.getEncoder();
        tenEncoder.setPosition(0);

        tenPidController = ten.getPIDController();
        tenPidController.setP(tenP);
        tenPidController.setI(tenI);
        tenPidController.setD(tenD);

        tenBrake = new WPI_TalonSRX(tenBrakePort);
        tenBrake.configFactoryDefault();

        // Initialize ten point arm solenoid release mechanisms
        tenSolenoidMain = new WPI_TalonSRX(tenLeftSolenoidPort);
        tenSolenoidMain.configFactoryDefault();

        tenSolenoidFollow = new WPI_TalonSRX(tenRightSolenoidPort);
        tenSolenoidFollow.configFactoryDefault();
        tenSolenoidFollow.follow(tenSolenoidMain);

        // Initialize fifteen point arm 775, encoder PID, and follower
        fifteenMain = new WPI_TalonSRX(fifteenLeftPort);
        fifteenMain.configFactoryDefault();
        fifteenMain.setNeutralMode(NeutralMode.Brake);

        fifteenMain.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        fifteenMain.setSelectedSensorPosition(0);
        fifteenMain.setSensorPhase(false);
        fifteenMain.config_kP(0, fifteenP);
        fifteenMain.config_kI(0, fifteenI);
        fifteenMain.config_kD(0, fifteenD);

        fifteenFollow = new WPI_TalonSRX(fifteenRightPort);
        fifteenFollow.configFactoryDefault();
        fifteenFollow.follow(fifteenMain);
        fifteenMain.setNeutralMode(NeutralMode.Brake);

        // Initialize Shuffleboard entries
        shuffleboardTab = Shuffleboard.getTab("Climb");
        shuffleboardSixPEntry = shuffleboardTab.add("Six point arm kP", sixP).getEntry();
        shuffleboardSixIEntry = shuffleboardTab.add("Six point arm kI", sixI).getEntry();
        shuffleboardSixDEntry = shuffleboardTab.add("Six point arm kD", sixD).getEntry();

        shuffleboardTenPEntry = shuffleboardTab.add("Ten point arm kP", tenP).getEntry();
        shuffleboardTenIEntry = shuffleboardTab.add("Ten point arm kI", tenI).getEntry();
        shuffleboardTenDEntry = shuffleboardTab.add("Ten point arm kD", tenD).getEntry();

        shuffleboardFifteenPEntry = shuffleboardTab.add("Fifteen point arm kP", fifteenP).getEntry();
        shuffleboardFifteenIEntry = shuffleboardTab.add("Fifteen point arm kI", fifteenI).getEntry();
        shuffleboardFifteenDEntry = shuffleboardTab.add("Fifteen point arm kD", fifteenD).getEntry();
    }

    @Override
    public void periodic() {
        // Get PID constants from Shuffleboard for testing
        sixPidController.setP(shuffleboardSixPEntry.getDouble(sixP));
        sixPidController.setP(shuffleboardSixIEntry.getDouble(sixI));
        sixPidController.setP(shuffleboardSixDEntry.getDouble(sixD));

        tenPidController.setP(shuffleboardTenPEntry.getDouble(tenP));
        tenPidController.setP(shuffleboardTenIEntry.getDouble(tenI));
        tenPidController.setP(shuffleboardTenDEntry.getDouble(tenD));

        fifteenMain.config_kP(0, shuffleboardFifteenPEntry.getDouble(fifteenP));
        fifteenMain.config_kI(0, shuffleboardFifteenIEntry.getDouble(fifteenI));
        fifteenMain.config_kD(0, shuffleboardFifteenDEntry.getDouble(fifteenD));
    }

    /**
     * PHASE 1 (6 point rung)
     * TODO: measure these positions (for all phases)
     */
    public class ClimbPhase1Command extends CommandBase {
        @Override
        public void initialize() {
            // Disengage the six point arm brake and extend the arm to grab the rung
            sixBrake.set(0);
            sixPidController.setReference(20, ControlType.kPosition);
        }

        @Override
        public boolean isFinished() {
            return withinThreshold(sixEncoder.getPosition(), 20);
        }

        @Override
        public void end(boolean interrupted) {
            // Engage the brake and partially retract the arm after grabbing
            sixBrake.set(1);
            sixPidController.setReference(10, ControlType.kPosition);
        }
    }

    /**
     * PHASE 2 (10 point rung)
     */
    public class ClimbPhase2Command extends CommandBase {
        @Override
        public void initialize() {
            // Disengage the ten point arm brake and extend the arm, simultaneously further retracting the six point arm.
            // After this motion the robot should be flat under the 10 point rung.
            tenBrake.set(0);
            tenPidController.setReference(20, ControlType.kPosition);
            sixPidController.setReference(5, ControlType.kPosition);
        }

        @Override
        public boolean isFinished() {
            return withinThreshold(tenEncoder.getPosition(), 20)
                && withinThreshold(sixEncoder.getPosition(), 5);
        }

        @Override
        public void end(boolean interrupted) {
            // Engage the brake and partially retract the arm after grabbing
            tenBrake.set(1);
            tenPidController.setReference(10, ControlType.kPosition);
        }
    }

    /**
     * PHASE 3 (15 point rung)
     */
    public class ClimbPhase3Command extends CommandBase {
        @Override
        public void initialize() {
            // Extend the 15 point arms and grab the bar
            fifteenMain.set(ControlMode.Position, 20);
        }

        @Override
        public boolean isFinished() {
            return withinThreshold(fifteenMain.getSelectedSensorPosition(), 20);
        }

        @Override
        public void end(boolean interrupted) {
            // Release the six point arm by extending it, and the ten point arm using a solenoid mechanism
            sixBrake.set(0);
            sixPidController.setReference(20, ControlType.kPosition);
            tenSolenoidMain.set(1);
        }
    }

    /**
     * Convenience method to perform thresholding with the same value on all climb position PID loops.
     * @param value The current encoder reading.
     * @param desired The desired motor position.
     * @return Whether the reading is within threshold of the desired value.
     * TODO: tune this threshold value
     */
    public boolean withinThreshold(double value, double desired) {
        return Math.abs(value - desired) < 1;
    }

    /**
     * Gets the climb sequence as a SequentialCommandGroup.
     * @return The command group representing the climb sequence.
     */
    public SequentialCommandGroup climb() {
        return new ClimbPhase1Command().andThen(
            new WaitUntilCommand(() -> withinThreshold(sixEncoder.getPosition(), 10)),
            new ClimbPhase2Command(),
            new WaitUntilCommand(() -> withinThreshold(tenEncoder.getPosition(), 10)),
            new ClimbPhase3Command()
        );
    }

    @Override
    public double getTotalCurrentDrawn() {
        return PowerController.getCurrentDrawnFromPDH(
            sixMotorPort, sixBrakePort, tenMotorPort, tenBrakePort, 
            fifteenLeftPort, fifteenRightPort, tenLeftSolenoidPort, tenRightSolenoidPort
        );
    }

    @Override
    public void setCurrentLimit(double limit) {
        // TODO: do the solenoids draw significant power? should they be factored in to the limit calculation?
        int motorLimit = (int) Math.floor(limit / 4);

        six.setSmartCurrentLimit(motorLimit);
        ten.setSmartCurrentLimit(motorLimit);
        fifteenMain.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, motorLimit, 0, 0));
    }
}
