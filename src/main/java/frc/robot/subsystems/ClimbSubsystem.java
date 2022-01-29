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

import static frc.robot.Constants.ClimbConstants.*;

public class ClimbSubsystem extends SubsystemBase {
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

        // Initialize ten point arm NEO, encoder PID, and solenoide brake
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
     * Start the climb sequence!
     * TODO: measure these positions
     * TODO: think about how we can add delays between the climb actions/phases (timer? driver input?)
     */
    public void climb() {
        // PHASE 1 (6 point rung): disengage the six point arm brake and extend the arm, engaging the brake and partially 
        // retracting the arm after grabbing the bar.
        sixBrake.set(0);
        sixPidController.setReference(20, ControlType.kPosition);
        sixBrake.set(1);
        sixPidController.setReference(10, ControlType.kPosition);

        // PHASE 2 (10 point rung): disengage the ten point arm brake and extend the arm, engaging the brake and partially 
        // retracting the arm after grabbing the bar. Simultaneously further retract the six point arm.
        tenBrake.set(0);
        tenPidController.setReference(20, ControlType.kPosition);
        sixPidController.setReference(5, ControlType.kPosition);
        tenBrake.set(1);

        // PHASE 3 (15 point rung): extend the fifteen point arm and grab the bar, releasing the six point arm by extending it
        // and the ten point arm using a solenoid mechanism.
        fifteenMain.set(ControlMode.Position, 20);
        sixBrake.set(0);
        sixPidController.setReference(20, ControlType.kPosition);
        tenSolenoidMain.set(1);
    }
}
