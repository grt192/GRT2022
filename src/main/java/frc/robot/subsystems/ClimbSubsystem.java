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
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import frc.robot.GRTSubsystem;
import frc.robot.shuffleboard.GRTNetworkTableEntry;
import frc.robot.shuffleboard.GRTShuffleboardTab;

import static frc.robot.Constants.ClimbConstants.*;

public class ClimbSubsystem extends SubsystemBase {
    private final CANSparkMax six;
    private final RelativeEncoder sixEncoder;
    private final SparkMaxPIDController sixPidController;
    private final WPI_TalonSRX sixBrake;

    /*
    private final CANSparkMax ten;
    private final RelativeEncoder tenEncoder;
    private final SparkMaxPIDController tenPidController;
    private final WPI_TalonSRX tenBrake;

    private final WPI_TalonSRX tenSolenoidMain;
    private final WPI_TalonSRX tenSolenoidFollow;

    private final WPI_TalonSRX fifteenMain;
    private final WPI_TalonSRX fifteenFollow;
    */

    // Six point arm smart motion PID constants
    private static final double sixP = 0;
    private static final double sixI = 0;
    private static final double sixD = 0;
    private static final double sixFF = 0.000125;
    private static final double maxVel = 150;
    private static final double maxAccel = 300;

    private static final double SIX_MAX_POS = 175;
    private static final double SIX_RETRACTED_POS = 50;

    // Ten point arm position PID constants
    private static final double tenP = 0.125;
    private static final double tenI = 0;
    private static final double tenD = 0;

    // Temp states for brake toggles
    private boolean sixBrakeEngaged = false;
    private boolean lastRetracted = false;
    private boolean retractToExtend = false;
    private boolean brakeLastEngaged = true;
    private Timer brakeSwitch;
    private boolean lowerLimit = true;


    // Fifteen point arm position PID constants
    private static final double fifteenP = 0.125;
    private static final double fifteenI = 0;
    private static final double fifteenD = 0;

    private final GRTShuffleboardTab shuffleboardTab;
    private final GRTNetworkTableEntry sixPosEntry;

    // Debug flags
    // Whether PID tuning shuffleboard entries should be displayed
    private static boolean DEBUG_PID = true;

    public ClimbSubsystem() {
        // Initialize six point arm NEO, encoder PID, and solenoid brake
        six = new CANSparkMax(sixMotorPort, MotorType.kBrushless);
        six.restoreFactoryDefaults();
        six.setIdleMode(IdleMode.kBrake);
        six.setInverted(true);
        brakeSwitch = new Timer();

        lastRetracted = true;

        six.setSoftLimit(SoftLimitDirection.kForward, (float) SIX_MAX_POS);
        six.enableSoftLimit(SoftLimitDirection.kForward, true);

        sixEncoder = six.getEncoder();
        sixEncoder.setPosition(0);

        sixPidController = six.getPIDController();
        sixPidController.setP(sixP);
        sixPidController.setI(sixI);
        sixPidController.setD(sixD);

        sixBrake = new WPI_TalonSRX(sixBrakePort);
        sixBrake.configFactoryDefault();

        // Initialize Shuffleboard entries
        shuffleboardTab = new GRTShuffleboardTab("Climb");
        sixPosEntry = shuffleboardTab.addEntry("Six pos", sixEncoder.getPosition());
        shuffleboardTab.addToggle("lower limit on", lowerLimit, this::setLowerLimit, 4, 0);


        // If DEBUG_PID is set, allow for PID tuning on shuffleboard
        if (DEBUG_PID) {
            shuffleboardTab
                .list("Six PID")
                .addListener("kP", sixP, this::setSixP)
                .addListener("kI", sixI, this::setSixI)
                .addListener("kD", sixD, this::setSixD)
                .addListener("kFF", sixFF, this::setSixFF)
                .addListener("Max vel", maxVel, this::setSixMaxVel)
                .addListener("Max accel", maxAccel, this::setSixMaxAccel);
        }
    }

    @Override
    public void periodic() {
        sixPosEntry.setValue(sixEncoder.getPosition());
    }

    /**
     * Manually sets the six winch power.
     * @param pow The power to set.
     */
    public void driveSixArm(double pow) {
        double power = pow;
        if (power < 0) {
            if (lowerLimit && sixEncoder.getPosition() <= 0) {
                setSixPower(0);
                lastRetracted = true;
                return;
            }
            lastRetracted = true;
        }

        if (power > 0 && sixEncoder.getPosition() >= SIX_MAX_POS) {
            setSixPower(0);
            return;
        }

        if (lastRetracted && brakeLastEngaged && (power > 0)) {
            System.out.println();
            brakeSwitch.start();
            lastRetracted = false;
            power = -0.07;
            retractToExtend = true;
        } else if (retractToExtend) {
            power = -0.07;
        }

        //stop retracting and start extending
        if (retractToExtend && brakeSwitch.hasElapsed(0.15)) {
            sixBrakeEngaged = false;
            brakeLastEngaged = false;
            retractToExtend = false;
            brakeSwitch.stop();
            brakeSwitch.reset();
        }

        // Set power and brake mode
        setSixPower(power);
    }

    /**
     * Sets the six arm power, automaticaly disengaging the brake when going up.
     * @param pow The power to set.
     */
    private void setSixPower(double pow) {
        six.set(pow);

        // Disengage the brake when the six motor is going up (positive power).
        sixBrake.set(pow > 0 ? 1 : 0);
    }

    /**
     * Manually sets the six brake status. When the brake is disengaged, 
     * the solenoid is powered (retracted).
     * @param brake Whether to engage the brake.
     */
    public void setSixBrake(boolean brake) {
        sixBrakeEngaged = brake;
        if (brake) {
            brakeLastEngaged = true;
        }
       // sixBrakeEngaged = !sixBrakeEngaged;
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
            sixPidController.setReference(SIX_MAX_POS, ControlType.kSmartMotion);
        }

        @Override
        public boolean isFinished() {
            return Math.abs(sixEncoder.getPosition() - SIX_MAX_POS) < 1;
        }

        @Override
        public void end(boolean interrupted) {
            // Engage the brake and partially retract the arm after grabbing
            sixBrake.set(1);
            sixPidController.setReference(SIX_RETRACTED_POS, ControlType.kSmartMotion);
        }
    }

    /**
     * Gets the climb sequence as a SequentialCommandGroup.
     * @return The command group representing the climb sequence.
     */
    public Command climb(GRTSubsystem... subsystems) {
        for (GRTSubsystem subsystem : subsystems) subsystem.climbInit();
        return new ClimbPhase1Command();
    }

    /**
     * Climb PID tuning NetworkTable callbacks.
     * @param change The `EntryNotification` representing the NetworkTable entry change.
     */
    private void setSixP(EntryNotification change) {
        sixPidController.setP(change.value.getDouble());
    }

    private void setSixI(EntryNotification change) {
        sixPidController.setI(change.value.getDouble());
    }

    private void setSixD(EntryNotification change) {
        sixPidController.setD(change.value.getDouble());
    }

    private void setSixFF(EntryNotification change) {
        sixPidController.setFF(change.value.getDouble());
    }

    private void setSixMaxVel(EntryNotification change) {
        sixPidController.setSmartMotionMaxVelocity(change.value.getDouble(), 0);
    }

    private void setSixMaxAccel(EntryNotification change) {
        sixPidController.setSmartMotionMaxAccel(change.value.getDouble(), 0);
    }

    private void setLowerLimit(EntryNotification change) {
        lowerLimit = change.value.getBoolean();
    }
    
}
