package frc.robot.subsystems.shooter;

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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ShooterConstants.*;

public class ShooterSubsystem extends SubsystemBase {
  private final WPI_TalonSRX turntable;

  private final CANSparkMax hood;
  private final RelativeEncoder hoodEncoder;
  private final SparkMaxPIDController hoodPidController;

  private final CANSparkMax flywheel;
  private final RelativeEncoder flywheelEncoder;
  private final SparkMaxPIDController flywheelPidController;

  private final double kP = 0.125;
  private final double kI = 0;
  private final double kD = 0;

  private boolean isOpen = false;
  private double lastUpdate = 0;
  private double lastPos = 0;

  private ShuffleboardTab sTab = Shuffleboard.getTab("shooter");
  private NetworkTableEntry targetPower = sTab.add("power", 0.8).getEntry();
  private NetworkTableEntry sPos = sTab.add("pos", 69).getEntry();
  private NetworkTableEntry sVelo = sTab.add("velo", 1337).getEntry();

  public ShooterSubsystem() {
    // Initialize turntable Talon and encoder PID
    turntable = new WPI_TalonSRX(turntablePort);
    turntable.configFactoryDefault();
    turntable.setNeutralMode(NeutralMode.Brake);

    turntable.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    turntable.setSensorPhase(false);
    turntable.config_kP(0, kP);
    turntable.config_kI(0, kI);
    turntable.config_kD(0, kD);

    // Initialize hood SparkMax and encoder PID
    hood = new CANSparkMax(hoodPort, MotorType.kBrushless);
    hood.restoreFactoryDefaults();
    hood.setIdleMode(IdleMode.kBrake);

    hoodEncoder = hood.getEncoder();
    hoodEncoder.setPosition(0);

    hoodPidController = hood.getPIDController();
    hoodPidController.setP(kP);
    hoodPidController.setI(kI);
    hoodPidController.setD(kD);

    // Initialize flywheel SparkMax and encoder PID
    flywheel = new CANSparkMax(flywheelPort, MotorType.kBrushless);
    flywheel.restoreFactoryDefaults();
    //flywheel.setIdleMode(IdleMode.kBrake);

    flywheelEncoder = flywheel.getEncoder();
    flywheelEncoder.setPosition(0);

    flywheelPidController = flywheel.getPIDController();
    flywheelPidController.setP(kP);
    flywheelPidController.setI(kI);
    flywheelPidController.setD(kD);
  }

  @Override
  public void periodic() {
    // TODO: implement vision tracking and turntable
  }

  /**
   * Shoots a ball from the flywheel shooter.
   * 
   * Currently, this has been co-opted by a PID testing function.
   * For the turntable and hood, this will toggle the position closed loop between a negative and positive value.
   * For the flywheel, this will toggle the velocity closed loop between two speeds.
   */
  public void shoot() {
    // TODO: implement this

    System.out.println(isOpen);
    //hoodPidController.setReference(isOpen ? 5 : -5, ControlType.kPosition);
    flywheelPidController.setReference(isOpen ? 30 : 60, ControlType.kVelocity);
    //turntable.set(ControlMode.Position, isOpen ? 1000 : -1000);
    isOpen = !isOpen;
  }

  /**
   * A test function to see if the plugged in motor works (and to spin it for position PID testing).
   * This will toggle the motor between 50% and 0% output.
   */
  public void test() {
    System.out.println(isOpen);
    //hood.set(!isOpen ? 0.5 : 0);
    flywheel.set(!isOpen ? 0.5 : 0);
    //turntable.set(ControlMode.PercentOutput, !isOpen ? 0.5 : 0);
    isOpen = !isOpen;
  }
}
