package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
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

  private final CANSparkMax flywheel;
  private final RelativeEncoder flywheelEncoder;

  private double lastUpdate = 0;
  private double lastPos = 0;

  private ShuffleboardTab sTab = Shuffleboard.getTab("shooter");
  private NetworkTableEntry targetPower = sTab.add("power", 0.8).getEntry();
  private NetworkTableEntry sPos = sTab.add("pos", 69).getEntry();
  private NetworkTableEntry sVelo = sTab.add("velo", 1337).getEntry();

  public ShooterSubsystem() {
    turntable = new WPI_TalonSRX(turntablePort);
    turntable.configFactoryDefault();
    turntable.setNeutralMode(NeutralMode.Brake);
    turntable.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);

    flywheel = new CANSparkMax(flywheelPort, MotorType.kBrushless);
    flywheel.restoreFactoryDefaults();
    flywheel.setIdleMode(IdleMode.kBrake);

    flywheelEncoder = flywheel.getEncoder();
    flywheelEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // TODO: implement vision tracking and turntable

    double time = Timer.getFPGATimestamp();
    double dTime = time - lastUpdate;

    double power = targetPower.getDouble(0.8);
    flywheel.set(power);

    double pos = flywheelEncoder.getPosition();
    sPos.setDouble(pos);

    double velo = (pos - lastPos) / dTime;
    sVelo.setDouble(velo);

    lastPos = pos;
  }

  /**
   * Shoots a ball from the flywheel shooter.
   */
  public void shoot() {
    // TODO: implement this
  }
}
