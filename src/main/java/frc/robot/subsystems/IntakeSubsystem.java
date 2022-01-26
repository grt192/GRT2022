package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.jetson.JetsonConnection;
import static frc.robot.Constants.IntakeConstants.*;

public class IntakeSubsystem extends SubsystemBase {
  private final JetsonConnection jetson;

  private final CANSparkMax intake;
  private final WPI_TalonSRX deploy;

  private boolean isDeployed = true;

  public IntakeSubsystem(JetsonConnection jetson) {
    this.jetson = jetson;

    intake = new CANSparkMax(intakePort, MotorType.kBrushless);
    intake.restoreFactoryDefaults();

    deploy = new WPI_TalonSRX(deploymentPort);
    deploy.configFactoryDefault();
    deploy.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    intake.set(jetson.ballDetected() ? 0.5 : 0);
  }

  /**
   * Toggle whether the intake is deployed.
   */
  public void toggle() {
    isDeployed = !isDeployed;
    // TODO: measure this position
    deploy.set(ControlMode.Position, isDeployed ? 5 : 0);
  }
}
