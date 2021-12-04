package frc.robot.subsystems.claw;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Solenoid;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import static frc.robot.Constants.ClawConstants.*;

public class ClawSubsystem extends SubsystemBase {
  private TalonSRX motor1;
  private TalonSRX motor2;
  private Solenoid pfft1;

  public boolean clawIsOpen = false;
  public boolean clawIsLifted = false;

  public ClawSubsystem() {

    CommandScheduler.getInstance().registerSubsystem(this);

    motor1 = new TalonSRX(motor1Port);
    motor2 = new TalonSRX(motor2Port);

    pfft1 = new Solenoid(pfftPCMPort);

    motor1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    motor1.setSelectedSensorPosition(0);

    motor2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    motor2.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    // If claw is meant to be open
    if (clawIsOpen) {

      // Set motor
      if (motor1.getSelectedSensorPosition() < clawOpenPosition) {
        motor1.set(ControlMode.PercentOutput, clawMotorSpeed);
      }

      if (motor2.getSelectedSensorPosition() < clawOpenPosition) {
        motor2.set(ControlMode.PercentOutput, -clawMotorSpeed);
      }
    } else {
      if (motor1.getSelectedSensorPosition() > 0) {
        motor1.set(ControlMode.PercentOutput, -clawMotorSpeed);
      }
      if (motor2.getSelectedSensorPosition() > 0) {
        motor2.set(ControlMode.PercentOutput, clawMotorSpeed);
      }
    }

    // If claw is meant to be lifted, trigger pneumatic
    if (clawIsLifted) {
      pfft1.set(true);
    } else {
      pfft1.set(false);
      // motor1.set(ControlMode.PercentOutput, -.5);
    }
  }

  /**
   * Closes and lifts the claw simultaneously.
   */
  public void closeAndLiftCommand() {
    clawIsOpen = false;
    clawIsLifted = true;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
