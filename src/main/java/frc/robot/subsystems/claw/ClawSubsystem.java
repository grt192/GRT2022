package frc.robot.subsystems.claw;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Solenoid;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import static frc.robot.Constants.ClawConstants.*;

public class ClawSubsystem extends SubsystemBase {
  private TalonSRX rightMotor;
  private TalonSRX leftMotor;
  private Solenoid pfft;

  // Set initial state: claw open and not lifted
  public boolean clawIsOpen = true;
  public boolean clawIsLifted = false;

  public ClawSubsystem() {
    CommandScheduler.getInstance().registerSubsystem(this);

    // Initialize motors and pneumatic
    rightMotor = new TalonSRX(rightMotorPort);
    leftMotor = new TalonSRX(leftMotorPort);

    pfft = new Solenoid(pfftPCMPort);

    // Configure the motors for the encoder
    rightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    rightMotor.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    // If claw is meant to be open
    if (clawIsOpen) {

      // Set motor
      if (rightMotor.getSelectedSensorPosition() < clawOpenPosition) {
        rightMotor.set(ControlMode.PercentOutput, clawMotorSpeed);
        leftMotor.set(ControlMode.PercentOutput, -clawMotorSpeed);
      }
    } else {
      if (rightMotor.getSelectedSensorPosition() > 0) {
        rightMotor.set(ControlMode.PercentOutput, -clawMotorSpeed);
        leftMotor.set(ControlMode.PercentOutput, clawMotorSpeed);
      }
    }

    // If claw is meant to be lifted, turn pneumatic on
    if (clawIsLifted) {
      pfft.set(true);
    } else {
      // If not, turn pfft off
      pfft.set(false);
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
