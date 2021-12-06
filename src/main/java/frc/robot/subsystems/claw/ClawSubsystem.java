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

  // Position variables for the motors
  private double rightPos;
  private double leftPos;

  // Set initial state: claw open and not lifted
  public boolean clawIsOpen = true;
  public boolean clawIsLifted = false;

  public ClawSubsystem() {
    CommandScheduler.getInstance().registerSubsystem(this);

    // Initialize motors and pneumatic
    rightMotor = new TalonSRX(rightMotorPort);
    leftMotor = new TalonSRX(leftMotorPort);

    pfft = new Solenoid(pfftPCMPort);

    // Configure the motors for the encoders
    rightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    rightMotor.setSelectedSensorPosition(0);

    leftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    leftMotor.setSelectedSensorPosition(0);

    // Set initial motor position vars
    rightPos = 0;
    leftPos = 0;

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
      // If claw is meant to be closed; give motors power until motors stall
      if (!isMotorStalled()) {
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
   * Checks if the claw motors are not moving (ie. if change in position is less
   * than a small delta value).
   * 
   * @return true if stalled; false if moving
   */
  public boolean isMotorStalled() {
    double newRightPos = rightMotor.getSelectedSensorPosition();
    double newLeftPos = leftMotor.getSelectedSensorPosition();

    // If motors are barely changing position
    boolean isStalled = (Math.abs(newRightPos - rightPos) <= stallDelta)
        && (Math.abs(newLeftPos - leftPos) <= stallDelta);

    // Save new motor positions
    rightPos = newRightPos;
    leftPos = newLeftPos;

    return isStalled;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
