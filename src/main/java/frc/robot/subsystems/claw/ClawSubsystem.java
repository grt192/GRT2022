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

  // Right and left motor powers -- state variables
  private double rightMotorPower;
  private double leftMotorPower;

  // State variable for whether the pneumatic is lifted or not
  private boolean liftClaw;

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

    // Set initial motor position vars (for checking if motors are stalled)
    rightPos = 0;
    leftPos = 0;

    liftClaw = false;

  }

  @Override
  public void periodic() {

    // Set motor powers
    rightMotor.set(ControlMode.PercentOutput, rightMotorPower);
    leftMotor.set(ControlMode.PercentOutput, leftMotorPower);

    // Set pneumatic
    pfft.set(liftClaw);
  }

  /**
   * Set motor powers to open.
   */
  public void setOpenPowers() {
    rightMotorPower = rightOpenPower;
    leftMotorPower = leftOpenPower;
  }

  /**
   * Set motor powers to close.
   */
  public void setClosePowers() {
    rightMotorPower = -rightOpenPower;
    leftMotorPower = -leftOpenPower;
  }

  /**
   * Set motor powers to zero.
   */
  public void setNeutralPowers() {
    rightMotorPower = 0;
    leftMotorPower = 0;
  }

  public boolean getClawLift() {
    return liftClaw;
  }

  public void setClawLift(boolean liftClaw) {
    this.liftClaw = liftClaw;
  }

  /**
   * Checks if the claw motors are not moving (ie. if change in position is less
   * than a small delta value).
   * 
   * @return true if stalled; false if moving
   */
  public boolean isMotorStalled(boolean right) {
    double newRightPos = rightMotor.getSelectedSensorPosition();
    double newLeftPos = leftMotor.getSelectedSensorPosition();

    // If motors are barely changing position
    boolean isRightStalled = Math.abs(newRightPos - rightPos) <= stallDelta;
    boolean isLeftStalled = Math.abs(newLeftPos - leftPos) <= stallDelta;

    // Save new motor positions
    rightPos = newRightPos;
    leftPos = newLeftPos;

    return right ? isRightStalled : isLeftStalled;
  }

  /**
   * Returns true if both motors are stalled.
   * 
   * @return true if both are stalled
   */
  public boolean areBothMotorsStalled() {
    return isMotorStalled(true) && isMotorStalled(false);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
