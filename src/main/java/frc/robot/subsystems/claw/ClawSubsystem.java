package frc.robot.subsystems.claw;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;

import com.ctre.phoenix.motorcontrol.ControlMode;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class ClawSubsystem extends SubsystemBase {

  private Servo[] servos;

  // Open position constants corresponding to servos [0, 1, 2, 3]
  private final double[] openAngles = {
    0.5, 0.5, 0.5, 0.5
  };
  // Closed position constants corresponding to servos [0, 1, 2, 3]
  private final double[] closedAngles = {
    0.15, 0.85, 0.15, 0.85
  };

  private Solenoid pfft1;

  public boolean clawIsOpen = true;
  public boolean clawIsLifted = false;

  public ClawSubsystem() {
    pfft1 = new Solenoid(1);

    servos = new Servo[] { 
      new Servo(0), new Servo(1), 
      new Servo(2), new Servo(3) 
    };
  }

  @Override
  public void periodic() {
    // Set each servo to its corresponding open or closed position
    for (int i = 0; i < servos.length; i++) {
      servos[i].set(clawIsOpen
        ? openAngles[i]
        : closedAngles[i]);
    }

    pfft1.set(clawIsLifted);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /**
   * Toggle whether the claw is open
   */
  public void toggleIsOpen() {
    clawIsOpen = !clawIsOpen;
  }

  /**
   * Toggle whether the pneumatic is lifted
   */
  public void toggleIsLifted() {
    clawIsLifted = !clawIsLifted;
  }
}
