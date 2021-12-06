package frc.robot.subsystems.claw;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class ClawSubsystem extends SubsystemBase {

  private Servo[] servos;

  // Open position constants corresponding to servos [0, 1, 2, 3]
  private final double[] openAngles = {
    0, 0, 0.95, 1
  };
  // Closed position constants corresponding to servos [0, 1, 2, 3]
  private final double[] closedAngles = {
    0.5, 0.5, 0.5, 0
  };

  private Solenoid pfft1;

  public boolean clawIsOpen = false;
  public boolean clawIsLifted = false;

  public ClawSubsystem() {
    pfft1 = new Solenoid(4);

    servos = new Servo[] { 
      new Servo(0), new Servo(1), 
      new Servo(2), new Servo(3) 
    };
  }

  @Override
  public void periodic() {
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
}
