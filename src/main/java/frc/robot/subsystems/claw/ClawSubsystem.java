package frc.robot.subsystems.claw;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class ClawSubsystem extends SubsystemBase {

  private Servo servo0;
  private Servo servo1;
  private Servo servo2;
  private Servo servo3;

  private Servo[] servos;
  private double[] openAngles;
  private double[] closedAngles;

  private Solenoid pfft1;

  public boolean clawIsOpen = false;
  public boolean clawIsLifted = false;

  private final double openAngle0 = 0;
  private final double openAngle1 = 0;
  private final double openAngle2 = 0.95;
  private final double openAngle3 = 1;

  private final double closedAngle0 = 0.5;
  private final double closedAngle1 = 0.5;
  private final double closedAngle2 = 0.5;
  private final double closedAngle3 = 0;

  public ClawSubsystem() {

    CommandScheduler.getInstance().registerSubsystem(this);

    servo0 = new Servo(0);
    servo1 = new Servo(1);
    servo2 = new Servo(2);
    servo3 = new Servo(3);

    pfft1 = new Solenoid(4);

    servos = new Servo[] { servo0, servo1, servo2, servo3 };
    // servos = new Servo[]{servo1};
    openAngles = new double[] { openAngle0, openAngle1, openAngle2, openAngle3 };
    closedAngles = new double[] { closedAngle0, closedAngle1, closedAngle2, closedAngle3 };
  }

  @Override
  public void periodic() {
    for (int i = 0; i < servos.length; i++) {
      if (clawIsOpen) {
        servos[i].set(openAngles[i]);
      } else {
        servos[i].set(closedAngles[i]);
      }
    }

    if (clawIsLifted) {
      pfft1.set(true);
    } else {
      pfft1.set(false);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}