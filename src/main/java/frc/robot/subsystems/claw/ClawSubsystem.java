package frc.robot.subsystems.claw;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class ClawSubsystem extends SubsystemBase{

    private Servo servo1;
    private Servo servo2;
    private Servo servo3;
    private Servo servo4;

    private Servo[] servos;
    private int[] openAngles;
    private int[] closedAngles;

    private Solenoid pfft1; 

    public boolean clawIsOpen = false;
    public boolean clawIsLifted = false;

    public final int openAngle1 = 100;
    public final int openAngle2 = 100;
    public final int openAngle3 = 100;
    public final int openAngle4 = 100;

    public final int closedAngle1 = 0;
    public final int closedAngle2 = 0;
    public final int closedAngle3 = 0;
    public final int closedAngle4 = 0;


    public ClawSubsystem() {
        
        CommandScheduler.getInstance().registerSubsystem(this);
  
          servo1 = new Servo(0);
          servo2 = new Servo(1);
          servo3 = new Servo(2);
          servo4 = new Servo(3);

          pfft1 = new Solenoid(4);

          servos = new Servo[]{servo1, servo2, servo3, servo4};
          openAngles = new int[]{openAngle1, openAngle2, openAngle3, openAngle4};
          closedAngles = new int[]{closedAngle1, closedAngle2, closedAngle3, closedAngle4};
      }
      
      @Override
      public void periodic() {

        for (int i = 0; i < servos.length; i++){
	        if (clawIsOpen){
            servos[i].set(openAngles[i]);
          }
	        else{
	          servos[i].set(closedAngles[i]);
	        }
        } 
        
        if (clawIsLifted){
            pfft1.set(true);
        }
        else{
          pfft1.set(false);
          if (!clawIsOpen){
            for (int i = 0; i < servos.length; i++){
              servos[i].set(openAngles[i]);
            }
          }
        }
      }
    
      @Override
      public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
      }
}
