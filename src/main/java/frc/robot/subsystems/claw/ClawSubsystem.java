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

    private Solenoid pfft1; 

    public ClawSubsystem() {
        
        CommandScheduler.getInstance().registerSubsystem(this);
  
          servo1 = new Servo(0);
          servo2 = new Servo(1);
          servo3 = new Servo(2);
          servo4 = new Servo(3);
          pfft1 = new Solenoid(4);

          servos = new Servo[]{servo1, servo2, servo3, servo4};
      }
      
      @Override
      public void periodic() {
        // This method will be called once per scheduler run
  
      }
    
      @Override
      public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
      }
  
      public void openClaw(){
          for (Servo s:servos){
            s.setAngle(45);
          }    
      }
  
      public void closeClaw(){
        for (Servo s:servos){
          s.setAngle(0);
        }   
      }
  
      public void liftClaw(){
          pfft1.set(true);
      }
  
      public void lowerClaw(){
          pfft1.set(false);
          closeClaw();
      }
}
