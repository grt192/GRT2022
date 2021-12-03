package frc.robot.subsystems.ClawSubsystem;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Solenoid;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class ClawSubsystem extends SubsystemBase{
    private TalonSRX motor1;
    private Solenoid pfft1;

    public boolean clawIsOpen;
    public boolean clawIsLifted;

    private int max_pos;

    public ClawSubsystem() {
        
      CommandScheduler.getInstance().registerSubsystem(this);

        motor1 = new TalonSRX(0);
        pfft1 = new Solenoid(1);
        
        motor1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        motor1.setSelectedSensorPosition(0);
    }
    
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      if (clawIsOpen){
        if(motor1.getSelectedSensorPosition()< max_pos)
        motor1.set(ControlMode.PercentOutput, .5);
      }
      else{
        if (motor1.getSelectedSensorPosition() > 0){
          motor1.set(ControlMode.PercentOutput, -.5);
        } 
      }

      if(clawIsLifted){
        pfft1.set(true);
      }
      else{
        pfft1.set(false);
        motor1.set(ControlMode.PercentOutput, -.5);
      }
    }
  
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
}

