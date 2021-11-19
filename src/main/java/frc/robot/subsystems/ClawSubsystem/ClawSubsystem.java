package frc.robot.subsystems.ClawSubsystem;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Solenoid;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class ClawSubsystem extends SubsystemBase{
    private TalonSRX motor1;
    private Solenoid pfft1;

    public ClawSubsystem() {
        
      CommandScheduler.getInstance().registerSubsystem(this);

        motor1 = new TalonSRX(0);
        pfft1 = new Solenoid(1);
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
        motor1.set(ControlMode.PercentOutput, .5);
    }

    public void closeClaw(){
        motor1.set(ControlMode.PercentOutput, -.5);
    }

    public void liftClaw(){
        pfft1.set(true);
    }

    public void lowerClaw(){
        pfft1.set(false);
        motor1.set(ControlMode.PercentOutput, -.5);
    }
}

