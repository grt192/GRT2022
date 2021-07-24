// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;


/** Add your docs here. */
public class ElevatorSubsystem extends SubsystemBase {
  private TalonSRX mainMotor;
  private TalonSRX followMotor;
  
  // limit switch variables
  DigitalInput topLimitSwitch;
  DigitalInput bottomLimitSwitch;

  public ElevatorSubsystem(int mainId, int followId) {
  
    CommandScheduler.getInstance().registerSubsystem(this);

    //motor config??
    mainMotor = new TalonSRX(mainId);   
    mainMotor.setNeutralMode(NeutralMode.Brake);
    mainMotor.setInverted(true);

    followMotor = new TalonSRX(followId);
    followMotor.setNeutralMode(NeutralMode.Brake);
    followMotor.follow(mainMotor);
    followMotor.setInverted(InvertType.FollowMaster);

    //limit switch config
    topLimitSwitch = new DigitalInput(0);
    bottomLimitSwitch = new DigitalInput(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void elevatorDownCommand() {
    mainMotor.set(ControlMode.PercentOutput,-1);
  }
  
  public void elevatorUpCommand() {
    mainMotor.set(ControlMode.PercentOutput,1);
  }

  public void elevatorStopCommand(){
    mainMotor.set(ControlMode.PercentOutput, 0);
  }
}
