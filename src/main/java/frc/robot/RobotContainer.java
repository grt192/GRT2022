// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.Properties;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.tank.DriveTankCommand;
import frc.robot.commands.elevator.ElevatorUpCommand;
import frc.robot.commands.elevator.ElevatorDownCommand;
import frc.robot.commands.elevator.ElevatorStopCommand;
import frc.robot.subsystems.ClawSubsystem.ClawSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.tank.TankSubsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Path of config file, relative to the deploy folder
  private static final String CONFIG_PATH = "config.txt";
  // config file
  private Properties config;

  // Subsystems
  private final TankSubsystem tankSubsystem;
  private final ClawSubsystem clawSubsystem;

  // Controllers
  private XboxController controlXbox = new XboxController(0);

  // Joysticks
  private Joystick joystickLeft = new Joystick(1);
  private Joystick joystickRight = new Joystick(2);

  // Commands
  private final DriveTankCommand tankCommand;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   * 
   */
  public RobotContainer() {

    // Load the config file
    this.config = new Properties();

    try {
      FileInputStream stream = new FileInputStream(new File(Filesystem.getDeployDirectory(), CONFIG_PATH));
      config.load(stream);
    } catch (IOException ie) {
      System.out.println("config file not found");
    }

    // Instantiate subsystems
    tankSubsystem = new TankSubsystem();
    clawSubsystem = new ClawSubsystem();

    // Instantiate commands
    tankCommand = new DriveTankCommand(tankSubsystem, 0, 0);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    controllerBindings();
  }

  private void controllerBindings() {
    Runnable tank = () -> {

      // Check which controller is being used
      boolean isXbox = (Math.abs(controlXbox.getY(Hand.kLeft))
          + Math.abs(controlXbox.getX(Hand.kRight))) > (Math.abs(joystickLeft.getY()) + Math.abs(joystickRight.getY()));

      // Set the drive powers based on which controller is being used
      if (isXbox) {
        tankSubsystem.setCarDrivePowers(-controlXbox.getY(Hand.kLeft), controlXbox.getX(Hand.kRight));
        if(controlXbox.getAButtonPressed()){
          clawSubsystem.openClaw();
        }
        if(controlXbox.getBButtonPressed()){
          clawSubsystem.openClaw();
        }
        if(controlXbox.getXButtonPressed()){
          clawSubsystem.liftClaw();
        }
        if(controlXbox.getYButtonPressed()){
          clawSubsystem.lowerClaw();
        }
      } else {
        tankSubsystem.setTankDrivePowers(-joystickLeft.getY(), -joystickRight.getY());
      }
    };
    tankSubsystem.setDefaultCommand(new RunCommand(tank, tankSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return tankCommand;
  }
}
