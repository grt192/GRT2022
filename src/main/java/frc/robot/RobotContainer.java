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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.tank.DriveTankCommand;
import frc.robot.subsystems.tank.TankSubsystem;

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

  // Controllers
  private XboxController controlXbox = new XboxController(0);

  // Joysticks
  private Joystick joystickLeft = new Joystick(1);
  private Joystick joystickRight = new Joystick(2);

  // Commands
  private final DriveTankCommand tankCommand;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Load the config file
    this.config = new Properties();

    try {
      FileInputStream stream = new FileInputStream(new File(Filesystem.getDeployDirectory(), CONFIG_PATH));
      config.load(stream);
    } catch (IOException ie) {
      System.out.println("Config file not found");
    }

    // Instantiate subsystems
    tankSubsystem = new TankSubsystem();

    // Instantiate commands
    tankCommand = new DriveTankCommand(tankSubsystem, 0, 0);

    // Configure the button bindings
    configureButtonBindings();

    // Configure the Jetson and run it
    JetsonConnection jetsonObj = new JetsonConnection();
    Runnable jetson = () -> {
      while (true) {
        jetsonObj.periodic();

        try {
          Thread.sleep(1000);
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }
    };

    Thread jetsonThread = new Thread(jetson);
    jetsonThread.start();
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
      boolean isXbox = (Math.abs(controlXbox.getLeftY())
          + Math.abs(controlXbox.getRightX())) > (Math.abs(joystickLeft.getY()) + Math.abs(joystickRight.getY()));

      // Set the drive powers based on which controller is being used
      if (isXbox) {
        tankSubsystem.setCarDrivePowers(-controlXbox.getLeftY(), controlXbox.getRightX());
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
