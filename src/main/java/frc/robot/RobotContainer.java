// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.List;
import java.util.Properties;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.brownout.PowerController;
import frc.robot.commands.tank.FollowPathCommand;
import frc.robot.subsystems.shooter.ShooterSubsystem;
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
  //private Properties config;

  // Subsystems
  private final TankSubsystem tankSubsystem;
  private final ShooterSubsystem shooterSubsystem;

  private final PowerController powerController;

  // Controllers
  private final XboxController controlXbox = new XboxController(0);
  private final JoystickButton xboxAButton = new JoystickButton(controlXbox, XboxController.Button.kA.value);
  private final JoystickButton xboxXButton = new JoystickButton(controlXbox, XboxController.Button.kX.value);

  // Commands
  private final Command tankCommand;

  public boolean runJetson = true;

  public boolean runJetson = true;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Load the config file
    /*
    this.config = new Properties();

    try {
      FileInputStream stream = new FileInputStream(new File(Filesystem.getDeployDirectory(), CONFIG_PATH));
      config.load(stream);
    } catch (IOException ie) {
      System.out.println("Config file not found");
    }
    */

    // Instantiate subsystems
    tankSubsystem = new TankSubsystem();
    shooterSubsystem = new ShooterSubsystem();

    powerController = new PowerController(tankSubsystem);

    // Instantiate commands
    // Drive an S-shaped curve from the origin to 3 meters in front through 2 waypoints
    tankCommand = new FollowPathCommand(
      tankSubsystem, 
      new Pose2d(), 
      List.of(
        new Translation2d(1, 1), 
        new Translation2d(2, -1)
      ), 
      new Pose2d(3, 0, new Rotation2d())
    );
    /*
    new FollowPathCommand(tankSubsystem, new Pose2d(), List.of(), new Pose2d(3, 0, new Rotation2d()))
      .andThen(new InstantCommand(() -> tankSubsystem.setTankDriveVoltages(0, 0)));
    */

    // Configure the button bindings
    configureButtonBindings();

    // Configure the Jetson and run it
    JetsonConnection jetsonObj = new JetsonConnection();
    Runnable jetson = () -> {

      while (runJetson) {
        jetsonObj.periodic();

        try {
          Thread.sleep(1000);
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }
      // Pause the thread so it doesn't run like crazy
      // try {
      // Thread.sleep(1000);
      // } catch (InterruptedException e) {
      // e.printStackTrace();
      // }
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
    // A button -> zero odometry readings
    // X button -> shoot ball from shooter
    xboxAButton.whenPressed(new InstantCommand(tankSubsystem::resetPosition));
    xboxXButton.whenPressed(new InstantCommand(shooterSubsystem::shoot));

    Runnable tank = () -> {
      tankSubsystem.setCarDrivePowers(-controlXbox.getLeftY(), controlXbox.getRightX());
    };
    tankSubsystem.setDefaultCommand(new RunCommand(tank, tankSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return tankCommand;
  }

  /**
   * Gets the PowerController instance.
   * @return The PowerController instance.
   */
  public PowerController getPowerController() {
    return powerController;
  }
}
