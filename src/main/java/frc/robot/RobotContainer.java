// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.tank.DriveTankCommand;
import frc.robot.commands.elevator.ElevatorUpCommand;
import frc.robot.commands.elevator.ElevatorDownCommand;
import frc.robot.commands.elevator.ElevatorStopCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.tank.TankSubsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final TankSubsystem tankSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;

  // Controllers
  private XboxController controlXbox = new XboxController(0);

  // Commands
  private final DriveTankCommand tankCommand;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   * 
   */
  public RobotContainer() {

    // Instantiate subsystems
    tankSubsystem = new TankSubsystem();

    elevatorSubsystem = new ElevatorSubsystem();

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

    // when A is pressed, move elevator Up
    new JoystickButton(controlXbox, 1).whenPressed(new ElevatorUpCommand(elevatorSubsystem));

    // when B is pressed, move elevator down
    new JoystickButton(controlXbox, 2).whenPressed(new ElevatorDownCommand(elevatorSubsystem));

    // X stop
    new JoystickButton(controlXbox, 3).whenPressed(new ElevatorStopCommand(elevatorSubsystem));

    Runnable tank = () -> {
      tankSubsystem.setDrivePowers(-controlXbox.getY(Hand.kLeft), controlXbox.getX(Hand.kRight));

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
