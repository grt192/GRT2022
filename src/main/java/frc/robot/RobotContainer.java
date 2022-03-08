// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.brownout.PowerController;
import frc.robot.commands.internals.RequestShotCommand;
import frc.robot.commands.tank.FollowPathCommand;
import frc.robot.jetson.JetsonConnection;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.internals.InternalSubsystem;
import frc.robot.subsystems.tank.TankSubsystem;

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
    private final TurretSubsystem turretSubsystem = null;
    private final IntakeSubsystem intakeSubsystem;
    private final InternalSubsystem internalSubsystem = null;

    private final JetsonConnection jetson = null;
    private final PowerController powerController = null;

    // Controllers and buttons
    private final XboxController driveController = new XboxController(0);
    private final JoystickButton 
        driveAButton = new JoystickButton(driveController, XboxController.Button.kA.value),
        driveBButton = new JoystickButton(driveController, XboxController.Button.kB.value),
        driveXButton = new JoystickButton(driveController, XboxController.Button.kX.value),
        driveYButton = new JoystickButton(driveController, XboxController.Button.kY.value);

    private final XboxController mechController = new XboxController(1);
    private final JoystickButton 
        mechAButton = new JoystickButton(mechController, XboxController.Button.kA.value),
        mechBButton = new JoystickButton(mechController, XboxController.Button.kB.value),
        mechXButton = new JoystickButton(mechController, XboxController.Button.kX.value),
        mechYButton = new JoystickButton(mechController, XboxController.Button.kY.value);

    // Commands
    private final Command autonCommand;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Instantiate subsystems
        tankSubsystem = new TankSubsystem();
        intakeSubsystem = new IntakeSubsystem(internalSubsystem);

        // Instantiate commands
        autonCommand = new InstantCommand();

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

    /**
     * Controller 1 (driver):
     * Joysticks -> car drive: left Y -> foward, right X -> angular (tank)
     * Triggers -> intake: left backwards, right forwards (intake)
     */
    private void controllerBindings() {
        Runnable tank = () -> tankSubsystem.setCarDrivePowers(-driveController.getLeftY(), driveController.getRightX());
        tankSubsystem.setDefaultCommand(new RunCommand(tank, tankSubsystem));

        // TODO: tune deadband
        Runnable intake = () -> {
            intakeSubsystem.setIntakePower(driveController.getRightTriggerAxis() - driveController.getLeftTriggerAxis());

            double deployPow = 0;
            if (driveController.getPOV() == 90) {
                deployPow = 0.2;
            } else if (driveController.getPOV() == 270) {
                deployPow = -0.2;
            }
            intakeSubsystem.setDeployPower(deployPow);
        };
        intakeSubsystem.setDefaultCommand(new RunCommand(intake, intakeSubsystem));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autonCommand;
    }

    /**
     * Gets the PowerController instance for scaling in `Robot.periodic()`.
     * @return The PowerController instance.
     */
    public PowerController getPowerController() {
        return powerController;
    }

    /**
     * Gets the InternalSubsystem for setting initial ball count in `Robot.autonomousInit()`.
     * @return The InternalSubsystem instance.
     */
    public InternalSubsystem getInternalSubsystem() {
        return internalSubsystem;
    }
}
