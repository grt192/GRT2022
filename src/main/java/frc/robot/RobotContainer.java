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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.brownout.PowerController;
import frc.robot.commands.intake.DeployIntakeCommand;
import frc.robot.commands.intake.RaiseIntakeCommand;
import frc.robot.commands.internals.RequestShotCommand;
import frc.robot.commands.tank.FollowPathCommand;
import frc.robot.jetson.JetsonConnection;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.InternalSubsystem;
import frc.robot.subsystems.TankSubsystem;

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
    private final TurretSubsystem turretSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final InternalSubsystem internalSubsystem;
    private final ClimbSubsystem climbSubsystem;

    private final JetsonConnection jetson;
    private final PowerController powerController;

    // Controllers
    private final XboxController driveController = new XboxController(0);
    private final JoystickButton driveAButton = new JoystickButton(driveController, XboxController.Button.kA.value);
    private final JoystickButton driveBButton = new JoystickButton(driveController, XboxController.Button.kB.value);

    private final XboxController mechController = new XboxController(1);
    private final JoystickButton mechAButton = new JoystickButton(mechController, XboxController.Button.kA.value);
    private final JoystickButton mechXButton = new JoystickButton(mechController, XboxController.Button.kX.value);

    // Commands
    private final Command tankCommand;

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

        // Instantiate the Jetson connection
        jetson = new JetsonConnection();

        // Instantiate subsystems
        tankSubsystem = new TankSubsystem();
        turretSubsystem = new TurretSubsystem(jetson);
        internalSubsystem = new InternalSubsystem(turretSubsystem);
        intakeSubsystem = new IntakeSubsystem(internalSubsystem, jetson);
        climbSubsystem = new ClimbSubsystem();

        // Instantiate power controller
        powerController = new PowerController(
            tankSubsystem, 
            turretSubsystem, 
            internalSubsystem, 
            intakeSubsystem, 
            climbSubsystem
        );

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
     * TODO: finalize these, talk to drivers
     * 
     * Controller 1 (driver):
     * Joysticks -> car drive (tank)
     * A button -> lower intake (intake)
     * B button -> raise intake (intake)
     * 
     * Controller 2 (mechanisms):
     * A button -> request shot (internals)
     * X button -> start climb sequence (climb)
     */
    private void controllerBindings() {
        driveAButton.whenPressed(new DeployIntakeCommand(intakeSubsystem));
        driveBButton.whenPressed(new RaiseIntakeCommand(intakeSubsystem));

        mechAButton.whenPressed(new RequestShotCommand(internalSubsystem));
        mechXButton.whenPressed(climbSubsystem.climb());

        Runnable tank = () -> tankSubsystem.setCarDrivePowers(-driveController.getLeftY(), driveController.getRightX());
        tankSubsystem.setDefaultCommand(new RunCommand(tank, tankSubsystem));

        // TODO: tune deadband
        Runnable intake = () -> intakeSubsystem.setDriverInput(driveController.getLeftTriggerAxis() > 0.1);
        intakeSubsystem.setDefaultCommand(new RunCommand(intake, intakeSubsystem));
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
