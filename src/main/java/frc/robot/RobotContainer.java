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
    private final TurretSubsystem turretSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final InternalSubsystem internalSubsystem;
    // private final ClimbSubsystem climbSubsystem;

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

        // Instantiate the Jetson connection
        // jetson = new JetsonConnection();
        // jetson.run();

        // Instantiate subsystems
        tankSubsystem = new TankSubsystem();
        turretSubsystem = new TurretSubsystem(tankSubsystem, jetson);
        internalSubsystem = new InternalSubsystem(turretSubsystem);
        intakeSubsystem = new IntakeSubsystem(internalSubsystem);
        // climbSubsystem = new ClimbSubsystem();

        // Instantiate power controller
        /*
        powerController = new PowerController(
            tankSubsystem,
            turretSubsystem,
            internalSubsystem,
            intakeSubsystem
            //, climbSubsystem
        );
        */

        // Instantiate commands
        // Drive an S-shaped curve from the origin to 3 meters in front through 2 waypoints
        if (tankSubsystem != null) {
            autonCommand = new FollowPathCommand(
                tankSubsystem,
                new Pose2d(),
                List.of(
                    new Translation2d(1, 1),
                    new Translation2d(2, -1)
                ),
                new Pose2d(3, 0, new Rotation2d())
            );
        } else {
            autonCommand = new InstantCommand();
        }

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
     * Left trigger -> run intake (intake)
     * A button -> lower intake (intake)
     * B button -> raise intake (intake)
     * 
     * Controller 2 (mechanisms):
     * A button -> request shot (internals)
     * X button -> start climb sequence (climb)
     */
    private void controllerBindings() {
        //driveAButton.whenPressed(new DeployIntakeCommand(intakeSubsystem));
        //driveBButton.whenPressed(new RaiseIntakeCommand(intakeSubsystem));

        mechAButton.whenPressed(new RequestShotCommand(internalSubsystem));
        mechXButton.whenPressed(new InstantCommand(() -> internalSubsystem.setPower(0)));

        mechYButton.whenPressed(new InstantCommand(() -> turretSubsystem.setFlywheelPower(0.2)));
        mechBButton.whenPressed(new InstantCommand(() -> turretSubsystem.setFlywheelPower(0)));

        if (tankSubsystem != null) {
            Runnable tank = () -> tankSubsystem.setCarDrivePowers(-driveController.getLeftY(), driveController.getRightX());
            tankSubsystem.setDefaultCommand(new RunCommand(tank, tankSubsystem));
        }

        if (intakeSubsystem != null) {
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

        if (internalSubsystem != null) {
            internalSubsystem.setDefaultCommand(new RunCommand(() -> {
                double leftTrigger = mechController.getLeftTriggerAxis();
                double rightTrigger = mechController.getRightTriggerAxis();

                if (leftTrigger != 0 || rightTrigger != 0) {
                    internalSubsystem.setDriverOverride(true);
                    internalSubsystem.setPower(leftTrigger - rightTrigger);
                } else {
                    internalSubsystem.setDriverOverride(false);
                }
            }, internalSubsystem));
        }

        if (turretSubsystem != null) {
            driveXButton.whileHeld(new StartEndCommand(
                () -> turretSubsystem.spin = true, 
                () -> turretSubsystem.spin = false, 
                turretSubsystem));

            turretSubsystem.setDefaultCommand(new RunCommand(() -> {
                double turntablePower = -mechController.getLeftY();
                double hoodPower = -mechController.getRightY();

                //System.out.println("Turntable power: " + turntablePower);
                //System.out.println("Hood power: " + hoodPower);

                turretSubsystem.setTurntablePower(turntablePower);
                turretSubsystem.setHoodPower(hoodPower);
            }, turretSubsystem));
        }
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * 
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autonCommand;
    }

    /**
     * Gets the PowerController instance.
     * 
     * @return The PowerController instance.
     */
    public PowerController getPowerController() {
        return powerController;
    }
}
