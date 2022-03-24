// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.brownout.PowerController;
import frc.robot.commands.intake.DeployIntakeCommand;
import frc.robot.commands.intake.RaiseIntakeCommand;
import frc.robot.commands.intake.RunIntakeCommand;
import frc.robot.commands.internals.RequestShotCommand;
import frc.robot.commands.tank.AutonRedBottomSequence;
import frc.robot.commands.tank.FollowPathCommand;
import frc.robot.jetson.JetsonConnection;
import frc.robot.subsystems.ClimbSubsystem;
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
    private final ClimbSubsystem climbSubsystem;

    private final JetsonConnection jetson;
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
        jetson = new JetsonConnection();
        
        // Instantiate subsystems
        tankSubsystem = new TankSubsystem();
        turretSubsystem = new TurretSubsystem(tankSubsystem, jetson);
        internalSubsystem = new InternalSubsystem(turretSubsystem);
        intakeSubsystem = new IntakeSubsystem(internalSubsystem, jetson);
        climbSubsystem = new ClimbSubsystem();

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

        // Configure the button bindings
        configureButtonBindings();

        // Set initial robot position
        // This is temporary; after shooter-testing is merged, each auton path should call this
        // in their constructor.
        double hubDist = 162;
        Pose2d initialPose = new Pose2d(Units.inchesToMeters(hubDist), 0, new Rotation2d());
        setInitialPosition(initialPose);

        // Instantiate auton command
        autonCommand = new AutonRedBottomSequence(this);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     * 
     * Controller 1 (driver):
     * Joysticks -> car drive -- left Y -> foward, right X -> angular (tank)
     * Triggers -> manual override intake -- left backwards, right forwards (intake)
     * A button -> lower intake (intake)
     * B button -> raise intake (intake)
     * 
     * Controller 2 (mechanisms):
     * A button -> request shot (internals)
     * X button -> start climb sequence (climb)
     */
    private void configureButtonBindings() {
        // driveAButton.whenPressed(new RequestShotCommand(internalSubsystem));
        // driveBButton.whenPressed(new DeployIntakeCommand(intakeSubsystem));
        // driveYButton.whenPressed(new RaiseIntakeCommand(intakeSubsystem));
        driveXButton.whenPressed(new InstantCommand(turretSubsystem::toggleClimb));

        mechAButton.whenPressed(new RequestShotCommand(internalSubsystem));
        mechBButton.whenPressed(new InstantCommand(intakeSubsystem::togglePosition));
        mechXButton.whenPressed(new InstantCommand(turretSubsystem::resetOffsets));
        mechYButton.whenPressed(new InstantCommand(turretSubsystem::toggleFreeze));

        // Car drive with the left Y axis controlling y power and the right X axis controlling angular
        tankSubsystem.setDefaultCommand(new RunCommand(() -> {
            // double turn_stick = driveController.getRightX();
            double turn = driveController.getRightX() * 0.75;
            tankSubsystem.setCarDrivePowers(-driveController.getLeftY(), turn);
        }, tankSubsystem));

        // Driver-override intake command
        intakeSubsystem.setDefaultCommand(new RunIntakeCommand(intakeSubsystem, mechController));

        // Set turret offsets from mech controller POV input:
        // Top/bottom to increase/decrease hub distance offset,
        // right/left to increase/decrease theta offset.
        turretSubsystem.setDefaultCommand(new RunCommand(() -> {
            switch (mechController.getPOV()) {
                case 0: turretSubsystem.changeDistanceOffset(2); break;
                case 90: turretSubsystem.changeTurntableOffset(Math.toRadians(2)); break;
                case 180: turretSubsystem.changeDistanceOffset(-2); break;
                case 270: turretSubsystem.changeTurntableOffset(Math.toRadians(-2)); break;
                default: break;
            }
        }, turretSubsystem));

        // Manual climb control with the right mech joystick:
        // Push up to extend, down to retract; brakes are automatically set when manual control 
        // is supplied.
        climbSubsystem.setDefaultCommand(new RunCommand(() -> {
            double pow = mechController.getRightY();
            climbSubsystem.setSixPower(pow);
            climbSubsystem.setSixBrake(pow == 0);
        }, climbSubsystem));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autonCommand;
    }

    /**
     * Sets the initial position of the robot. Wraps a call to reset localization
     * and set the initial `r` and `theta` of the turret.
     * @param position The initial position of the robot.
     */
    public void setInitialPosition(Pose2d position) {
        tankSubsystem.resetPosition(position);
        turretSubsystem.setInitialPose(position);
    }

    /**
     * Gets the PowerController instance for scaling in `Robot.periodic()`.
     * @return The PowerController instance.
     */
    public PowerController getPowerController() {
        return powerController;
    }

    public TankSubsystem getTankSubsystem() {
        return tankSubsystem;
    }

    public TurretSubsystem getTurretSubsystem() {
        return turretSubsystem;
    }

    public IntakeSubsystem getIntakeSubsystem() {
        return intakeSubsystem;
    }

    public InternalSubsystem getInternalSubsystem() {
        return internalSubsystem;
    }
}
