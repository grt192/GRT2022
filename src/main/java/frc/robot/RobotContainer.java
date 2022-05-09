// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.brownout.PowerController;
import frc.robot.commands.intake.DeployIntakeCommand;
import frc.robot.commands.intake.RaiseIntakeCommand;
import frc.robot.commands.intake.RunIntakeCommand;
import frc.robot.commands.internals.OverrideInternalsCommand;
import frc.robot.commands.internals.RequestShotCommand;
import frc.robot.commands.tank.AutonBlueBottomSequence;
import frc.robot.commands.tank.AutonBlueMiddleSequence;
import frc.robot.commands.tank.AutonBlueTopSequence;
import frc.robot.commands.tank.AutonRedBottomSequence;
import frc.robot.commands.tank.AutonRedMiddleSequence;
import frc.robot.commands.tank.AutonRedTopSequence;
import frc.robot.commands.tank.PlebAutonSequence;
import frc.robot.jetson.JetsonConnection;
import frc.robot.shuffleboard.GRTShuffleboardTab;
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
    private final PowerDistribution powerDistribution;

    // Subsystems
    private final TankSubsystem tankSubsystem;
    private final TurretSubsystem turretSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final InternalSubsystem internalSubsystem;
    private final ClimbSubsystem climbSubsystem;

    private final JetsonConnection jetson;
    private final PowerController powerController;

    // Controllers and buttons
    private final XboxController driveController = new XboxController(0);
    private final JoystickButton 
        driveAButton = new JoystickButton(driveController, XboxController.Button.kA.value),
        driveBButton = new JoystickButton(driveController, XboxController.Button.kB.value),
        driveXButton = new JoystickButton(driveController, XboxController.Button.kX.value),
        driveYButton = new JoystickButton(driveController, XboxController.Button.kY.value),
        driveLBumper = new JoystickButton(driveController, XboxController.Button.kLeftBumper.value),
        driveRBumper = new JoystickButton(driveController, XboxController.Button.kRightBumper.value);

    private final XboxController mechController = new XboxController(1);
    private final JoystickButton 
        mechAButton = new JoystickButton(mechController, XboxController.Button.kA.value),
        mechBButton = new JoystickButton(mechController, XboxController.Button.kB.value),
        mechXButton = new JoystickButton(mechController, XboxController.Button.kX.value),
        mechYButton = new JoystickButton(mechController, XboxController.Button.kY.value),
        mechLBumper = new JoystickButton(mechController, XboxController.Button.kLeftBumper.value),
        mechRBumper = new JoystickButton(mechController, XboxController.Button.kRightBumper.value);

    // Commands
    private final SendableChooser<Command> autonChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        powerDistribution = new PowerDistribution();

        // Instantiate the Jetson connection
        jetson = new JetsonConnection();
        
        // Instantiate subsystems
        tankSubsystem = new TankSubsystem();
        turretSubsystem = new TurretSubsystem(tankSubsystem, jetson);
        internalSubsystem = new InternalSubsystem(turretSubsystem);
        intakeSubsystem = new IntakeSubsystem(internalSubsystem, jetson);
        climbSubsystem = new ClimbSubsystem();

        // Instantiate power controller
        powerController = new PowerController(
            powerDistribution,
            tankSubsystem,
            turretSubsystem,
            internalSubsystem,
            intakeSubsystem
        );

        // Configure the button bindings
        configureButtonBindings();

        // Add auton sequences to the chooser and add the chooser to shuffleboard
        autonChooser = new SendableChooser<>();
        autonChooser.addOption("Red top", new AutonRedTopSequence(this));
        autonChooser.addOption("Red middle", new AutonRedMiddleSequence(this));
        autonChooser.addOption("Red bottom", new AutonRedBottomSequence(this));
        autonChooser.addOption("Blue top", new AutonBlueTopSequence(this));
        autonChooser.addOption("Blue middle", new AutonBlueMiddleSequence(this));
        autonChooser.addOption("Blue bottom", new AutonBlueBottomSequence(this));
        autonChooser.setDefaultOption("Pleb auton", PlebAutonSequence.from(this));
        autonChooser.addOption("Skip auton", new InstantCommand());

        new GRTShuffleboardTab("Drivetrain").addWidget("Auton sequence", autonChooser);

        double hubDist = 70;
        Pose2d initialPose = new Pose2d(Units.inchesToMeters(hubDist), 0, new Rotation2d());

        // Setting initial position assuming we are facing the hub
        // at a distance `hubDist` inches and 0 on the y axis.
        setInitialPosition(initialPose);

        // Shuffleboard manual reset button
        new GRTShuffleboardTab("Turret").addWidget("Zero position", new InstantCommand(() -> {
            setInitialPosition(initialPose);
            turretSubsystem.zeroEncoders();
        }, turretSubsystem, tankSubsystem));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     * 
     * Controller 1 (drive):
     * Joysticks (held)       -> car drive -- left Y axis controls foward power, right X axis controls angular power
     * X button (pressed)     -> toggle turret retracted mode
     * Left trigger (held)    -> freeze turret
     * Right bumper (pressed) -> manual reset turret r to 140 in. (edge of tarmac)
     * Right bumper (held)    -> slow mode; scales down drive powers for more precise inputs
     * 
     * Controller 2 (mech):
     * A button (pressed)           -> request shot
     * B button (pressed)           -> toggle intake position (raise / lower)
     * X button (pressed)           -> reset turret offsets
     * Y button (pressed)           -> toggle turret low hub mode
     * Triggers (held)              -> run intake -- left backwards, right forwards
     * Left joystick Y axis (held)  -> offset intake position if it fails to hit a hard stop
     * Right joystick Y axis (held) -> run climb -- up to extend, down to retract
     * Left bumper (held)           -> override flywheel only (force flywheel to run)
     * Right bumper (held)          -> override both flywheel and internals (force flywheel to run, internals after spinup delay)
     * Right bumper (held)          -> slow mode; scales down drive powers for more precise inputs
     */
    private void configureButtonBindings() {
        driveAButton.whenPressed(new RequestShotCommand(internalSubsystem));
        // driveBButton.whenPressed(new DeployIntakeCommand(intakeSubsystem));
        // driveYButton.whenPressed(new RaiseIntakeCommand(intakeSubsystem));
        driveXButton.whenPressed(new InstantCommand(turretSubsystem::toggleClimb));
        driveRBumper.whenPressed(new InstantCommand(() -> turretSubsystem.setR(140)));

        mechAButton.whenPressed(new RequestShotCommand(internalSubsystem));
        mechBButton.whenPressed(new InstantCommand(intakeSubsystem::togglePosition));
        mechXButton.whenPressed(new InstantCommand(turretSubsystem::resetOffsets));
        // mechYButton.whenPressed(new InstantCommand(turretSubsystem::toggleFreeze));
        // mechYButton.whenPressed(new InstantCommand(turretSubsystem::toggleClimb));
        mechYButton.whenPressed(new InstantCommand(turretSubsystem::toggleLow));
        // mechLBumper.toggleWhenPressed(new StartEndCommand(
        //     () -> turretSubsystem.setDriverOverrideFlywheel(true),
        //     () -> turretSubsystem.setDriverOverrideFlywheel(false), 
        //     turretSubsystem
        // ));

        // Car drive with the left Y axis controlling y power and the right X axis controlling angular
        tankSubsystem.setDefaultCommand(new RunCommand(() -> {
            boolean slowMode = driveController.getRightBumper() || mechController.getRightBumper();

            // double turn_stick = driveController.getRightX();
            double yPow = -driveController.getLeftY();
            double turnPow = driveController.getRightX() * 0.65;

            if (slowMode) {
                yPow *= 0.3;
                turnPow *= 0.3;
            }

            turretSubsystem.setDriving(Math.abs(yPow) + Math.abs(turnPow) > 0.3);
            tankSubsystem.setCarDrivePowers(yPow, turnPow);
        }, tankSubsystem));

        // Driver-override intake command
        intakeSubsystem.setDefaultCommand(new RunIntakeCommand(intakeSubsystem, mechController));

        // Set turret offsets from mech controller POV input:
        // Top/bottom to increase/decrease hub distance offset,
        // right/left to increase/decrease theta offset.
        turretSubsystem.setDefaultCommand(new RunCommand(() -> {
            switch (mechController.getPOV()) {
                case 0: turretSubsystem.changeDistanceOffset(1.5); break;
                case 90: turretSubsystem.changeTurntableOffset(Math.toRadians(3)); break;
                case 180: turretSubsystem.changeDistanceOffset(-1.5); break;
                case 270: turretSubsystem.changeTurntableOffset(Math.toRadians(-3)); break;
                default: break;
            }

            turretSubsystem.setFreeze(driveController.getLeftTriggerAxis() > 0.2);
        }, turretSubsystem));

        // Override internals and flywheel on mech left and right bumpers
        internalSubsystem.setDefaultCommand(new OverrideInternalsCommand(internalSubsystem, turretSubsystem, mechController));

        // Manual climb control with the right mech joystick:
        // Push up to extend, down to retract; brakes are automatically set when manual control 
        // is supplied.
        climbSubsystem.setDefaultCommand(new RunCommand(() -> {
            double pow = -mechController.getRightY();
            climbSubsystem.driveSixArm(pow);
        }, climbSubsystem));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
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

    public PowerDistribution getPowerDistribution() {
        return powerDistribution;
    }

    /**
     * Gets the PowerController instance for scaling in `Robot.periodic()`.
     * @return The PowerController instance.
     */
    public PowerController getPowerController() {
        return powerController;
    }

    /**
     * Gets the robot's TankSubsystem instance.
     * @return The TankSubsystem instance.
     */
    public TankSubsystem getTankSubsystem() {
        return tankSubsystem;
    }

    /**
     * Gets the robot's TurretSubsystem instance.
     * @return The TurretSubsystem instance.
     */
    public TurretSubsystem getTurretSubsystem() {
        return turretSubsystem;
    }

    /**
     * Gets the robot's IntakeSubsystem instance.
     * @return The IntakeSubsystem instance.
     */
    public IntakeSubsystem getIntakeSubsystem() {
        return intakeSubsystem;
    }

    /**
     * Gets the robot's InternalSubsystem instance.
     * @return The InternalSubsystem instance.
     */
    public InternalSubsystem getInternalSubsystem() {
        return internalSubsystem;
    }
}
