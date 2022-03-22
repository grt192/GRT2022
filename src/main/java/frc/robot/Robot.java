// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.tank.TankSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command autonomousCommand;

    private RobotContainer robotContainer;
    private PowerDistribution powerDistribution;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer.    This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();
        powerDistribution = new PowerDistribution();
        LiveWindow.disableAllTelemetry();
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();

        // Calculate current limits for subsystems
        //robotContainer.getPowerController().calculateLimits();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
        // Turn off the ring light when the robot disables
        powerDistribution.setSwitchableChannel(false);
    }

    @Override
    public void disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        // Schedule the autonomous command
        autonomousCommand = robotContainer.getAutonomousCommand();
        if (autonomousCommand != null) autonomousCommand.schedule();

        // Turn on the ring light
        powerDistribution.setSwitchableChannel(true);
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }

        // Turn on the ring light
        powerDistribution.setSwitchableChannel(true);
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        TankSubsystem tankSubsystem = robotContainer.getTankSubsystem();
        TurretSubsystem turretSubsystem = robotContainer.getTurretSubsystem();
        IntakeSubsystem intakeSubsystem = robotContainer.getIntakeSubsystem();

        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();

        // Turn on the ring light
        powerDistribution.setSwitchableChannel(true);

        // Run DT turning left and right alternating
        tankSubsystem.setCarDrivePowers(0, 1); // Rotate DT right
        Timer.delay(2);
        tankSubsystem.setCarDrivePowers(0, -1); // Rotate DT left
        Timer.delay(2);
        tankSubsystem.setCarDrivePowers(0, 0);

        // Sweep intake from -1 to 1 power
        intakeSubsystem.setDriverOverride(true);
        for (double pow = -1; pow <= 1; pow += 0.1) {
            intakeSubsystem.setIntakePower(pow);
            Timer.delay(0.1);
        }
        intakeSubsystem.setDriverOverride(false);

        // Alternate intake deploy
        for (int i = 0; i < 2; i++) {
            intakeSubsystem.setPosition(IntakeSubsystem.IntakePosition.DEPLOYED);
            Timer.delay(4);
            intakeSubsystem.setPosition(IntakeSubsystem.IntakePosition.RAISED);
            Timer.delay(4);
        }

        // Sweep turntable (sequence: left, center, right, center)
        double turretTimeDelta = 1.5;  // Seconds to rotate turret 180
        turretSubsystem.changeTurntableOffset(Math.toRadians(-180));
        Timer.delay(turretTimeDelta);
        turretSubsystem.changeTurntableOffset(-Math.toRadians(-180));
        Timer.delay(turretTimeDelta);
        turretSubsystem.changeTurntableOffset(Math.toRadians(180));
        Timer.delay(turretTimeDelta);
        turretSubsystem.changeTurntableOffset(-Math.toRadians(180));

        // Run flywheel at a low speed 
        turretSubsystem.setFlywheelVel(1000);
        Timer.delay(5);
        turretSubsystem.setFlywheelVel(0);

        // Sweep hood up and down twice
        for (int i = 0; i < 2; i++) {
            turretSubsystem.setHoodPos(TurretSubsystem.HOOD_MAX_POS);
            Timer.delay(1.5);
            turretSubsystem.setHoodPos(TurretSubsystem.HOOD_MIN_POS);
            Timer.delay(1.5);
        }
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}
}
