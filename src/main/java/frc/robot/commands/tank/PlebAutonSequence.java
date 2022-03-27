package frc.robot.commands.tank;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakePosition;
import frc.robot.subsystems.internals.InternalSubsystem;
import frc.robot.subsystems.tank.TankSubsystem;

public class PlebAutonSequence extends CommandBase {
    private final RobotContainer container;
    private final TankSubsystem tankSubsystem;
    private final TurretSubsystem turretSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final InternalSubsystem internalSubsystem;

    private Translation2d start = new Translation2d(Units.inchesToMeters(87), 0);

    private final Timer autonTimer = new Timer();
    private final Timer forceTimer = new Timer();
    private final Timer finalTimer = new Timer();

    private int shotsRequested = 0;
    private boolean complete = false;

    public PlebAutonSequence(RobotContainer robotContainer) {
        this.container = robotContainer;
        this.tankSubsystem = robotContainer.getTankSubsystem();
        this.turretSubsystem = robotContainer.getTurretSubsystem();
        this.intakeSubsystem = robotContainer.getIntakeSubsystem();
        this.internalSubsystem = robotContainer.getInternalSubsystem();

        addRequirements(
            tankSubsystem, 
            internalSubsystem,
            turretSubsystem, 
            intakeSubsystem
        );
    }

    @Override
    public void initialize() {
        // Set initial position assuming we are 0in on the y axis, 87in on the x axis,
        // with theta equal to the current turntable position.
        double turnPos = turretSubsystem.getTurntablePosition();
        container.setInitialPosition(new Pose2d(start, new Rotation2d(Math.PI - turnPos)));

        autonTimer.start();
        forceTimer.start();

        intakeSubsystem.setDriverOverride(true);
        intakeSubsystem.setIntakePower(1);
        intakeSubsystem.setPosition(IntakePosition.DEPLOYED);

        turretSubsystem.setDriverOverrideFlywheel(true);
    }

    @Override
    public void execute() {
        if (complete) return;

        // We are done driving if: we are beyond 55 inches of our starting position,
        // or 8 seconds have passed.
        boolean doneDriving = tankSubsystem.distance(start) > Units.inchesToMeters(55) 
            || autonTimer.hasElapsed(8);

        // If we're not done driving the set distance or time, drive forwards at 0.4 power
        tankSubsystem.setCarDrivePowers(!doneDriving ? 0.4 : 0, 0);

        // This runs once immediately and after every successful shot.
        // If we're not requesting a shot, request one and reset the force timer.
        // End the command after shooting twice.
        if (!internalSubsystem.getShotRequested()) {
            if (doneDriving && shotsRequested == 2) {
                intakeSubsystem.setPosition(IntakePosition.RAISED);
                tankSubsystem.setCarDrivePowers(0.4, 0);
                finalTimer.start();
                complete = true;
            }
            forceTimer.reset();
            internalSubsystem.requestShot();
            shotsRequested++;
        }

        // Force a shot if we haven't shot in 6 seconds
        if (forceTimer.hasElapsed(6)) {
            internalSubsystem.requestShot();
            forceTimer.reset();
        }
    }

    @Override
    public boolean isFinished() {
        return finalTimer.hasElapsed(2.5);
    }

    @Override
    public void end(boolean interrupted) {
        turretSubsystem.setDriverOverrideFlywheel(false);
        tankSubsystem.setCarDrivePowers(0, 0);

        // Reset all timers when done
        autonTimer.stop();
        autonTimer.reset();
        forceTimer.stop();
        forceTimer.reset();
        finalTimer.stop();
        finalTimer.reset();
    }
}
