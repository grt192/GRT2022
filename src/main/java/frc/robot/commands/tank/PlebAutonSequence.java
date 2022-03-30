package frc.robot.commands.tank;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.RobotContainer;
import frc.robot.commands.intake.DeployIntakeCommand;
import frc.robot.commands.intake.RaiseIntakeCommand;
import frc.robot.shuffleboard.GRTNetworkTableEntry;
import frc.robot.shuffleboard.GRTShuffleboardTab;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakePosition;
import frc.robot.subsystems.internals.InternalSubsystem;
import frc.robot.subsystems.tank.TankSubsystem;

public class PlebAutonSequence extends SequentialCommandGroup {
    private Translation2d start = new Translation2d(Units.inchesToMeters(87), 0);

    private final RobotContainer container;
    private final TankSubsystem tankSubsystem;
    private final TurretSubsystem turretSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final InternalSubsystem internalSubsystem;

    private final GRTShuffleboardTab shuffleboardTab = new GRTShuffleboardTab("Auton");

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

        addCommands(
            new DeployIntakeCommand(intakeSubsystem),
            new PlebAutonCommand(),
            new RaiseIntakeCommand(intakeSubsystem)
        );
    }

    @Override
    public void initialize() {
        // Set initial position assuming we are 0in on the y axis, 87in on the x axis,
        // with theta equal to the current turntable position.
        double turnPos = turretSubsystem.getTurntablePosition();
        container.setInitialPosition(new Pose2d(start, new Rotation2d(Math.PI - turnPos)));
    }

    private class PlebAutonCommand extends CommandBase {
        private final Timer autonTimer = new Timer();
        private final Timer forceTimer = new Timer();
    
        private int shotsCompleted = 0;
        private boolean complete = false;

        private final GRTNetworkTableEntry forceTimerEntry = shuffleboardTab.addEntry("Force timer", forceTimer.get());
        private final GRTNetworkTableEntry shotsCompletedEntry = shuffleboardTab.addEntry("Shots completed", shotsCompleted);
        private final GRTNetworkTableEntry completedEntry = shuffleboardTab.addEntry("Completed", complete);

        @Override
        public void initialize() {
            turretSubsystem.setDriverOverrideFlywheel(true);
            autonTimer.start();
            shotsCompleted = 0;
        }

        @Override
        public void execute() {
            forceTimerEntry.setValue(forceTimer.get());
            shotsCompletedEntry.setValue(shotsCompleted);

            // We are done driving if: we are beyond 55 inches of our starting position,
            // or 8 seconds have passed.
            boolean doneDriving = tankSubsystem.distance(start) > Units.inchesToMeters(55) 
                || autonTimer.hasElapsed(8);

            // If we're not done driving, drive forwards at 0.4 power
            // If we are done driving, start the timer to force a shot.
            tankSubsystem.setCarDrivePowers(!doneDriving ? 0.4 : 0, 0);
            if (doneDriving) forceTimer.start();

            // This runs once immediately and after every successful shot.
            // If we're not requesting a shot, request one and reset the force timer.
            // End the command after shooting twice.
            if (!internalSubsystem.getShotRequested()) {
                if (doneDriving && shotsCompleted == 2) {
                    intakeSubsystem.setPosition(IntakePosition.RAISED);
                    completedEntry.setValue(true);
                    complete = true;
                }
                forceTimer.reset();
                internalSubsystem.requestShot();
                shotsCompleted++;
            }

            // Force a shot if we haven't shot in 4 seconds
            if (forceTimer.hasElapsed(4)) {
                internalSubsystem.requestShot();
            }
        }

        @Override
        public boolean isFinished() {
            return complete;
        }

        @Override
        public void end(boolean interrupted) {
            turretSubsystem.setDriverOverrideFlywheel(false);
            autonTimer.stop();
            autonTimer.reset();
            forceTimer.stop();
            forceTimer.reset();
        }
    }
}
