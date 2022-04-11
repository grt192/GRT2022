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
import frc.robot.subsystems.TurretSubsystem.TurretMode;
import frc.robot.subsystems.internals.InternalSubsystem;
import frc.robot.subsystems.tank.TankSubsystem;

public class PlebAutonSequence extends CommandBase {
    private Translation2d start = new Translation2d(Units.inchesToMeters(87), 0);

    private final RobotContainer container;
    private final TankSubsystem tankSubsystem;
    private final TurretSubsystem turretSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final InternalSubsystem internalSubsystem;

    private final Timer autonTimer = new Timer();
    private final Timer forceTimer = new Timer();
    private final Timer internalsTimer = new Timer();

    private int shotsRequested = 0;
    private boolean complete = false;

    private final GRTShuffleboardTab shuffleboardTab = new GRTShuffleboardTab("Auton");
    private final GRTNetworkTableEntry forceTimerEntry = shuffleboardTab.addEntry("Force timer", forceTimer.get());
    private final GRTNetworkTableEntry shotsRequestedEntry = shuffleboardTab.addEntry("Shots requested", shotsRequested);
    private final GRTNetworkTableEntry completedEntry = shuffleboardTab.addEntry("Completed", complete);

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

    /**
     * Composes this command in a sequence that deploys intake at the start and raises it at the end.
     * Use this method instead of the constructor to create this auton sequence.
     * 
     * @param robotContainer The RobotContainer.
     * @return The SequentialCommandGroup representing the auton sequence.
     */
    public static SequentialCommandGroup from(RobotContainer robotContainer) {
        IntakeSubsystem intakeSubsystem = robotContainer.getIntakeSubsystem();

        return new PlebAutonSequence(robotContainer)
            .beforeStarting(new DeployIntakeCommand(intakeSubsystem))
            .andThen(new RaiseIntakeCommand(intakeSubsystem));
    }

    @Override
    public void initialize() {
        turretSubsystem.setMode(TurretMode.SHOOTING);
        turretSubsystem.setDriverOverrideFlywheel(true);
        autonTimer.start();
        shotsRequested = 0;

        // Set initial position assuming we are 0 in on the y axis, 87 in on the x axis,
        // with theta equal to the current turntable position.
        double turnPos = turretSubsystem.getTurntablePosition();
        container.setInitialPosition(new Pose2d(start, new Rotation2d(Math.PI - turnPos)));
    }

    @Override
    public void execute() {
        forceTimerEntry.setValue(forceTimer.get());
        shotsRequestedEntry.setValue(shotsRequested);

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
        if ((!internalSubsystem.getDriverOverride() && !internalSubsystem.getShotRequested()) || internalsTimer.hasElapsed(1)) {
            if (doneDriving && shotsRequested == 2) {
                intakeSubsystem.setPosition(IntakePosition.RAISED); // TODO: is this needed? we;re already composed within the sequence to be followed by a RaiseIntakeCommand
                completedEntry.setValue(true);
                complete = true;
            }
            forceTimer.reset();
            internalsTimer.stop();
            internalsTimer.reset();

            internalSubsystem.setDriverOverride(false);
            internalSubsystem.requestShot();
            shotsRequested++;
        }

        // Force a shot if we haven't shot in 4 seconds
        if (forceTimer.hasElapsed(4)) {
            internalSubsystem.setDriverOverride(true);
            internalsTimer.start();
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
