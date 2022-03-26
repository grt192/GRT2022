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
        double turnPos = turretSubsystem.getTurntablePosition();
        container.setInitialPosition(new Pose2d(start, new Rotation2d(Math.PI - turnPos)));

        autonTimer.start();

        /*
        intakeSubsystem.setDriverOverride(true);
        intakeSubsystem.setIntakePower(1);
        intakeSubsystem.setPosition(IntakePosition.DEPLOYED);
        */
    }

    @Override
    public void execute() {
        boolean distDone = tankSubsystem.distance(start) > Units.inchesToMeters(55);
        boolean timeDone = autonTimer.hasElapsed(8);
        if (!distDone && !timeDone) {
            tankSubsystem.setCarDrivePowers(0.4, 0);
        } else {
            tankSubsystem.setCarDrivePowers(0, 0);
        }

        if (internalSubsystem.isShotRequested() == false) {
            if (distDone || timeDone) this.complete = true;
        } else if (autonTimer.hasElapsed(4)) {
            internalSubsystem.requestShot();
        }
    }

    @Override
    public boolean isFinished() {
        return complete;
    }
}
