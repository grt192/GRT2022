package frc.robot.commands.tank;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem.IntakePosition;

public class PlebAutonSequence extends CommandBase {
    private static final double IN_TO_M = 0.0254;
    Translation2d start = new Translation2d(87 * IN_TO_M, 0);
    private final RobotContainer container;

    private double startTime;
    private boolean complete = false;

    public PlebAutonSequence(RobotContainer robotContainer) {
        this.container = robotContainer;

        addRequirements(robotContainer.getTankSubsystem(), robotContainer.getInternalSubsystem(),
                robotContainer.getTurretSubsystem(), robotContainer.getIntakeSubsystem());
    }

    @Override
    public void initialize() {
        double turnPos = container.getTurretSubsystem().getTurntablePosition();

        container.setInitialPosition(new Pose2d(start, new Rotation2d(Math.PI - turnPos)));

        container.getIntakeSubsystem().setDriverOverride(true);
        container.getIntakeSubsystem().setIntakePower(1);
        container.getIntakeSubsystem().setPosition(IntakePosition.DEPLOYED);

        this.startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        boolean distDone = container.getTankSubsystem().distance(start) > (55 * IN_TO_M);
        boolean timeDone = (Timer.getFPGATimestamp() - startTime >= 8);
        if (!distDone && !timeDone) {
            container.getTankSubsystem().setCarDrivePowers(0.4, 0);
        } else {
            container.getTankSubsystem().setCarDrivePowers(0, 0);
        }

        container.getInternalSubsystem().requestShot();

        if ((distDone || timeDone) && container.getInternalSubsystem().isShotRequested() == false) {
            this.complete = true;
        }
    }

    @Override
    public boolean isFinished() {
        return complete;
    }
}
