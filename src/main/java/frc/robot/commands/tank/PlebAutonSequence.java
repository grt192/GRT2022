package frc.robot.commands.tank;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class PlebAutonSequence extends CommandBase {
    private final RobotContainer container;
    private final Pose2d start;

    public PlebAutonSequence(RobotContainer robotContainer, Pose2d start) {
        this.start = start;
        this.container = robotContainer;

        addRequirements(robotContainer.getTankSubsystem(), robotContainer.getInternalSubsystem(), robotContainer.getTurretSubsystem());
    }

    @Override
    public void initialize() {
        container.setInitialPosition(new Pose2d());
    }
}
