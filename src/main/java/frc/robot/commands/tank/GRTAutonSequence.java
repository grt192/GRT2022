package frc.robot.commands.tank;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.RobotContainer;
import frc.robot.commands.intake.DeployIntakeCommand;
import frc.robot.commands.intake.RaiseIntakeCommand;
import frc.robot.commands.internals.ShootCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.internals.InternalSubsystem;
import frc.robot.subsystems.tank.TankSubsystem;

/**
 * The superclass for all auton sequences, containing helper methods and shared constructor logic.
 */
public abstract class GRTAutonSequence extends SequentialCommandGroup {
    private final TankSubsystem tankSubsystem;
    private final InternalSubsystem internalSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    /**
     * Creates a GRTAutonSequence from an initial and ball one pose. This is used for shared initialization logic
     * between all auton paths, as well as one-ball sequences like the top and bottom paths. Pathfollows to the ball, 
     * then shoots two balls.
     * 
     * @param robotContainer The RobotContainer instance, for calling `.setInitialPose()`.
     * @param initialPose The initial pose of the sequence.
     * @param ballOnePose The first ball pose of the sequence.
     */
    public GRTAutonSequence(RobotContainer robotContainer, Pose2d initialPose, Pose2d ballOnePose) {
        tankSubsystem = robotContainer.getTankSubsystem();
        internalSubsystem = robotContainer.getInternalSubsystem();
        intakeSubsystem = robotContainer.getIntakeSubsystem();

        addRequirements(tankSubsystem, internalSubsystem, intakeSubsystem);
        robotContainer.setInitialPosition(initialPose);

        addCommands(
            new DeployIntakeCommand(intakeSubsystem),
            new FollowPathCommand(tankSubsystem, initialPose, List.of(), ballOnePose),
            new ShootCommand(internalSubsystem),
            new ShootCommand(internalSubsystem)
        );
    }

    /**
     * Creates a GRTAutonSequence from an initial, ball one, and ball two pose. This is used for two-ball sequences, like
     * the middle paths where we also drive to the terminal to pick up and shoot a second ball. Pathfollows to the ball,
     * shoots two, then pathfollows to the second ball and shoots it.
     * 
     * @param robotContainer The RobotContainer instance, for calling `.setInitialPose()`.
     * @param initialPose The initial pose of the sequence.
     * @param ballOnePose The first ball pose of the sequence.
     * @param ballTwoPose The second ball pose of the sequence.
     */
    public GRTAutonSequence(RobotContainer robotContainer, Pose2d initialPose, Pose2d ballOnePose, Pose2d ballTwoPose) {
        this(robotContainer, initialPose, ballOnePose);

        addCommands(
            new FollowPathCommand(tankSubsystem, ballOnePose, List.of(), ballTwoPose),
            new ShootCommand(internalSubsystem)
        );
    }

    /**
     * Localizes a ball coordinate to a robot pose used in path following. The robot cannot path follow
     * directly to the ball coordinate, as the robot's position is based on the center of the chassis;
     * instead, the robot should path follow to the Pose2d localized from the intake and chassis width.
     * 
     * @param ball The ball coordinate, represented as a Translation2d.
     * @param angleOfApproach The angle the robot should approach the ball, in degrees.
     * @return The Pose2d to path follow to.
     */
    protected static Pose2d localizeBallCoordinate(Translation2d ball, double angleOfApproach) {
        // Robot length / 2
        double h = 34 / 2;

        double rads = Math.toRadians(angleOfApproach);
        double dx = Units.inchesToMeters(h * Math.cos(rads));
        double dy = Units.inchesToMeters(h * Math.sin(rads));

        double x = ball.getX() - dx;
        double y = ball.getY() - dy;
        return new Pose2d(x, y, new Rotation2d(rads));
    }
}
