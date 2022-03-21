package frc.robot.commands.tank;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.RobotContainer;

/**
 * The superclass for all auton sequences, containing helper methods and shared constructor logic.
 */
public class GRTAutonSequence extends SequentialCommandGroup {
    public GRTAutonSequence(RobotContainer robotContainer, Pose2d initialPose) {
        //robotContainer.setInitialPose(initialPose);
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
        // Intake length + robot length / 2
        double h = 11.734 + 34 / 2;

        double rads = Math.toRadians(angleOfApproach);
        double dx = Units.inchesToMeters(h * Math.cos(rads));
        double dy = Units.inchesToMeters(h * Math.sin(rads));

        double x = ball.getX() - dx;
        double y = ball.getY() - dy;
        return new Pose2d(x, y, new Rotation2d(rads));
    }
}
