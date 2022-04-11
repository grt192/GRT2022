# GRT Controls Writeup 2022
[...]

## Tank ([#1](https://github.com/grt192/GRTCommandBased/pull/1), [#6](https://github.com/grt192/GRTCommandBased/pull/6))
The tank subsystem [`TankSubsystem`](https://github.com/grt192/GRTCommandBased/blob/develop/src/main/java/frc/robot/subsystems/tank/TankSubsystem.java)
[...]. Our robot had a west-coast (drop center) tank drivetrain, controlled by a gearbox on each side with 3 NEOs each.
The system was controlled by a left and right main NEO, and 2 follower NEOs which mirrored the leader's inputs.

During shop project, we tested two different control schemes for the drivetrain. In tank drive, the driver supplies directly
a left and right power for the robot. To drive forward, supply the same power to the left and right motors. To turn in one
direction, supply less power to that side than the other.
```java
/**
 * Drive the system with the given power scales using the tank system.
 * @param leftScale Scale in the forward/backward direction of the left motor, from -1 to 1.
 * @param rightScale Scale in the forward/backward direction of the right motor, from -1 to 1.
 */
public void setTankDrivePowers(double leftScale, double rightScale, boolean squareInput) {
    if (squareInput) {
        leftScale = squareInput(leftScale);
        rightScale = squareInput(rightScale);
    }

    // Set motor output state
    leftMain.set(leftScale);
    rightMain.set(rightScale);
}
```
##### [`TankSubsystem` L143-157](https://github.com/grt192/GRTCommandBased/blob/develop/src/main/java/frc/robot/subsystems/tank/TankSubsystem.java#L143-L157)
This was deemed [...] by the drivers. Instead, we used car (arcade) drive, where the driver supplies a forward and angular
power. To drive forwards, supply only forward power. To turn, supply angular power in the direction of turning.
```java
public void setCarDrivePowers(double yScale, double angularScale, boolean squareInput) {
    // Square the input if needed for finer control
    if (squareInput) {
        yScale = squareInput(yScale);
        angularScale = squareInput(angularScale);
    }

    double leftPowerTemp = yScale + angularScale;
    double rightPowerTemp = yScale - angularScale;

    // Scale powers greater than 1 back to 1 if needed
    double largest_power = Math.max(Math.abs(leftPowerTemp), Math.abs(rightPowerTemp));
    if (largest_power > 1.0) {
        double scale = 1.0 / largest_power;

        leftPowerTemp *= scale;
        rightPowerTemp *= scale;
    }

    // Set motor output state
    leftMain.set(leftPowerTemp);
    rightMain.set(rightPowerTemp);
}
```
##### [`TankSubsystem` L119-141](https://github.com/grt192/GRTCommandBased/blob/develop/src/main/java/frc/robot/subsystems/tank/TankSubsystem.java#L119-L141)
[...]

### Localization and Path Following ([#9](https://github.com/grt192/GRTCommandBased/pull/9))
For localization, we decided that writing our own pose estimation would take too much time considering our relative lack
of experience and decided to use WPILib's built-in [`DifferentialDrivePoseEstimator`](https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/math/estimator/DifferentialDrivePoseEstimator.html) 
instead. The `DifferentialDrivePoseEstimator` wraps a Kalman filter around raw encoder, gyro, and vision measurements to
more accurately estimate the robot's position on the field.
```java
public void update() {
    Rotation2d gyroAngle = getGyroHeading();
    lastWheelSpeeds = getWheelSpeeds();
    double leftDistance = leftEncoder.getPosition();
    double rightDistance = rightEncoder.getPosition();

    poseEstimator.update(gyroAngle, lastWheelSpeeds, leftDistance, rightDistance);
}
```
##### [`PoseEstimator` L41-48](https://github.com/grt192/GRTCommandBased/blob/develop/src/main/java/frc/robot/subsystems/tank/PoseEstimator.java#L41-L48)

Similarly, path following code used the builtin WPILib [`RamseteCommand`](https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/RamseteCommand.html) 
and [`TrajectoryGenerator`](https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/math/trajectory/TrajectoryGenerator.html) utilities.
`TrajectoryGenerator` generates a clamped cubic spline path [`Trajectory`](https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/math/trajectory/Trajectory.html) 
(essentially a list of wheel speeds and positions) between two points with constraints dictated by config parameters, and
`RamseteCommand` feeds the trajectory into the [`RamseteController`](https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/math/controller/RamseteController.html),
which consumes the localization robot pose to adjust PID references to maintain the desired trajectory-state pose. `RamseteCommand`
uses a [`SimpleMotorFeedForward`](https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/math/controller/SimpleMotorFeedforward.html) 
to calculate inputs to maintain the desired trajectory velocity and acceleration, then performs PID for motor voltage which
is supplied to `TankSubsystem`.
```java
/**
 * Creates a FollowPathCommand from a given trajectory.
 * 
 * @param tankSubsystem The tank subsystem.
 * @param trajectory The trajectory to follow.
 */
public FollowPathCommand(TankSubsystem tankSubsystem, Trajectory trajectory) {
    super(
        trajectory,
        tankSubsystem::getRobotPosition, // Position supplier
        new RamseteController(RAMSETE_B, RAMSETE_ZETA),
        new SimpleMotorFeedforward(Ks, Kv, Ka),
        KINEMATICS,
        tankSubsystem::getWheelSpeeds, // Wheel speed supplier
        // PID controllers
        new PIDController(Kp, Ki, Kd),
        new PIDController(Kp, Ki, Kd),
        tankSubsystem::setTankDriveVoltages, // Wheel voltage consumer
        tankSubsystem
    );

    tankSubsystem.resetPosition(trajectory.getInitialPose());
}

/**
 * Creates a FollowPathCommand from a given start point, list of waypoints, end point, and boolean representing whether
 * the path should be reversed (if the robot should drive backwards through the trajectory).
 * 
 * @param tankSubsystem The tank subsystem.
 * @param start The start point of the trajectory as a Pose2d.
 * @param waypoints A list of waypoints the robot must pass through as a List<Translation2d>.
 * @param end The end point of the trajectory as a Pose2d.
 * @param reversed Whether the trajectory is reversed.
 */
public FollowPathCommand(TankSubsystem tankSubsystem, Pose2d start, List<Translation2d> waypoints, Pose2d end, boolean reversed) {
    this(
        tankSubsystem,
        // Target trajectory
        TrajectoryGenerator.generateTrajectory(
            start, waypoints, end, 
            new TrajectoryConfig(MAX_VEL, MAX_ACCEL)
                .setReversed(reversed)
                .setKinematics(KINEMATICS)
                .addConstraint(new DifferentialDriveVoltageConstraint(
                    new SimpleMotorFeedforward(Ks, Kv, Ka), 
                    KINEMATICS, 
                    10))
        )
    );
}
```
##### [`FollowPathCommand` L52-101](https://github.com/grt192/GRTCommandBased/blob/develop/src/main/java/frc/robot/commands/tank/FollowPathCommand.java#L52-L101)

While using WPILib builtins greatly reduced the time required to write and start testing localization and path following
code, the fact that they were essentially black boxes [...]. [...].

## Intake
[...]

## Internals
[...]

## Turret
[...]

### The `rtheta` state system
[...]

## Vision
[...]
