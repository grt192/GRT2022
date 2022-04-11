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

### Auton ([#22](https://github.com/grt192/GRTCommandBased/pull/22), merged into #27)
[...]

## Intake
[...]

## Internals
[...]

## Turret ([#16](https://github.com/grt192/GRTCommandBased/pull/16), [#27](https://github.com/grt192/GRTCommandBased/pull/27), [#32](https://github.com/grt192/GRTCommandBased/pull/32))
[...]. To aim the turret, `TurretSubsystem` maintains a hub distance (`r`) and turntable angle (`theta`) to constantly
align the turret with the hub and set the hood position and flywheel velocity [...].

### The `rtheta` state system ([#24](https://github.com/grt192/GRTCommandBased/pull/24), merged into #27)
While vision is out of range (in the turntable's blind spot) or not working (flywheel on), `TurretSubsystem` continues to
modify its internal states from `TankSubsystem` localization deltas. This is done by creating a polar coordinate system 
with the robot lying on the x-axis a distance `r` and facing an angle `theta` from the hub.

<p align="center">
    <img src="https://user-images.githubusercontent.com/60120929/162667303-702bf9a9-b417-4350-889b-60ed466afa7b.jpg" width="700px">
</p>

When the robot moves, the delta x and y can be retrieved from localization. Using the deltas, we can calculate `h = Math.hypot(dx, dy)`
and `alpha = Math.atan2(-dy, dx)`.

<p align="center">
    <img src="https://user-images.githubusercontent.com/60120929/162667299-c2bad0ce-bbdf-40a9-a784-e955f1690d7f.jpg" width="700px">
</p>

Using `h` and `alpha`, we can calculate the new `r` value, as well as the angle `phi`.

<p align="center">
    <img src="https://user-images.githubusercontent.com/60120929/162667295-cbf1ac42-8360-4374-a973-cfbdae04e4ef.jpg" width="700px">
</p>

To calculate the new `theta`, take the total angle `theta + dtheta` and subtract the angle `phi` between the old and new 
"x-axes". 

<p align="center">
    <img src="https://user-images.githubusercontent.com/60120929/162668298-190d8620-19c3-4eb8-ae65-847828932742.jpg" width="700px">
</p>

The turntable reference is the supplemental angle to `theta`, or `Math.PI - theta` radians.

<!-- explain feedforward? -->
For feedforward purposes, the turret calculates delta r and theta instead of just the new values. When vision is out of 
range, these deltas are added to the current state values to update them.
```java
/**
 * Calculates the deltas of the `r` and `theta` coordinate system from odometry deltas.
 * Used for `rtheta` feedforward, as well as `rtheta` state updating while vision is
 * out of range.
 * 
 * @param lastPosition The previous odometry position.
 * @param currentPosition The current odometry position.
 * @return The `rtheta` deltas, as a pair of [dr (in), dtheta (rads)].
 */
private Pair<Double, Double> calculateRThetaDeltas(Pose2d lastPosition, Pose2d currentPosition) {
    Pose2d deltas = currentPosition.relativeTo(lastPosition);

    double dx = Units.metersToInches(deltas.getX());
    double dy = Units.metersToInches(deltas.getY());
    double dTheta = deltas.getRotation().getRadians();

    double h = Math.hypot(dx, dy);
    double alpha = Math.atan2(-dy, dx);
    double beta = alpha + this.theta;

    double x = r + h * Math.cos(beta);
    double y = h * Math.sin(beta);

    if (PRINT_STATES) System.out.println(
        "dx: " + dx + " dy: " + dy + " dtheta: " + dTheta
        + "\npose: " + currentPosition + " last: " + lastPosition
        + "\nh: " + h + " alpha: " + alpha + " beta: " + beta
        + "\nx: " + x + " y: " + y
        + "\nr: " + r + ", theta: " + Math.toDegrees(theta)
    );

    double deltaR = Math.hypot(x, y) - r;
    double deltaTheta = dTheta - Math.atan2(y, x);
    return new Pair<>(deltaR, deltaTheta);
}
```
##### [`TurretSubsystem` L462-496](https://github.com/grt192/GRTCommandBased/blob/develop/src/main/java/frc/robot/subsystems/TurretSubsystem.java#L462-L496)

### Interpolation
To find the flywheel speed and hood angle that would work for the turret's current hub distance, `TurretSubsystem` 
linearly interpolates flywheel RPM and hood degrees from known manual testing data points, recorded in the following
[spreadsheet](https://docs.google.com/spreadsheets/d/1hAsBn0KIxucuwOv96UMS3bZOrim03YxqXL-2-pBrjzI/edit?usp=sharing).

<p align="center">
    <img src="https://user-images.githubusercontent.com/60120929/162670097-63f1183c-8f15-473d-a90d-0830b2b6153e.png" width="700px">
    <img src="https://user-images.githubusercontent.com/60120929/162670129-8a0280be-02f1-44d2-b035-4bf4287bdeaf.png" width="700px">
</p>

In code, these values were stored in a `double[][]` interpolation table representing tuples of `[hub dist (in), 
flywheel speed (RPM), hood angle (degs)]`.
```java
private static final double[][] INTERPOLATION_TABLE = {
    { 59, 4600, 0 },
    // { 69, 5000, 7 },
    { 75, 4800, 8 },
    { 88, 5100, 13.5 },
    // { 91, 4950, 10 },
    { 112, 5300, 17 },
    { 139, 5600, 20 },
    { 175, 6000, 25 },
    { 202, 6400, 30 },
    { 221, 6800, 36 }
};
```
##### [`TurretSubsystem` L132-143](https://github.com/grt192/GRTCommandBased/blob/develop/src/main/java/frc/robot/subsystems/TurretSubsystem.java#L132-L143)

<!-- reword -->
To interpolate values for any hub distance, draw a line from the immediately lower distance known data point to the 
immediately greater distance known data point; the intersection of that line and the current hub distance is the interpolated
value.
```java
/**
 * Interpolate the hood angle and flywheel RPM from the hub distance and interpolation 
 * table. If rejecting, scale down flywheel speed to prevent scoring.
 */
private void interpolateFlywheelHoodRefs(double r) {
    // Apply offset and clamp between minimum and maximum hub distance entries
    // TODO: clamp slightly below min and above max?
    double hubDistance = Math.min(Math.max(r + distanceOffset, 
        INTERPOLATION_TABLE[0][0]), INTERPOLATION_TABLE[INTERPOLATION_TABLE.length - 1][0]);

    for (int i = 1; i < INTERPOLATION_TABLE.length; i++) {
        double[] above = INTERPOLATION_TABLE[i];
        double[] below = INTERPOLATION_TABLE[i - 1];

        double rTop = above[0], flywheelTop = above[1], hoodAngleTop = above[2];
        double rBottom = below[0], flywheelBottom = below[1], hoodAngleBottom = below[2];

        // If the entry's distance is above the current distance, the current distance
        // is between the entry and the previous entry.
        if (rTop > hubDistance) {
            // Where x axis is distance d, y axis is flywheel RPM f, the top and bottom
            // points are A and B, and the interpolated point is C:
            // m = (f_A - f_B) / (d_A - d_B)
            // C = (d_B + (d_C - d_B), mx) = (d_C, md_C)
            double flywheelSlope = (flywheelTop - flywheelBottom) / (rTop - rBottom);
            desiredFlywheelRPM = flywheelBottom + flywheelSlope * (hubDistance - rBottom);
            if (mode == TurretMode.REJECTING && !SKIP_REJECTION) desiredFlywheelRPM *= 0.5;

            double hoodSlope = (hoodAngleTop - hoodAngleBottom) / (rTop - rBottom);
            desiredHoodRadians = Math.toRadians(hoodAngleBottom + hoodSlope * (hubDistance - rBottom));
            return;
        }
    }
}
```
##### [`TurretSubsystem` L517-550](https://github.com/grt192/GRTCommandBased/blob/develop/src/main/java/frc/robot/subsystems/TurretSubsystem.java#L517-L550)

While [...], [...].

## Vision
[...]
