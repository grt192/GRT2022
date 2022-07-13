# GRT Controls Writeup 2022
[...]

## Tank ([#1](https://github.com/grt192/GRTCommandBased/pull/1), [#6](https://github.com/grt192/GRTCommandBased/pull/6))
The tank subsystem [`TankSubsystem`](https://github.com/grt192/GRTCommandBased/blob/develop/src/main/java/frc/robot/subsystems/tank/TankSubsystem.java)
controls the robot's west-coast (drop center) tank drive, supplying power to the 3-NEO gearboxes on each side.
In code, each side of the drive train was controlled by a "main" SparkMax, with two followers mirroring the output of the
main to the other NEO motors.

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

From driver input, we instead used car (arcade) drive, where the driver supplies a forward and angular power. To drive
forwards, supply only forward power. To turn, supply angular power in the direction of turning.
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

### Localization and Path Following ([#9](https://github.com/grt192/GRTCommandBased/pull/9))
For localization, even before the season it had been discussed that writing our own pose estimation would take too much time considering our relative lack
of experience, with us deciding to use WPILib's built-in [`DifferentialDrivePoseEstimator`](https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/math/estimator/DifferentialDrivePoseEstimator.html)
instead. The `DifferentialDrivePoseEstimator` wraps a [Kalman filter](https://en.wikipedia.org/wiki/Kalman_filter) around raw encoder, gyro, and vision measurements to
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
a Pure-Pursuit-esque "nonlinear controller" which consumes the localization robot pose to adjust PID references to maintain the desired trajectory-state pose. `RamseteCommand`
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
                    .addConstraint(
                        new DifferentialDriveVoltageConstraint(
                            new SimpleMotorFeedforward(Ks, Kv, Ka), 
                            KINEMATICS, 
                            10
                        )
                    )
        )
    );
}
```
##### [`FollowPathCommand` L52-104](https://github.com/grt192/GRTCommandBased/blob/develop/src/main/java/frc/robot/commands/tank/FollowPathCommand.java#L52-L104)

<a href="https://drive.google.com/file/d/10scxpLwNVB7aSsjQ-AoTtCNd0iGscWOs/view?usp=sharing">
    <p align="center">
        <img src="https://user-images.githubusercontent.com/60120929/164111907-db77cab4-0e1b-4ccb-abf3-93ae2c0e1da2.jpg" width="700px">
    </p>
</a>

##### Pathfollowing testing on the old DT base [VIDEO]

While using WPILib builtins greatly reduced the time required to write and start testing localization and path following
code, the fact that they were essentially black boxes caused confusion and in some cases prevented perfectly ideal behavior. 
For example, a Kalman filter allows the pose estimator fuse both vision and odometry readings when calculating the robot 
position, greatly increasing the accuracy of the resulting pose. However, due to the circular (and thus symmetric) nature 
of this year's vision target, vision is unable to obtain an absolute robot coordinate, only being able to calculate the 
distance to the hub (generating a ring of possible robot positions). In an ideal world, we could pass this ring into the 
Kalman filter's probability matrix for a more accurate position estimate, but WPILib's `DifferentialDrivePoseEstimator` 
only accepts a `Pose2d` as vision measurement, so we never ended up using vision data to correct odometry.

### Auton ([#22](https://github.com/grt192/GRTCommandBased/pull/22), merged into #27)
Originally, auton consisted of 6 paths: a top, middle, and bottom red sequence, and a top, middle, and bottom blue sequence.
These sequences lowered the intake, pathfollowed from a known starting point to the closest ball, then shot two balls.

<a href="https://pausd.onshape.com/documents/8003f7997b588f082d72c477/w/14e5661479750659f282841e/e/4af5ee78827743d1d23aca5f">
    <p align="center">
        <img src="https://user-images.githubusercontent.com/60120929/164557932-f1ac9991-af44-412d-affb-11d4e6715ccf.png" width="700px">
    </p>
</a>

##### Auton path and ball location document [CAD]

To reuse shared auton logic, all sequences inherited from `GRTAutonSequence` and invoked super constructors to define
their path.
```java
/**
 * Creates a GRTAutonSequence from an initial and ball one pose. This is used for shared initialization logic
 * between all auton paths, as well as one-ball sequences like the top and bottom paths. Drives to the ball, 
 * then shoots twice.
 * 
 * @param robotContainer The RobotContainer instance, for calling `.setInitialPose()`.
 * @param initialPose The initial pose of the sequence.
 * @param ballOnePose The first ball pose of the sequence.
 */
public GRTAutonSequence(RobotContainer robotContainer, Pose2d initialPose, Pose2d ballOnePose) {
    this.robotContainer = robotContainer;
    this.initialPose = initialPose;

    tankSubsystem = robotContainer.getTankSubsystem();
    internalSubsystem = robotContainer.getInternalSubsystem();
    intakeSubsystem = robotContainer.getIntakeSubsystem();

    addRequirements(tankSubsystem, internalSubsystem, intakeSubsystem);

    addCommands(
        new DeployIntakeCommand(intakeSubsystem),
        new FollowPathCommand(tankSubsystem, initialPose, List.of(), ballOnePose)
            .withTimeout(5),
        new ShootCommand(internalSubsystem),
        new ShootCommand(internalSubsystem)
    );
}
```
##### [`GRTAutonSequence` L30-56](https://github.com/grt192/GRTCommandBased/blob/develop/src/main/java/frc/robot/commands/tank/GRTAutonSequence.java#L30-L56)

Auton was not ready for Monterey. Our first practice match, an S-curve trajectory was set as our auton routine (left over from path following testing) and we incurred a penalty during autonomous when we rammed into an opposing alliance bot. Pretty quickly following the first few matches it was decided that there wasn't enough time to tune path following, and that we should instead rewrite auton to be simpler and more reliable.

The revised "pleb" auton lowers the intake, drives forwards for 55 inches or 8 seconds (whichever completed first),
and shoots two balls, using overrides and timers to guarantee shooter and internals cooperate.
```java
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
```
##### [`PlebAutonSequence` L87-125](https://github.com/grt192/GRTCommandBased/blob/develop/src/main/java/frc/robot/commands/tank/PlebAutonSequence.java#L87-L125)

<a href="https://drive.google.com/file/d/12qvF5H1EAqaNPT_eMLM5U1sfSgBAcgTB/view?usp=sharing">
    <p align="center">
        <img src="https://user-images.githubusercontent.com/60120929/165888726-0966e09d-fda9-4a34-809b-5905d93c4098.jpg" width="700px">
    </p>
</a>

##### Two ball pleb auton [VIDEO]

At Monterey, even the revised pleb auton didn't fare so well. Originally, it still relied on the driver-oriented 
`requestShot()` and force-shot-on-second-call override, and it seemed every other match that either internals or the 
flywheel wouldn't recognize the ball and try to fire it. One-ball auton was inconsistent, two-ball auton was practically 
nonexistent due to an uncaught bug that made auton exit after it successfully shot once, and there was much confusion 
around [why the flywheel was "spinning backwards" during auton, or why two-ball auton never seemed to work](https://docs.google.com/document/d/1-vH8RTIFkwTzP3HBMQ2mBGBzPsPOIh3eer36siv3sD0/edit?usp=sharing). 
Though we eventually managed to pinpoint and eliminate most of the issues plaguing auton at Monterey, it took us the whole 
regional to do so.

Given two more weeks to debug and improve logic however, auton at SVR performed a lot more consistently. Though we still
missed shots occasionally due to vision glitches or unplugged motors, two-ball autons were semi-frequent, a welcome
improvement from the frantic code rewrites done in the pit at Monterey.

## Intake ([#16](https://github.com/grt192/GRTCommandBased/pull/16), [#27](https://github.com/grt192/GRTCommandBased/pull/27), [#32](https://github.com/grt192/GRTCommandBased/pull/32))
The intake subsystem [`IntakeSubsystem`](https://github.com/grt192/GRTCommandBased/blob/develop/src/main/java/frc/robot/subsystems/IntakeSubsystem.java)
controls the robot's intake. It uses a SparkMax to control the intake rollers NEO and a Talon to control the 775
deploy mechanism.

### Deployment
To represent the intake's deploy position, `IntakeSubsystem` uses an enum containing the respective encoder tick position 
value.
```java
/**
 * An enum representing the position of the intake, with `IntakePosition.value`
 * representing the counterclockwise angle from straight upwards.
 */
public enum IntakePosition {
    START(0), RAISED(0), DEPLOYED(107891), BOTTOM(107891);

    public double value;

    private IntakePosition(double value) {
        this.value = value;
    }
}
```
##### [`IntakeSubsystem` L31-43](https://github.com/grt192/GRTCommandBased/blob/develop/src/main/java/frc/robot/subsystems/IntakeSubsystem.java#L31-L43)

To set the desired positions, the intake deploy uses a [motion-profiling PID loop](https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html#motion-magic-control-mode) 
to scale up and down the desired velocities at the beginning and end of the trajectory.
```java
deploy.set(ControlMode.MotionMagic, 
    (autoDeployIntake && power > 0.1
        ? IntakePosition.DEPLOYED.value
        : targetPosition.value) 
    + intakePosOffset);
```
##### [`IntakeSubsystem` L216-220](https://github.com/grt192/GRTCommandBased/blob/develop/src/main/java/frc/robot/subsystems/IntakeSubsystem.java#L216-L220)

Initially, we experimented with automatically deploying the intake when drivers ran the rollers and keeping it raised at 
all other times, but the PID proved too slow for such a solution to be effective. Instead, the desired intake position is 
set by driver input and toggled between `RAISED` and `DEPLOYED`.

PID tuning of the intake deploy was rough, possibly because of the mechanical properties of the system; the loop needed to
resist gravity both in going up and from acting too quickly on the way down. The original intake 775 motor had a
broken encoder, something not caught until an early attempt at tuning ran the intake full power into the bottom hard stop.

Initially as well, the tuned PID loop was too slow in bringing the intake up and down and attempts to increase the cruise
velocity and make it faster encountered mechanical issues. For Monterey, a hackier solution using set thresholds and
powers was quickly implemented to raise and lower the intake at more reasonable speeds.
```java
private void moveDeployTo(double targPos) {
    double currentPos = deploy.getSelectedSensorPosition();

    //System.out.println("current: " + currentPos + " targ: " + targPos);
    if (Math.abs(targPos - currentPos) < 2000) {
        deploy.set(0);
    } else if (targPos > currentPos || targPos == IntakePosition.DEPLOYED.value) { // going down
        if (currentPos < 20000) {
            deploy.set(0.1);
        } else {
            deploy.set(0.25);
        }
    } else { // going up
        if (currentPos < 20000) {
            deploy.set(-0.1);
        } else {
            deploy.set(-0.3);
        }
    }
}
```
##### [`IntakeSubsystem` L234-253](https://github.com/grt192/GRTCommandBased/blob/develop/src/main/java/frc/robot/subsystems/IntakeSubsystem.java#L234-L253)

Unlike the PID solution however, this approach was prone to large oscillations (as seen in some Monterey qualifiers).
Thankfully, after the intake gear ratio and chain was mechanically adjusted, the retuned PID was able to achieve "good enough"
speeds and the intake reverted to using motion profiling closed loop control.

### Limit switches
To reset the intake's position to known values when the encoder drifts (and to prevent potentially dangerous PID overshooting), 
the intake features two limit switches, accessed via DIO.
```java
topLimitSwitch = new DigitalInput(tLimitSwitchPort);
bottomLimitSwitch = new DigitalInput(bLimitSwitchPort);
```
##### [`IntakeSubsystem` L123-124](https://github.com/grt192/GRTCommandBased/blob/develop/src/main/java/frc/robot/subsystems/IntakeSubsystem.java#L123-L124)

When testing on robot 1, it was discovered that because the deploy motor and limit switch were on opposite sides of the 
intake and the limit switch side sagged more than the motor side, the deploy motor would move only until the right side 
of the intake activated the limit switch, after which the encoder would be reset to the target position and the PID would 
stop supplying power. <!-- reword "stop supplying power" --> This caused the left side of the intake to visibly hang above 
the right.

<!-- reword probably -->
To address this problem, we added a delay to the limit switch reset where activating a limit switch would only reset the 
position after a fraction of a second, causing the PID to keep moving the motor a bit longer before resetting the deploy 
position.
```java
/**
 * Checks the target limit switch for whether the intake is touching it and resets the encoder on
 * the given delay to prevent uneven tilting on the limit switch side.
 * 
 * @param limitSwitch The limit switch to check.
 * @param timer The timer corresponding to that limit switch.
 * @param delay The delay to reset to the position after, in seconds.
 * @param resetPos The position to reset to, in encoder ticks.
 */
private void delayLimitSwitchReset(DigitalInput limitSwitch, Timer timer, double delay, double resetPos) {
    if (!limitSwitch.get()) {
        timer.start();
    } else {
        timer.stop();
        timer.reset();
    }

    if (timer.hasElapsed(delay)) {
        deploy.setSelectedSensorPosition(resetPos);
        intakePosOffset = 0;
    }
}
```
##### [`IntakeSubsystem` L256-277](https://github.com/grt192/GRTCommandBased/blob/develop/src/main/java/frc/robot/subsystems/IntakeSubsystem.java#L256-L277)

```java
delayLimitSwitchReset(bottomLimitSwitch, bottomTimer, BOTTOM_LIMIT_DELAY, IntakePosition.BOTTOM.value);
// TODO: is a top delay even necessary?
delayLimitSwitchReset(topLimitSwitch, topTimer, TOP_LIMIT_DELAY, IntakePosition.START.value);
```
##### [`IntakeSubsystem` L166-168](https://github.com/grt192/GRTCommandBased/blob/develop/src/main/java/frc/robot/subsystems/IntakeSubsystem.java#L166-L168)

Following intake's mechanical modifcations after Monterey, we started to run into the opposite problem: the motor side 
was being driven below the limit switch side, causing balls to bounce out on the left. Initially, to rectify this we split 
the `DEPLOYED` position into two: `BOTTOM` was set to the position at the hard stop, what the limit switch would reset to, 
and `DEPLOYED` would be slightly above that to keep the intake raised enough so that balls could still be consumed.
```java
START(0), RAISED(0), DEPLOYED(107891), BOTTOM(107891);
```
##### [`IntakeSubsystem` L36](https://github.com/grt192/GRTCommandBased/blob/develop/src/main/java/frc/robot/subsystems/IntakeSubsystem.java#L36)

After a few days of testing this problem miraculously fixed itself and we never ended up employing a different value for 
`BOTTOM` than `DEPLOYED`.

Similarly, `START` and `RAISED` were originally intended to be separate values: start would be the position of the intake 
all the way up (within the frame perimeter) at the start of the match, and raised would be a value slightly in front of 
that position to not collide with the turret mid-match. After the gear ratio and chain were changed however, it was 
discovered that there was enough slack in the chain such that when the motor tried to return to `0` it would return to a 
satisfactory raised position instead.

### Rollers
The intake rollers are controlled using overrides supplied by driver input and auton commands.
```java
// Otherwise, use auto input if it's overriding, driver input if they're overriding 
// and default to running intake automatically from vision.
power = autoOverride ? autoPower
    : driverOverride ? driverPower
    : jetson.ballDetected() ? 0.5 : 0;
```
##### [`IntakeSubsystem` L176-180](https://github.com/grt192/GRTCommandBased/blob/develop/src/main/java/frc/robot/subsystems/IntakeSubsystem.java#L176-L180)

The rollers depend on a ball count from internals, and won't run if internals is already full.
```java
// If the ball count is greater than 2, don't run intake.
// Skip this check if disabled on shuffleboard.
double power;
if (internalSubsystem.getBallCount() > 2 && !skipInternalsCheck) {
    power = 0;
} else {
```
##### [`IntakeSubsystem` L170-175](https://github.com/grt192/GRTCommandBased/blob/develop/src/main/java/frc/robot/subsystems/IntakeSubsystem.java#L170-L175)

Initially, we experimented with using vision to detect balls and running rollers automatically from vision data, with 
driver input only designed to override the rollers if the detection failed. However, due to time constraints forcing the 
abandoning of vision ball detection, the driver override became the main way the rollers were controlled.
```java
public RunIntakeCommand(IntakeSubsystem intakeSubsystem, XboxController xboxController) {
    super(() -> {
        double leftTrigger = xboxController.getLeftTriggerAxis();
        double rightTrigger = xboxController.getRightTriggerAxis();

        intakeSubsystem.setDriverPower((rightTrigger - leftTrigger) * 0.85);

        // If the driver is supplying intake power with the triggers, override intake control
        if (leftTrigger > 0 || rightTrigger > 0) {
            intakeSubsystem.setDriverOverride(true);

            overrideTimer.reset();
            overrideTimer.start();
        } else if (overrideTimer.hasElapsed(2)) {
            // Otherwise, if 2 seconds have passed without driver input, resume automatic intake control
            intakeSubsystem.setDriverOverride(false);
        }

        intakeSubsystem.intakePosOffset += -xboxController.getLeftY() * 100;
    }, intakeSubsystem);
}
```
##### [`RunIntakeCommand` L18-38](https://github.com/grt192/GRTCommandBased/blob/develop/src/main/java/frc/robot/commands/intake/RunIntakeCommand.java#L18-L38); the timer to relinquish driver control became largely irrelevant after ditching intake vision.

Running the intake was bound to the mech controller, with the right trigger running the rollers forwards (to intake balls) 
and the left trigger running the rollers backwards (to reject balls).

## Turret ([#16](https://github.com/grt192/GRTCommandBased/pull/16), [#27](https://github.com/grt192/GRTCommandBased/pull/27), [#32](https://github.com/grt192/GRTCommandBased/pull/32))
The turret subsystem [`TurretSubsystem`](https://github.com/grt192/GRTCommandBased/blob/develop/src/main/java/frc/robot/subsystems/TurretSubsystem.java)
manages the robot's turret shooter hub locking and aiming. To aim the turret,`TurretSubsystem` maintains a hub distance
(`r`) and turntable angle (`theta`) to constantly align the turret with the hub and set the correct hood position and
flywheel velocity to score shots.

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

For smoother hub tracking and possibly even shooting while moving, the turret attempts to roughly predict future system 
states using feedforward. Instead of calculating the absolute `r` and `theta`, the turret calculates the changes in system 
state, `delta r` and `delta theta`.
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

When vision is out of range, these deltas are continuously added to the current state values to update them.
```java
Pair<Double, Double> deltas = calculateRThetaDeltas(previousPosition, currentPosition);

// If the hub is in vision range and the jetson has fresh data, use vision's `r` and `theta` as ground truth.
// While the flywheel is running, use the manual system values instead to prevent camera issues while the 
// flywheel shakes the turntable.
// If the jetson has been disabled on shuffleboard, use `rtheta` instead.
boolean jetsonWorking = jetson.turretVisionWorking();
if (jetsonWorking && (!runFlywheel || mode == TurretMode.RETRACTED) && !jetson.getConsumed() && !jetsonDisabled) {
    Pair<Double, Double> data = jetson.getData();
    r = data.getFirst();
    theta = Math.PI + data.getSecond() - turntableEncoder.getPosition();

    // Reset theta offset when we refresh rtheta from vision.
    turntableOffset = 0;
} else {
    // Otherwise, update our `r` and `theta` state system from the previous `r` and
    // `theta` values and the delta X and Y since our last position. We do this instead of using
    // raw odometry coordinates to prevent error accumulation over time; `r` and `theta` are 
    // reset to vision values when vision is in range, so our states only accumulate error while 
    // vision is out of range (as opposed to odometry, which accumulates error throughout the match).
    r += deltas.getFirst();
    theta += deltas.getSecond();
}
```
##### [`TurretSubsystem` L370-392](https://github.com/grt192/GRTCommandBased/blob/develop/src/main/java/frc/robot/subsystems/TurretSubsystem.java#L370-L392)

Regardless of whether vision is in range, the rtheta deltas from localization are used to calculate the immediate rate 
of change of the `r` and `theta` system states. These rates are then multiplied by tuned constants to predict the future 
state of the system.
```java
/**
 * Updates `r` and `theta, applying feedforward to new `rtheta` values from manual calculation.
 * @param states The new `rtheta` states, as a pair of [dr (in), dtheta (rads)].
 */
private void applyRThetaFeedForward(Pair<Double, Double> deltas) {
    double deltaR = deltas.getFirst(), deltaTheta = deltas.getSecond();

    double newTimestamp = Timer.getFPGATimestamp();
    double deltaLoopTime = newTimestamp - previousLoopTime;

    // Velocities in in/s and radians/s respectively
    double rVel = deltaR / deltaLoopTime;
    double thetaVel = deltaTheta / deltaLoopTime;

    rFeedForward = r + rVel * R_FF;
    thetaFeedForward = theta + thetaVel * THETA_FF;
    previousLoopTime = Timer.getFPGATimestamp();
}
```
##### [`TurretSubsystem` L498-515](https://github.com/grt192/GRTCommandBased/blob/develop/src/main/java/frc/robot/subsystems/TurretSubsystem.java#L498-L515)

The advantages of feedforward (as opposed to feed*back*) control are that, if only feedback is used, systems like the 
turntable or flywheel will always be behind while the robot is moving; as robot's position changes, the setpoint passed 
to the turntable will continuously change as well, causing the turntable to keep having to rotate to catch up to a setpoint 
which has already changed by the time the turntable has reached it. By using feedforward, the turntable can theoretically 
spend `x` seconds turning to the exact position needed of it `x` seconds in the future and arrive exactly on time at its 
desired position.

Of course, we never had the time to tune these feedforward constants beyond making turntable tracking while moving slightly
smoother; half a shoptime was never going to be enough to attempt shooting while moving. Still, while the rtheta 
system was not a replacement for vision on the competition field (as demonstrated at Monterey), it performed well in short, 
collisionless time intervals, consistently scoring shots when testing in the shop.

### Interpolation
To find the flywheel speed and hood angle that would work for the turret's current hub distance, `TurretSubsystem`
linearly interpolates flywheel RPM and hood degrees from known manual testing data points, recorded in the following
[spreadsheet](https://docs.google.com/spreadsheets/d/1hAsBn0KIxucuwOv96UMS3bZOrim03YxqXL-2-pBrjzI/edit?usp=sharing).

<p align="center">
    <img src="https://user-images.githubusercontent.com/60120929/162670097-63f1183c-8f15-473d-a90d-0830b2b6153e.png" height="300px"> <img src="https://user-images.githubusercontent.com/60120929/162670129-8a0280be-02f1-44d2-b035-4bf4287bdeaf.png" height="300px">
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

### PID and Flywheel Logic
When the desired hood, flywheel, and turntable references are calculated from `rtheta` and interpolation, the motors are
set using PID control. The flywheel NEO is controlled with `kVelocity` PID in units of flywheel RPM (converting from NEO
to flywheel RPM with the gear ratio) and the turntable with `kSmartMotion` in units of radians. Talons don't support the
conversion factor API, but the hood uses `Position` PID with references converted from radians to ticks before being set.
```java
flywheelPidController.setReference(runFlywheel ? desiredFlywheelRPM : 0, ControlType.kVelocity);
turntablePidController.setReference(desiredTurntableRadians, ControlType.kSmartMotion);
hood.set(ControlMode.Position, desiredHoodRadians * HOOD_RADIANS_TO_TICKS);
```
##### [`TurretSubsystem` L423-425](https://github.com/grt192/GRTCommandBased/blob/develop/src/main/java/frc/robot/subsystems/TurretSubsystem.java#L423-L425)

The flywheel cannot be run all the time however; it is loud, consumes power, and more importantly vibrates the entire
turntable and distorts the vision camera stream while running. Instead, the flywheel only runs while a ball is ready in
internals and after 0.5 seconds have elapsed after stopping driving.
```java
boolean runFlywheel = (drivingTimer.hasElapsed(0.5) && ballReady) || driverOverrideFlywheel;
```
##### [`TurretSubsystem` L363](https://github.com/grt192/GRTCommandBased/blob/develop/src/main/java/frc/robot/subsystems/TurretSubsystem.java#L363)
```java
/**
 * Sets whether the robot is currently being driven by controller input.
 * @param driving Whether the robot is being driven.
 */
public void setDriving(boolean driving) {
    if (!driving) {
        drivingTimer.start();
    } else {
        drivingTimer.stop();
        drivingTimer.reset();
    }
    //this.driving = driving;
}
```
##### [`TurretSubsystem` L683-695](https://github.com/grt192/GRTCommandBased/blob/develop/src/main/java/frc/robot/subsystems/TurretSubsystem.java#L683-L695)

Originally, `driving` was a boolean which was set to false immediately after controller input ceased. This, however,
meant that the flywheel would turn on as soon as the robot stopped, killing vision before it could lock on. A timer was
added to add a delay before enabling the flywheel and give vision more time to detect the hub.

<a href="https://drive.google.com/file/d/1y6PzbTDGo8iuTLlkWDuZbjf7fBNy__x3/view?usp=sharing">
    <p align="center">
        <img src="https://user-images.githubusercontent.com/60120929/165888736-64c7dcb3-ffa0-4713-9c92-92c5ad22727d.jpg" height="600px">
    </p>
</a>

##### Early flywheel logic testing [VIDEO]

### Module Alignment
To represent whether a module (hood, flywheel, turntable) is aligned and ready to shoot, their current position or velocity
is thresholded against the target value and converted into a `ModuleState` enum representing whether they are in `HIGH_TOLERANCE`
alignment, `LOW_TOLERANCE` alignment, or completely unaligned.
```java
public enum ModuleState {
    HIGH_TOLERANCE, LOW_TOLERANCE, UNALIGNED;

    @Override
    public String toString() {
        switch (this) {
            case HIGH_TOLERANCE: return "HIGH_TOLERANCE";
            case LOW_TOLERANCE: return "LOW_TOLERANCE";
            case UNALIGNED: return "UNALIGNED";
        }
        return "UNKNOWN";
    }
}
```
##### [`TurretSubsystem` L69-81](https://github.com/grt192/GRTCommandBased/blob/develop/src/main/java/frc/robot/subsystems/TurretSubsystem.java#L69-L81)
```java
/**
 * Gets the state of the turntable (whether it is aligned to the hub). 
 * @return The state of the turntable.
 */
private ModuleState turntableAligned() {
    // Thresholding in units of radians
    double diffRads = Math.abs(turntableEncoder.getPosition()
        - (MANUAL_CONTROL ? Math.toRadians(turntableRefPos) : desiredTurntableRadians));
    return diffRads < Math.toRadians(2.5) ? ModuleState.HIGH_TOLERANCE
        : diffRads < Math.toRadians(7.5) ? ModuleState.LOW_TOLERANCE
        : ModuleState.UNALIGNED;
}
```
##### [`TurretSubsystem` L622-633](https://github.com/grt192/GRTCommandBased/blob/develop/src/main/java/frc/robot/subsystems/TurretSubsystem.java#L622-L633), similar for `hoodReady()` and `flywheelReady()`

To represent the turret's readiness to shoot, the lowest module state is used as the state of the system.

The idea behind maintaining a state enum instead of a simple "ready" boolean was that, while a shot that the robot is trying to
score requires a tight alignment tolerance for the turntable, hood, and flywheel, a ball that is being *rejected* can be
fired much sooner and without having to wait for the flywheel to spin up all the way or the turntable to reach perfect
alignment with the hub. Therefore, a rejected ball could be shot while the subsystem state was `LOW_TOLERANCE`, while a
ball intended to be scored needed to wait for `HIGH_TOLERANCE`.

An issue with thresholding was lack of testing time. During Monterey, many shots we took were waiting forever for too-tight
tolerances and had to be forced. We repeatedly blanket-increased the tolerances in response without the time to test if
a ball fired when the flywheel was 150 RPM off would still make it in the hub. [...]

### Turret Mode and Rejection
`TurretSubsystem` maintains a `mode` enum representing what mode it is in and how it should behave.
```java
public enum TurretMode {
    SHOOTING, LOW_HUB, REJECTING, RETRACTED;

    @Override
    public String toString() {
        switch (this) {
            case SHOOTING: return "SHOOTING";
            case REJECTING: return "REJECTING";
            case LOW_HUB: return "LOW_HUB";
            case RETRACTED: return "RETRACTED";
        }
        return "UNKNOWN";
    }
}
```
##### [`TurretSubsystem` L46-59](https://github.com/grt192/GRTCommandBased/blob/develop/src/main/java/frc/robot/subsystems/TurretSubsystem.java#L46-L59)

In `SHOOTING` mode, the turret behaves as normal, continuously tracking the hub while positioning the flywheel and hood
to score shots in the hub.

In `RETRACTED` mode, the turret retracts itself, setting the hood and flywheel reference to 0 and the turntable to 180
degrees.
```java
// If retracted, skip interpolation calculations
if (mode == TurretMode.RETRACTED) {
    desiredTurntableRadians = Math.toRadians(180);
    desiredHoodRadians = 0;
    desiredFlywheelRPM = 0;
} else {
```
##### [`TurretSubsystem` L397-402](https://github.com/grt192/GRTCommandBased/blob/develop/src/main/java/frc/robot/subsystems/TurretSubsystem.java#L397-L402)

<!-- citation needed on usage -->
This was originally created to retract everything before climb, but ended up as a power conservation feature for playing
defense as well as a method to get everything back to their starting positions before a code redeploy.

`LOW_HUB` was a mode quickly added before Monterey as a failsafe in case upper hub interpolation stopped working during
a match. `LOW_HUB` functions similarly to `RETRACTED`, except instead of setting references back to their starting positions,
references are set to static values for "dumping" balls into the lower hub from the hub wall.
```java
// TODO tune
if (this.mode == TurretMode.LOW_HUB) {
    desiredFlywheelRPM = 3000;
    desiredHoodRadians = Math.toRadians(16);
    desiredTurntableRadians = Math.toRadians(180);
}
```
##### [`TurretSubsystem` L408-413](https://github.com/grt192/GRTCommandBased/blob/develop/src/main/java/frc/robot/subsystems/TurretSubsystem.java#L408-L413)

The toggle for `LOW_HUB` mode was bound to the mech controller's Y button, but dumping was not attempted often by drivers.

In `REJECTING` mode, the turret scales down the flywheel RPM by 0.5 during interpolation to intentionally miss wrong-colored
balls (but still cause them to bounce and be difficult to intake). This mode would be set by internals if it detected that
the current ball color did not match the current alliance color.
```java
if (mode == TurretMode.REJECTING && !SKIP_REJECTION) desiredFlywheelRPM *= 0.5;
```
##### [`TurretSubsystem` L543](https://github.com/grt192/GRTCommandBased/blob/develop/src/main/java/frc/robot/subsystems/TurretSubsystem.java#L543)

Rejection logic was heavily untested, and remained disabled for the majority of Monterey and the entirety of SVR. Though
it may not have directly caused problems, disabling it allowed further isolation of where a problem originated. As we didn't
end up making many shots (or even intaking wrong colored balls at all), rejection logic didn't end up being all too necessary.

In retrospect as well, rejection could have been made a separate boolean which was ignored in `LOW_HUB` and `RETRACTED`,
as it was basically just `SHOOTING` mode with an extra step in interpolation.
```java
/**
 * Set whether the turret should reject the current ball.
 * @param reject Whether to reject the ball.
 */
public void setReject(boolean reject) {
    // Don't do anything if the turret is retracted
    if (mode == TurretMode.RETRACTED) return;
    if (mode == TurretMode.LOW_HUB) return;
    mode = reject ? TurretMode.REJECTING : TurretMode.SHOOTING;
}
```
##### [`TurretSubsystem` L552-561](https://github.com/grt192/GRTCommandBased/blob/develop/src/main/java/frc/robot/subsystems/TurretSubsystem.java#L552-L561), not the prettiest code.

### Overrides
Because running without vision quickly accumulated `rtheta` error and faulty values could potentially render the robot 
unable to score for the rest of the match, several overrides were put in place for drivers to wrest control of shooting 
from `rtheta` autoaiming.

When pressed, the drive controller's right bumper resets the turret's `r` to 140in. (approximately the distance to the 
hub at the edge of the tarmac).
```java
driveRBumper.whenPressed(new InstantCommand(() -> turretSubsystem.setR(140)));
```
##### [`RobotContainer` L170](https://github.com/grt192/GRT2022/blob/develop/src/main/java/frc/robot/RobotContainer.java#L170)

The drive controller X button toggles `RETRACTED` mode, initially intended to clean up before climb but used during 
competitions as a way to lock the turret while the driver rotated the robot to regain vision data.
```java
driveXButton.whenPressed(new InstantCommand(turretSubsystem::toggleClimb));
```
##### [`RobotContainer` L169](https://github.com/grt192/GRT2022/blob/develop/src/main/java/frc/robot/RobotContainer.java#L169)

Counteracting the error accumulated by `rtheta` could be attempted by applying offsets to `r` and `theta`, bound to the 
mech controller's POV (4 directional arrow buttons arranged in a cross). The `r` offset could be changed with the top and 
bottom buttons (0 and 180 degrees respectively), and the `theta` offset with the left and right buttons (90 and 270 degrees 
respectively).
```java
// Set turret offsets from mech controller POV input:
// Top/bottom to increase/decrease hub distance offset,
// right/left to increase/decrease theta offset.
turretSubsystem.setDefaultCommand(new RunCommand(() -> {
    switch (mechController.getPOV()) {
        case 0: turretSubsystem.changeDistanceOffset(1.5); break;
        case 90: turretSubsystem.changeTurntableOffset(Math.toRadians(3)); break;
        case 180: turretSubsystem.changeDistanceOffset(-1.5); break;
        case 270: turretSubsystem.changeTurntableOffset(Math.toRadians(-3)); break;
        default: break;
    }

    turretSubsystem.setFreeze(driveController.getLeftTriggerAxis() > 0.2);
}, turretSubsystem));
```
##### [`RobotContainer` L204-217](https://github.com/grt192/GRT2022/blob/develop/src/main/java/frc/robot/RobotContainer.java#L204-L217)

Finally, the turret could be frozen entirely with the drive controller's left trigger. The turret saves the values of 
`r` and `theta` when the trigger is initially pressed and references the "frozen" values instead of the new states while 
the trigger is held.
```java
public void setFreeze(boolean frozen) {
    if (frozen && !this.frozen) {
        this.frozenR = this.rFeedForward;
        this.frozenTheta = this.thetaFeedForward;
    }
    this.frozen = frozen;
}
```
##### [`TurretSubsystem` L760-766](https://github.com/grt192/GRT2022/blob/develop/src/main/java/frc/robot/subsystems/TurretSubsystem.java#L760-L766)

Putting it all together was a system that relied on vision for absolute-truth data, generated the data itself if vision 
couldn't be used, and could at any time be overridden by drivers if either vision or `rtheta` was acting up (something 
that admittedly happened all too often).

<a href="https://drive.google.com/file/d/1ZiiMFkPfQoj4i31D5cdbZt8RWSbFmb5o/view?usp=sharing">
    <p align="center">
        <img src="https://user-images.githubusercontent.com/60120929/178643043-d173e027-26ff-473e-bab1-79de668fc637.jpg" width="700px">
    </p>
</a>

##### Post-Monterey driver practice [VIDEO]

[...]

### Other features (lazy tracking, debug flags)
While internals doesn't have a ball ready to shoot, the turntable can be less aggressive in staying aligned. To accomplish 
this, the maximum output power of the turntable PID is set every loop depending on whether a ball is ready.
```java
// Set turntable lazy tracking if a ball isn't ready
double pow = !ballReady ? 0.25 : 0.5;
turntablePidController.setOutputRange(-pow, pow);
```
##### [`TurretSubsystem` L366-368](https://github.com/grt192/GRTCommandBased/blob/develop/src/main/java/frc/robot/subsystems/TurretSubsystem.java#L366-L368)

While this wasn't isolated to just the turret, a feature that made testing and feature isolation / toggling much easier 
was debug flags. Instead of repeatedly commenting and uncommenting code, static booleans could be toggled to quickly 
disable untested or problematic logic or expose debug interfaces like prints or shuffleboard entries.
```java
// Whether rtheta (`r`, `theta`, `dx`, `dy`, `dtheta`, `alpha`, `beta`, `x`, `y`, `h`) system states 
// should be printed.
private static boolean PRINT_STATES = false;
// Whether current system tolerances (flywheel, hood, turntable) should be printed.
private static boolean PRINT_TOLERANCES = false;
// Whether PID tuning shuffleboard entries should be displayed.
private static boolean DEBUG_PID = true;
// Whether rtheta logic should be skipped and the turntable, hood, and flywheel references 
// should be manually set through shuffleboard.
private static boolean MANUAL_CONTROL = false;
// Whether the turret should fire at full speed regardless of rejection logic.
private static boolean SKIP_REJECTION = true;
```
##### [`TurretSubsystem` L195-206](https://github.com/grt192/GRTCommandBased/blob/develop/src/main/java/frc/robot/subsystems/TurretSubsystem.java#L195-L206)

## Vision
[...]

## Internals
[...]
