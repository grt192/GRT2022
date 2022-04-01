package frc.robot.subsystems.internals;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

import frc.robot.GRTSubsystem;
import frc.robot.brownout.PowerController;
import frc.robot.shuffleboard.GRTNetworkTableEntry;
import frc.robot.shuffleboard.GRTShuffleboardTab;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.TurretSubsystem.ModuleState;
import frc.robot.subsystems.TurretSubsystem.TurretMode;

import static frc.robot.Constants.InternalConstants.*;

public class InternalSubsystem extends GRTSubsystem {

    private final TurretSubsystem turretSubsystem;
    private final ColorSensorThread colorSensorThread;

    private final WPI_TalonSRX motorBottom;
    private final WPI_TalonSRX motorTop;

    private final AnalogPotentiometer entrance;
    private final ColorSensorV3 storage;
    private final AnalogPotentiometer staging;

    private final Color RED = new Color(0.437255859375, 0.394775390625, 0.16845703125);
    private final Color BLUE = new Color(0.170654296875, 0.4189453125, 0.41064453125);
    private final Color EMPTY = new Color(0.272216796875, 0.48291015625, 0.2451171875);
    private final Color ALLIANCE_COLOR;

    private final ColorMatch colorMatcher;

    private final Timer entranceTimer;
    private final Timer exitTimer;
    private final Timer storageTimer;

    private boolean entranceStorageBall = false;
    private boolean storageStagingBall = false;
    private boolean stagingExitBall = false;
    private int ballCount = 0;

    private boolean shotRequested = false;
    private boolean rejecting = false;
    private boolean rejectingChecked = false;
    private boolean skipToleranceCheck = false;

    // Shuffleboard
    private final GRTShuffleboardTab shuffleboardTab;
    private final GRTNetworkTableEntry ballCountEntry, shotRequestedEntry, skipToleranceEntry;
    private final GRTNetworkTableEntry 
        entranceEntry, entranceStorageEntry, storageEntry, storageStagingEntry, stagingEntry, stagingExitEntry;
    private final GRTNetworkTableEntry entranceRawEntry, storageRawEntry, stagingRawEntry;

    public InternalSubsystem(TurretSubsystem turretSubsystem) {
        super(15);

        this.turretSubsystem = turretSubsystem;

        // Initialize bottom motor
        motorBottom = new WPI_TalonSRX(motorPortBottom);
        motorBottom.configFactoryDefault();
        motorBottom.setInverted(true);
        motorBottom.setNeutralMode(NeutralMode.Brake);

        // Initialize top motor
        motorTop = new WPI_TalonSRX(motorPortTop);
        motorTop.configFactoryDefault();
        motorTop.setInverted(true);
        motorTop.setNeutralMode(NeutralMode.Brake);

        // Initialize sensors
        entrance = new AnalogPotentiometer(entranceIRPort);
        storage = new ColorSensorV3(I2C.Port.kMXP);
        staging = new AnalogPotentiometer(stagingIRPort);

        colorSensorThread = new ColorSensorThread(storage);

        colorMatcher = new ColorMatch();
        colorMatcher.addColorMatch(RED);
        colorMatcher.addColorMatch(BLUE);
        colorMatcher.addColorMatch(EMPTY);
        colorMatcher.setConfidenceThreshold(0.8);

        entranceTimer = new Timer();
        storageTimer = new Timer();
        exitTimer = new Timer();

        // Set alliance color from FMS
        switch (DriverStation.getAlliance()) {
            case Red: ALLIANCE_COLOR = RED; break;
            case Blue: ALLIANCE_COLOR = BLUE; break;
            default: ALLIANCE_COLOR = RED; break;
        }

        // Shuffleboard
        shuffleboardTab = new GRTShuffleboardTab("Internals");
        entranceEntry = shuffleboardTab.addEntry("Entrance", entrance.get() >= 0.1).at(0, 0);
        entranceStorageEntry = shuffleboardTab.addEntry("Entrance -> storage", entranceStorageBall).at(1, 0);
        storageEntry = shuffleboardTab.addEntry("Storage", isBall(colorSensorThread.getLastStorage())).at(2, 0);
        storageStagingEntry = shuffleboardTab.addEntry("Storage -> staging", storageStagingBall).at(3, 0);
        stagingEntry = shuffleboardTab.addEntry("Staging", staging.get() >= 0.2).at(4, 0);
        stagingExitEntry = shuffleboardTab.addEntry("Staging -> exit", stagingExitBall).at(5, 0);

        ballCountEntry = shuffleboardTab.addEntry("Ball count", ballCount).at(0, 2);
        shotRequestedEntry = shuffleboardTab.addEntry("Shot requested", shotRequested).at(1, 2);
        skipToleranceEntry = shuffleboardTab.addEntry("Skip tolerance", skipToleranceCheck).at(2, 2);

        entranceRawEntry = shuffleboardTab.addEntry("Entrance raw", entrance.get()).at(0, 1);
        storageRawEntry = shuffleboardTab.addEntry("Storage color", colorToString(matchColor(colorSensorThread.getLastStorage()))).at(2, 1);
        stagingRawEntry = shuffleboardTab.addEntry("Staging raw", staging.get()).at(4, 1);
    }

    @Override
    public void periodic() {
        // Get states from IR and color sensors
        double entranceRaw = entrance.get();
        double stagingRaw = staging.get();

        boolean entranceDetected = entranceRaw >= 0.1;
        Color storageColor = matchColor(colorSensorThread.getLastStorage());
        boolean storageDetected = isBall(storageColor);
        boolean stagingDetected = stagingRaw >= 0.2;

        // Display system state on shuffleboard
        entranceStorageEntry.setValue(entranceStorageBall);
        entranceEntry.setValue(entranceDetected);
        storageEntry.setValue(storageDetected);
        storageStagingEntry.setValue(storageStagingBall);
        stagingEntry.setValue(stagingDetected);
        stagingExitEntry.setValue(stagingExitBall);
        shotRequestedEntry.setValue(shotRequested);
        skipToleranceEntry.setValue(skipToleranceCheck);
        entranceRawEntry.setValue(entranceRaw);
        storageRawEntry.setValue(colorToString(storageColor));
        stagingRawEntry.setValue(stagingRaw);

        // If there is a ball in the entrance, run the bottom motor.
        if (entranceDetected && !storageDetected) {
            motorBottom.set(0.3);
            entranceTimer.start();
            entranceStorageBall = true;
        }

        // If 5 seconds have elapsed or the ball has progressed past entrance, stop the bottom motor
        if ((storageDetected || entranceTimer.hasElapsed(5)) && entranceTimer.get() > 0) {
            motorBottom.set(0);
            entranceTimer.stop();
            entranceTimer.reset();
            entranceStorageBall = false;
        }

        // If there is a ball between storage and staging and staging is empty, run the top and bottom motors
        if (storageDetected && !stagingDetected && !stagingExitBall) {
            // Spin the bottom and top motors on a timer
            storageTimer.start();
            motorTop.set(0.5);
            motorBottom.set(0.3);
            storageStagingBall = true;

            if (!rejectingChecked) {
                rejecting = storageColor != ALLIANCE_COLOR;
                rejectingChecked = true;
            }
        }

        // If 0.5 seconds have elapsed or staging has detected the ball, stop the motors
        if (storageTimer.hasElapsed(0.5)) {
            motorTop.set(0);
            motorBottom.set(0);
            storageTimer.stop();
            storageTimer.reset();
            storageStagingBall = false;
        }

        // If there is a ball in staging, we don't want to push it into turret, especially if there is a shot going
        if (stagingDetected) {
            motorTop.set(0);
            storageStagingBall = false;
        }

        // If a shot was requested and the turret is ready, load a ball into the turret.
        // For high goal shots, require that the turret be in high tolerance alignment.
        // Otherwise, if we're rejecting or going into the low goal, low tolerance
        // alignment is fine.
        turretSubsystem.setReject(rejecting);
        ModuleState turretState = turretSubsystem.getState();
        if (shotRequested && (stagingDetected || skipToleranceCheck) && (skipToleranceCheck 
            || turretState == ModuleState.HIGH_TOLERANCE
            || ((rejecting || turretSubsystem.getMode() == TurretMode.LOW_HUB) && turretState == ModuleState.LOW_TOLERANCE))
        ) {
            // Spin the top motor on a timer
            exitTimer.start();
            motorTop.set(0.5);
            // TEST stagingExitBall = true;
        }

        /*if (stagingExitBall) {
            motorTop.set(0.5);
        }
        */

        // shot requested, timer starts, while ball is in staging timer keeps 
        // starting, when ball exits staging 0.5
        // seconds after shot is finished, if in that time period 0<t<0.5 
        // another ball is detected, timer *should* reset
        // however if there is still 1 or 0 balls in internals, shot is 
        // finished and timer will not start again if ball is detected

        // If 0.5 seconds have elapsed, mark the shot as finished
        if (exitTimer.hasElapsed(0.5)) {
            exitTimer.stop();
            exitTimer.reset();
            motorTop.set(0);

            // Reset states
            // If the only ball in the system is the one we just shot, mark the shot as completed
            if (ballCount < 1) shotRequested = false;
            rejectingChecked = false;
            skipToleranceCheck = false;
            // TEST stagingExitBall = false;
        }

        // Set the ball count from staging, storage, and the transition state booleans.
        ballCount = (entranceStorageBall ? 1 : 0)
            + (storageDetected ? 1 : 0)
            + (storageStagingBall ? 1 : 0)
            + (stagingDetected ? 1 : 0) ;
            // TEST + (stagingExitBall ? 1 : 0);
        ballCountEntry.setValue(ballCount);

        turretSubsystem.setBallReady(ballCount > 0);
    }

    /**
     * Temporary testing function to run the internals motors at a set power.
     * @param pow The power to run the bottom roller at.
     */
    public void setPower(double pow) {
        motorBottom.set(pow);
        motorTop.set(pow);
    }

    /**
     * Request that a ball be loaded and shot.
     * The ball will *actually* be shot when the turret is aimed and ready.
     * Calling this while a shot is already requested will tell internals
     * to force a shot by skiping the tolerance check.
     */
    public void requestShot() {
        if (shotRequested) skipToleranceCheck = true;
        shotRequested = true;
    }

    /**
     * Gets whether a shot has been requested.
     * @return Whether a shot has been requested.
     */
    public boolean getShotRequested() {
        return shotRequested;
    }

    /**
     * Gets how many total balls are inside internals.
     * @return The current ball count.
     */
    public int getBallCount() {
        return ballCount;
    }

    /**
     * Normalizes a raw color from a color sensor to the closest stored color in colorMatcher using a tolerance of 0.8, 
     * returning `null` if no color was matched.
     * @param color The color to match.
     * @return The normalized color (RED, BLUE, EMPTY, or null if no match).
     */
    private Color matchColor(Color color) {
        ColorMatchResult match = colorMatcher.matchColor(color);
        if (match == null) return null;
        return match.color;
    }

    /**
     * Checks whether a normalized color sensor color represents a ball being detected.
     * @param detected The color to check.
     * @return Whether a ball (of any color) has been detected.
     */
    private boolean isBall(Color detected) {
        return detected == RED || detected == BLUE;
    }

    /**
     * Utility function for pretty-printing matched Color objects.
     * @param c The matched color to pretty-print.
     * @return The name of the color (RED, BLUE, EMPTY, NULL).
     */
    private String colorToString(Color c) {
        return c == RED ? "RED" 
            : c == BLUE ? "BLUE" 
            : c == EMPTY ? "EMPTY" 
            : "NULL";
    }

    @Override
    public double getTotalCurrentDrawn() {
        return PowerController.getCurrentDrawnFromPDH(motorPortBottom, motorPortTop);
    }

    @Override
    public void setCurrentLimit(double limit) {
        int motorLimit = (int) Math.floor(limit / 2);

        motorBottom.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, motorLimit, 0, 0));
        motorTop.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, motorLimit, 0, 0));
    }
}
