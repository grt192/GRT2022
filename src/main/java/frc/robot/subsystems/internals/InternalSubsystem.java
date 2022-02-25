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
import frc.robot.subsystems.TurretSubsystem;

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

    private final Timer exitTimer;
    //private final Timer entranceTimer;
    private final Timer storageTimer;

    private int ballCount = 0;

    private boolean prevEntranceDetected = false;
    private boolean entToStorage = false;

    private boolean shotRequested = false;
    private boolean rejecting = false;
    private boolean rejectingChecked = false;

    private boolean driverOverride = false;

    public InternalSubsystem(TurretSubsystem turretSubsystem) {
        super(15);

        this.turretSubsystem = turretSubsystem;

        // Initialize bottom motor
        motorBottom = new WPI_TalonSRX(motorPortBottom);
        motorBottom.configFactoryDefault();
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

        exitTimer = new Timer();
        //entranceTimer = new Timer();
        storageTimer = new Timer();

        // Set alliance color from FMS
        switch (DriverStation.getAlliance()) {
            case Red: ALLIANCE_COLOR = RED; break;
            case Blue: ALLIANCE_COLOR = BLUE; break;
            default: ALLIANCE_COLOR = RED; break;
        }

        // TODO: it probably isn't great to do this in the constructor? probably hard code the ball count
        Color storageColor = matchColor(colorSensorThread.getLastStorage());
        boolean storageDetected = isBall(storageColor);
        boolean stagingDetected = staging.get() >= 0.2;
        ballCount = (stagingDetected ? 1 : 0) + (storageDetected ? 1 : 0);
    }

    @Override
    public void periodic() {
        // If a new ball has entered internals, increment ball count
        // TODO: either we keep this or use storage and staging to set ball count; currently this is redundant
        boolean entranceDetected = entrance.get() >= 0.4;
        if (!prevEntranceDetected && entranceDetected) ballCount++;
        prevEntranceDetected = entranceDetected;

        Color storageColor = matchColor(colorSensorThread.getLastStorage());
        boolean storageDetected = isBall(storageColor);

        boolean stagingDetected = staging.get() >= 0.2;

        // for making sure ball count is correct
        ballCount = (stagingDetected ? 1 : 0) + (storageDetected ? 1 : 0);

        if (driverOverride) return;

        // If there is a ball in the entrance, run the bottom motor.
        if (prevEntranceDetected && !storageDetected) {
            // Spin the bottom motor on a timer
            //entranceTimer.start();
            motorBottom.set(0.3);
            entToStorage = true;
        }

        // If 1.5 second has elapsed and ball has progressed past entrance, stop motor
        //(entranceTimer.get() >= 1.5) && 
        if (storageDetected && entToStorage) {
            //entranceTimer.stop();
            //entranceTimer.reset();
            motorBottom.set(0);
            entToStorage = false;
        }

        // If there is a ball between storage and staging and staging is empty, run the top and bottom motors
        if (storageDetected && !stagingDetected && !shotRequested) {
            // Spin the bottom and top motors on a timer
            storageTimer.start();
            motorTop.set(0.5);
            motorBottom.set(0.3);

            if (!rejectingChecked) {
                rejecting = storageColor != ALLIANCE_COLOR;
                rejectingChecked = true;
            }
        }

        // If 0.5 seconds have elapsed, stop the motors
        if (storageTimer.hasElapsed(0.5)) {
            storageTimer.stop();
            storageTimer.reset();
            motorTop.set(0);
            motorBottom.set(0);
        }

        // If there is a ball in staging, we don't want to push it into turret, especially if there is a shot going
        if (stagingDetected) motorTop.set(0);

        if (turretSubsystem != null) {
            // If a shot was requested and the turret is ready, load a ball into the turret.
            // TODO: why do we no longer check turret for alignment before shooting?
            turretSubsystem.setReject(rejecting);
            if (shotRequested) {
                // Spin the top motor on a timer
                exitTimer.start();
                motorTop.set(0.5);

                // If 1.5 seconds have elapsed, mark the shot as finished
                if (exitTimer.hasElapsed(1)) {
                    exitTimer.stop();
                    exitTimer.reset();
                    motorTop.set(0);

                    // Reset states
                    shotRequested = false;
                    rejectingChecked = false;

                    //TODO one problem here: if we request a shot without a ball being output
                    //our ball count will be off
                    ballCount--;
                    System.out.println("ball exited turret, was" + rejecting);
                }  
            }
        }
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
     */
    public void requestShot() {
        this.shotRequested = true;
    }

    /**
     * Gets how many total balls are inside internals.
     * @return The current ball count.
     */
    public int getBallCount() {
        return ballCount;
    }

    /**
     * Sets the ball count to the initial expected when autonomous starts (1).
     */
    public void setAutonInitialBallCount() {
        ballCount = 1;
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
     * Utility function for pretty-printing Color objects.
     * @param c The color to pretty-print.
     * @return The color represented as an RGB string.
     */
    private String colorToString(Color c) {
        return "(R: " + c.red + ", G: " + c.green + ", B: " + c.blue + ")";
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

    public void setDriverOverride(boolean override) {
        this.driverOverride = override;
    }
}
