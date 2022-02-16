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
    // private final AnalogPotentiometer exit;

    private final Color RED = new Color(0.437255859375, 0.394775390625, 0.16845703125);
    private final Color BLUE = new Color(0.170654296875, 0.4189453125, 0.41064453125);
    private final Color EMPTY = new Color(0.272216796875, 0.48291015625, 0.2451171875);
    private final Color ALLIANCE_COLOR;

    private final ColorMatch colorMatcher;
    private final Timer exitTimer;

    private boolean shotRequested = false;
    private boolean rejecting = false;
    private boolean rejectingChecked = false;
    private int ballCount = 0;

    private boolean prevEntranceDetected = false;

    public InternalSubsystem(TurretSubsystem turretSubsystem) {
        // TODO: measure this
        super(50);

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
        // exit = new AnalogPotentiometer(exitIRPort);

        colorSensorThread = new ColorSensorThread(storage);

        colorMatcher = new ColorMatch();
        colorMatcher.addColorMatch(RED);
        colorMatcher.addColorMatch(BLUE);
        colorMatcher.addColorMatch(EMPTY);
        colorMatcher.setConfidenceThreshold(0.8);

        exitTimer = new Timer();

        // Set alliance color from FMS
        switch (DriverStation.getAlliance()) {
            case Red: ALLIANCE_COLOR = RED; break;
            case Blue: ALLIANCE_COLOR = BLUE; break;
            default: ALLIANCE_COLOR = RED; break;
        }
    }

    @Override
    public void periodic() {
        // Get last detected storage color from the sensor thread and check IR sensors for balls.
        Color storageColor = matchColor(colorSensorThread.getLastStorage());
        boolean entranceDetected = entrance.get() > 0.4;
        boolean stagingDetected = entrance.get() > 0.3;

        System.out.println("STORAGE matched " + (
            storageColor == RED ? "RED" :
            storageColor == BLUE ? "BLUE" :
            storageColor == EMPTY ? "EMPTY" :
            "null"));

        // Check the entrance sensor for incoming balls and update the ball count accordingly
        if (!prevEntranceDetected && entranceDetected) ballCount++;
        prevEntranceDetected = entranceDetected;

        // If a ball has entered internals, start the bottom motor
        if (entranceDetected) motorBottom.set(0.3);

        // If there is a ball in storage, start the top motor
        if (isBall(storageColor)) {
            // If we haven't already checked rejection logic, reject the ball if it doesn't match alliance color
            if (!rejectingChecked) {
                rejecting = storageColor != ALLIANCE_COLOR;
                rejectingChecked = true;
            }
            motorTop.set(0.5);
            System.out.println("STORAGE detected, running top motor");
        }

        // If there is a ball in staging, stop the bottom and top motors
        if (stagingDetected) {
            motorBottom.set(0);
            motorTop.set(0);
            System.out.println("STAGING detected, stopping both motors");
        }

        if (turretSubsystem != null) {
            // If a shot was requested and the turret is ready, load a ball into the turret.
            // If rejecting, the turret can be in a semi-aligned state; otherwise, require it to be fully lined up.
            turretSubsystem.setReject(rejecting);
            if (shotRequested /* && turretSubsystem.getState() == TurretSubsystem.ModuleState.READY
                 || rejecting && turretSubsystem.getState() == TurretSubsystem.ModuleState.ALMOST */) {
                // Spin the top motor on a timer
                exitTimer.start();
                motorTop.set(0.5);
                turretSubsystem.setFlywheelPower(0.2);
                System.out.println("Shot requested, processing");

                // If 1.5 seconds have elapsed, mark the shot as finished and decrement the ball count.
                if (exitTimer.hasElapsed(1.5)) {
                    exitTimer.stop();
                    exitTimer.reset();

                    motorTop.set(0);
                    turretSubsystem.setFlywheelPower(0);

                    shotRequested = false;
                    rejectingChecked = false;
                    ballCount--;

                    System.out.println("Shot finished, terminating loop");
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
     * Gets how many balls are inside internals.
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
}
