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
    private final ColorSensorV3 staging;
    private final ColorSensorV3 storage;
    // private final AnalogPotentiometer exit;

    private final Color RED = new Color(0.437255859375, 0.394775390625, 0.16845703125);
    private final Color BLUE = new Color(0.170654296875, 0.4189453125, 0.41064453125);
    private final Color EMPTY = new Color(0.272216796875, 0.48291015625, 0.2451171875);
    private final Color ALLIANCE_COLOR;

    private final ColorMatch colorMatcher;

    private boolean shotRequested = false;
    private int ballCount = 0;

    // Previous entrance / exit detection states
    private boolean prevEntranceDetected;

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
        staging = new ColorSensorV3(I2C.Port.kOnboard);
        storage = new ColorSensorV3(I2C.Port.kMXP);
        // exit = new AnalogPotentiometer(exitIRPort);

        colorSensorThread = new ColorSensorThread(storage, staging);

        colorMatcher = new ColorMatch();
        colorMatcher.addColorMatch(RED);
        colorMatcher.addColorMatch(BLUE);
        colorMatcher.addColorMatch(EMPTY);
        colorMatcher.setConfidenceThreshold(0.8);

        // Set alliance color from FMS
        switch (DriverStation.getAlliance()) {
            case Red: ALLIANCE_COLOR = RED; break;
            case Blue: ALLIANCE_COLOR = BLUE; break;
            default: ALLIANCE_COLOR = RED; break;
        }
    }

    @Override
    public void periodic() {
        // Check the entrance sensor for incoming balls and update the ball count accordingly
        boolean entranceDetected = ballDetected(entrance);
        if (!prevEntranceDetected && entranceDetected) ballCount++;
        prevEntranceDetected = entranceDetected;

        // Get last detected storage and staging colors from the sensor thread
        Color storageColor = matchColor(colorSensorThread.getLastStorage());
        Color stagingColor = matchColor(colorSensorThread.getLastStaging());

        boolean reject = false;

        // If a ball has entered internals, start the bottom motor
        if (entranceDetected) motorBottom.set(0.5);

        // If there is a ball in storage, stop the bottom motor and start the top motor
        if (isBall(storageColor)) {
            // Reject the ball if it doesn't match alliance color
            // Call this in storage *and* staging to give the turret more time to aim in a one-ball scenario;
            // a second ball in staging will override this call and not have any effect.
            reject = storageColor != ALLIANCE_COLOR;
            motorBottom.set(0);
            motorTop.set(0.5);
        }

        // If there is a ball in staging, stop the top motor
        if (isBall(stagingColor)) {
            // Reject the ball if it doesn't match alliance color
            reject = stagingColor != ALLIANCE_COLOR;
            motorTop.set(0);
        }

        if (turretSubsystem != null) {
            // If a shot was requested and the turret is ready, load a ball into the turret.
            // If rejecting, the turret can be in a semi-aligned state; otherwise, require it to be fully lined up.
            turretSubsystem.setReject(reject);
            if (shotRequested && turretSubsystem.getState() == TurretSubsystem.ModuleState.READY
                    || reject && turretSubsystem.getState() == TurretSubsystem.ModuleState.ALMOST) {
                // If the ball hasn't left staging, spin the top motor
                if (isBall(stagingColor)) {
                    motorTop.set(0.5);
                } else {
                    // Otherwise, the ball has left internals;
                    // Mark the shot as finished and decrement the ball count.
                    shotRequested = false;
                    ballCount--;
                }
            }
        }
    }

    public void setPower(double pow) {
        motorTop.set(pow);
        motorBottom.set(pow);
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
    public Color matchColor(Color color) {
        ColorMatchResult match = colorMatcher.matchColor(color);
        if (match == null) return null;
        return match.color;
    }

    /**
     * Checks whether a normalized color sensor color represents a ball being detected.
     * @param detected The color to check.
     * @return Whether a ball (of any color) has been detected.
     */
    public boolean isBall(Color detected) {
        return detected == RED || detected == BLUE;
    }

    /**
     * Checks whether a ball has been detected by an IR sensor.
     * @param s The IR sensor to check.
     * @return Whether a ball has been detected by the sensor.
     * TODO: test this and make sure it works
     */
    public boolean ballDetected(AnalogPotentiometer s) {
        // TODO: measure the resting value of the IR sensor on the wall
        return s.get() > 0.40;
    }

    /**
     * Utility function for pretty-printing Color objects.
     * @param c The color to pretty-print.
     * @return The color represented as an RGB string.
     */
    public String colorToString(Color c) {
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
