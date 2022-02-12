package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

import frc.robot.GRTSubsystem;
import frc.robot.brownout.PowerController;

import static frc.robot.Constants.InternalConstants.*;

public class InternalSubsystem extends GRTSubsystem {

    private final TurretSubsystem turretSubsystem;

    private final WPI_TalonSRX motorBottom;
    private final WPI_TalonSRX motorTop;

    private final AnalogPotentiometer entrance;
    private final ColorSensorV3 staging;
    private final ColorSensorV3 storage;
    //private final AnalogPotentiometer exit;

    private final ColorMatch colorMatcher;

    private boolean shotRequested = false;

    // Previous entrance / exit detection states
    private boolean prevEntranceDetected;

    private int ballCount = 0;

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
        //exit = new AnalogPotentiometer(exitIRPort);

        colorMatcher = new ColorMatch();
        colorMatcher.addColorMatch(RED);
        colorMatcher.addColorMatch(BLUE);
        // TODO: add third color to color match to support wall color
    }

    @Override
    public void periodic() {
        updateBallCount();
        boolean reject = false;

        // If a ball has entered internals, start the bottom motor
        if (ballDetected(entrance)) motorBottom.set(0.5);

        // If there is a ball in storage, stop the bottom motor and start the top motor
        if (ballDetected(storage)) {
            // Reject the ball if it doesn't match alliance color
            // Call this in storage *and* staging to give the turret more time to aim in a one-ball scenario; 
            // a second ball in staging will override this call and not have any effect.
            reject = getColor(storage) != allianceColor;
            motorBottom.set(0);
            motorTop.set(0.5);
        }

        // If there is a ball in staging, stop the top motor
        if (ballDetected(staging)) {
            // Reject the ball if it doesn't match alliance color
            reject = getColor(staging) != allianceColor;
            motorTop.set(0);
        }

        if (turretSubsystem != null) {
            // If a shot was requested and the turret is ready, load a ball into the turret.
            // If rejecting, the turret can be in an orange state; otherwise, require it to be green (fully lined up).
            turretSubsystem.setReject(reject);
            if (shotRequested && turretSubsystem.getState() == TurretSubsystem.ModuleState.GREEN
                || reject && turretSubsystem.getState() == TurretSubsystem.ModuleState.ORANGE) { 
                // If the ball hasn't left staging, spin the top motor
                if (ballDetected(staging)) {
                    motorTop.set(0.5);
                } else {
                    shotRequested = false;
                    ballCount--;
                }
            }
        }

        motorBottom.set(0.4);
        motorTop.set(0.4);
    }

    /**
     * Request that a ball be loaded and shot.
     * The ball will *actually* be shot when the turret is aimed and ready.
     */
    public void requestShot() {
        this.shotRequested = true;
    }

    /**
     * Checks sensors for incoming balls and updates ball count accordingly.
     */
    public void updateBallCount() {
        boolean entranceDetected = ballDetected(entrance);
        if (!prevEntranceDetected && entranceDetected)
            ballCount++;

        prevEntranceDetected = entranceDetected;
    }

    /**
     * Gets how many balls are inside internals.
     * @return The current ball count.
     */
    public int getBallCount() {
        return ballCount;
    }

    /**
     * Gets the detected color from a color sensor, normalizing it to the closest stored color in colorMatcher.
     * @param s The color sensor to get.
     * @return The normalized detected color.
     */
    public Color getColor(ColorSensorV3 s) {
        Color detectedColor = s.getColor();
        ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
        return match.color;
    }

    /**
     * Checks whether a ball has been detected by a color sensor.
     * @param s The color sensor to check.
     * @return Whether a ball has been detected by the sensor.
     */
    public boolean ballDetected(ColorSensorV3 s){
        Color detected = getColor(s);
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

    @Override
    public double getTotalCurrentDrawn() {
        return PowerController.getCurrentDrawnFromPDH();
    }

    @Override
    public void setCurrentLimit(double limit) {
        int motorLimit = (int) Math.floor(limit / 2);

        motorBottom.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, motorLimit, 0, 0));
        motorTop.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, motorLimit, 0, 0));
    }
}
