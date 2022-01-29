package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

import static frc.robot.Constants.InternalConstants.*;

public class InternalSubsystem extends SubsystemBase {

    private final TurretSubsystem turretSubsystem;

    private final WPI_TalonSRX motorBottom;
    private final WPI_TalonSRX motorTop;

    // Sensors
    private final ColorSensorV3 entrance;
    private final ColorSensorV3 staging;
    private final ColorSensorV3 storage;
    private final ColorSensorV3 exit;

    private final ColorMatch colorMatcher;

    private boolean shotRequested = false;

    // Previous entrance / exit color states
    private Color prevEntranceColor;
    private Color prevExitColor;

    private int ballCount = 0;

    public InternalSubsystem(TurretSubsystem turretSubsystem) {
        this.turretSubsystem = turretSubsystem;

        // Initialize bottom motor
        motorBottom = new WPI_TalonSRX(motorPortBottom);
        motorBottom.configFactoryDefault();
        motorBottom.setNeutralMode(NeutralMode.Brake);

        // Initialize top motor
        motorTop = new WPI_TalonSRX(motorPortTop);
        motorTop.configFactoryDefault();
        motorTop.setNeutralMode(NeutralMode.Brake);

        // Initialize color sensors
        entrance = new ColorSensorV3(I2C.Port.kOnboard);
        staging = new ColorSensorV3(I2C.Port.kOnboard);
        storage = new ColorSensorV3(I2C.Port.kOnboard);
        exit = new ColorSensorV3(I2C.Port.kOnboard);

        colorMatcher = new ColorMatch();
        colorMatcher.addColorMatch(RED);
        colorMatcher.addColorMatch(BLUE);
        // TODO: add third color to color match to support wall color
    }

    @Override
    public void periodic() {
        updateBallCount();

        // If a ball has entered internals, start the bottom motor
        if (ballDetected(entrance)) motorBottom.set(0.5);

        // If there is a ball in storage, stop the bottom motor and start the top motor
        if (ballDetected(storage)) {
            motorBottom.set(0);
            motorTop.set(0.5);
        }

        // If there is a ball in staging, stop the top motor
        if (ballDetected(staging)) {
            // Reject the ball if it doesn't match alliance color
            turretSubsystem.setReject(getColor(staging) != allianceColor);
            motorTop.set(0);
        }

        // If a shot was requested and the turret is ready, load a ball into the turret
        if (shotRequested && turretSubsystem.flywheelReady() && turretSubsystem.turntableAligned()) { 
            // If the ball hasn't left the mechanism, spin the top motor
            if (!ballDetected(exit)) {
                motorTop.set(0.5);
            } else {
                shotRequested = false;
            }
        }
    }

    /**
     * Request that a ball be loaded and shot.
     * The ball will *actually* be shot when the turret is aimed and ready.
     */
    public void requestShot() {
        this.shotRequested = true;
    }

    /**
     * Checks sensors for incoming and outbound balls and update ball count accordingly.
     */
    public void updateBallCount() {
        Color entranceColor = getColor(entrance);
        if (!entranceColor.equals(prevEntranceColor) && (entranceColor.equals(RED) || entranceColor.equals(BLUE)))
            ballCount++;

        Color exitColor = getColor(exit);
        if (!exitColor.equals(prevExitColor) && (exitColor.equals(RED) || exitColor.equals(BLUE)))
            ballCount--;

        prevEntranceColor = entranceColor;
        prevExitColor = exitColor;
    }

    /**
     * Gets how many balls are inside internals.
     * @return The current ball count.
     */
    public int getBallCount() {
        return ballCount;
    }

    public boolean isRed(ColorSensorV3 s) {
        return getColor(s).equals(RED);
    }

    public boolean isBlue(ColorSensorV3 s) {
        return getColor(s).equals(BLUE);
    }

    public Color getColor(ColorSensorV3 s) {
        Color detectedColor = s.getColor();
        ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
        return match.color;
    }

    public boolean ballDetected(ColorSensorV3 s){
        return isRed(s) || isBlue(s);
    }
}
