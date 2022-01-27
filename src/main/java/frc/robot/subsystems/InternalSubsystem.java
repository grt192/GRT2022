package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

import static frc.robot.Constants.InternalConstants.*;

// think about being able to indicate alliance color on driver station at least
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

    // will replace color values after testing
    private final Color RED = new Color(0.561, 0.232, 0.114);
    private final Color BLUE = new Color(0.143, 0.427, 0.429);

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
    }

    @Override
    public void periodic() {
        updateBallCount();

        // TODO: this motor stopping needs to work together with shotRequested and intake
        // If there is a ball in storage, stop the bottom motor
        if (ballDetected(storage))
            motorBottom.set(ControlMode.PercentOutput, 0);

        // If there is a ball in staging, stop the top motor
        if (ballDetected(staging))
            motorTop.set(ControlMode.PercentOutput, 0);

        // If a shot was requested and the turret is ready, load a ball into the turret
        if (shotRequested && turretSubsystem.flywheelReady() && turretSubsystem.turntableAligned()) { 
            //launch ball into turret
            if (!ballDetected(top)){
                motorTop.set(ControlMode.PercentOutput, .5);
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
        if (!exitColor.equals(prevEntranceColor) && (exitColor.equals(RED) || exitColor.equals(BLUE)))
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

    public void topBall(){
        if (getColor(staging) != allianceColor){
            turretSubsystem.setHoodAngle();
        }
    }
}
