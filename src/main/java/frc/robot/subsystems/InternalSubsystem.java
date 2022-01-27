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

    private final ColorSensorV3 sensorTop;
    private final ColorSensorV3 sensorBottom;

    private final ColorMatch colorMatcher;

    private boolean shotRequested;

    // will replace color values after testing
    private final Color RED = new Color(0.561, 0.232, 0.114);
    private final Color BLUE = new Color(0.143, 0.427, 0.429);

    private final int ballCount;

    public InternalSubsystem(TurretSubsystem turretSubsystem) {
        this.turretSubsystem = turretSubsystem;

        motorBottom = new WPI_TalonSRX(motorPortBottom);
        motorBottom.configFactoryDefault();
        motorBottom.setNeutralMode(NeutralMode.Brake);

        motorTop = new WPI_TalonSRX(motorPortTop);
        motorTop.configFactoryDefault();
        motorTop.setNeutralMode(NeutralMode.Brake);

        sensorTop = new ColorSensorV3(I2C.Port.kOnboard);
        sensorBottom = new ColorSensorV3(I2C.Port.kOnboard);

        colorMatcher = new ColorMatch();
        shotRequested = false;

        ballCount = 0;
    }

    @Override
    public void periodic() {
        controlFeed();

        if (shotRequested && turretSubsystem.flywheelReady() && turretSubsystem.turntableAligned()) { 
            // TODO launch ball into turret
        }
    }

    /**
     * Request that a ball be loaded and shot.
     * The ball will *actually* be shot when the turret is aimed and ready.
     */
    public void requestShot() {
        this.shotRequested = true;
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
      return (isRed(s) || isBlue(s));
    }

    public void updateBallCount(){
      if ()
    }

    public void controlFeed() {
        if (ballDetected(sensorTop)) {
            motorBottom.set(ControlMode.PercentOutput, 0);
        } else if (ballDetected(sensorBottom)) {
            motorBottom.set(ControlMode.PercentOutput, .5);
        }
    }
}
