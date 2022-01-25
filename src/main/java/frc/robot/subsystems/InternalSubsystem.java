package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

import static frc.robot.Constants.InternalConstants.*;
import frc.robot.jetson.JetsonConnection;

import frc.robot.subsystems.TurretSubsystem;

public class InternalSubsystem extends SubsystemBase {

  private final TurretSubsystem turretSubsystem;

  private final WPI_TalonSRX motor;

  private final JetsonConnection jetson;

  private final ColorSensorV3 sensorTop;
  private final ColorSensorV3 sensorBottom;

  private final ColorMatch colorMatcher;
  
  private boolean shotRequested;

  // will replace color values after testing
  private final Color RED = new Color(0.561, 0.232, 0.114);
  private final Color BLUE = new Color(0.143, 0.427, 0.429);

  public InternalSubsystem() {

    jetson = new JetsonConnection();

    turretSubsystem = new TurretSubsystem(jetson);

    motor = new WPI_TalonSRX(motorPort);

    /**
     * A Rev Color Sensor V3 object is constructed with an I2C port as a
     * parameter. The device will be automatically initialized with default
     * parameters.
     */
    sensorTop = new ColorSensorV3(I2C.Port.kOnboard);
    sensorBottom = new ColorSensorV3(I2C.Port.kOnboard);

    /**
     * A Rev Color Match object is used to register and detect known colors. This
     * can
     * be calibrated ahead of time or during operation.
     * 
     * This object uses a simple euclidian distance to estimate the closest match
     * with given confidence range.
     */
    colorMatcher = new ColorMatch();
    shotRequested = false;
  }

  @Override
  public void periodic() {
    controlFeed();

    if (turretSubsystem.flywheelReady() && shotRequested && turretSubsystem.turntableAligned()){ 
      //TODO launch ball into turret
    }
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

  public void controlFeed() {
    if (isRed(sensorTop) || isBlue(sensorTop)) {
      motor.set(ControlMode.PercentOutput, 0);
    } else if (isBlue(sensorBottom) || isRed(sensorBottom)) {
      motor.set(ControlMode.PercentOutput, .5);
    }
  }

  public void requestShot() {
    this.shotRequested = true;
  }
}
