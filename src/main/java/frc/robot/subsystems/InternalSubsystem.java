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

  // Change the I2C port below to match the connection of your color sensor
  private I2C.Port i2cPortTop = I2C.Port.kOnboard;
  private I2C.Port i2cPortBottom = I2C.Port.kOnboard;

  private final ColorMatch colorMatcher;

  // will replace color values after testing
  private final Color BlueTarget = new Color(0.143, 0.427, 0.429);
  private final Color RedTarget = new Color(0.561, 0.232, 0.114);

  private final String color;
  
  private final boolean shotRequested;

  public InternalSubsystem() {

    jetson = new JetsonConnection();

    TurretSubsystem turretSubsystem = new TurretSubsystem(jetson);

    motor = new WPI_TalonSRX(motorPort);

    /**
     * A Rev Color Sensor V3 object is constructed with an I2C port as a
     * parameter. The device will be automatically initialized with default
     * parameters.
     */
    sensorTop = new ColorSensorV3(i2cPortTop);
    sensorBottom = new ColorSensorV3(i2cPortBottom);

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

    if (turretSubsystem.flywheelReady() && shotRequested){ 
      // also add turntable aligned condition
      //TODO launch ball into turret
    }
  }

  public boolean isRed(ColorSensorV3 s) {
    Color detectedColor = s.getColor();
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
    return match.color == RedTarget;
  }

  public boolean isBlue(ColorSensorV3 s) {
    Color detectedColor = s.getColor();
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
    return match.color == BlueTarget;
  }

  public void controlFeed() {
    if (isRed(sensorTop) || isBlue(sensorTop)) {
      motor.set(ControlMode.PercentOutput, 0);
    } else if (isBlue(sensorBottom) || isRed(sensorBottom)) {
      while (isBlue(sensorBottom) || isRed(sensorBottom)) {
        motor.set(ControlMode.PercentOutput, .5);
      }
    }
  }

  public void requestShot() {
    this.shotRequested = true;
  }
}
