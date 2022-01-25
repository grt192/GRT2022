//Skeleton Code
package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

public class InternalSubsystem extends SubsystemBase{

    private TalonSRX motor;

    private ColorSensorV3 sensorTop;
    private ColorSensorV3 sensorBottom;

    private I2C.Port i2cPortTop;
    private I2C.Port i2cPortBottom;

    private ColorMatch colorMatcher;
    private final Color BlueTarget;
    private final Color RedTarget;

    String color;

public InternalSubsystem(){
    this.motor = new WPI_TalonSRX(Config.getInt("motor_id"));

        /**
        * A Rev Color Sensor V3 object is constructed with an I2C port as a 
        * parameter. The device will be automatically initialized with default 
        * parameters.
        */
        this.sensorTop = new ColorSensorV3(i2cPortTop);
        this.sensorBottom = new ColorSensorV3(i2cPortBottom);

        //Change the I2C port below to match the connection of your color sensor
        this.i2cPortTop = new I2C.Port.kOnboard;
        this.i2cPortBottom = new I2C.Port.kOnboard;

        /**
        * A Rev Color Match object is used to register and detect known colors. This can 
        * be calibrated ahead of time or during operation.
        * 
        * This object uses a simple euclidian distance to estimate the closest match
        * with given confidence range.
        */
        this.colorMatcher = new ColorMatch();

        //will replace color values after testing
        BlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
        RedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114); 
}

@Override
  public void periodic() {
    // This method will be called once per scheduler run
    controlFeed();

   
  }

public boolean isRed(ColorSensorV3 s){
    Color detectedColor = s.getColor()
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
    return match.color == RedTarget
}

public boolean isBlue(ColorSensorV3 s){
    Color detectedColor = s.getColor();
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
    return match.color == BlueTarget;
}
/*
public void turnFeedOn(){
    while(isBlue(sensorBottom) && !isRed(sensorTop) && !isBlue(sensorTop)){
        motor.set(ControlMode.PercentOutput, .5);
    }
}

public void turnFeedOff(){
    if(isRed(sensorTop) || isBlue(sensorTop)){
        motor.set(ControlMode.PercentOutput, 0);
    }
}
*/
public void controlFeed(){
    if(isRed(sensorTop) || isBlue(sensorTop)){
        motor.set(ControlMode.PercentOutput, 0);
    }
    else if (isBlue(sensorBottom) || isRed(sensorBottom)){
        while (isBlue(sensorBottom) || isRed(sensorBottom)){
            motor.set(ControlMode.PercentOutput, .5);
        }
    }
}