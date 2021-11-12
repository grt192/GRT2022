package frc.robot.subsystems.sensors;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.cameraserver.*;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.I2C;

//import com.revrobotics.ColorSensorV3;

public class SensorTesting extends SubsystemBase {

    private AnalogGyro analogGyro1 = new AnalogGyro(0);
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    //private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
    private DigitalInput limitSwitch = new DigitalInput(1);
    //private Encoder encoder = new Encoder(port1, port2, reverseEncoding, EncodingType.k4X);
    private Encoder encoder = new Encoder(1, 2, true, EncodingType.k4X);



      @Override
      public void periodic() {
        // This method will be called once per scheduler run

        //Gyro sensor test
        System.out.println("Gyro output: " + Math.round(analogGyro1.getAngle()));

        //2-in-1 color+distance sensor test
        //System.out.println("Color detetcted: " + m_colorSensor.getColor() + ", Proximity: " + m_sensor.getProximity());

        //limit switch test
        System.out.println("Limit Switch is pressed: " + limitSwitch.get());

        //encoder test
        System.out.println("Encoder distance: " + encoder.getDistance() + ", Encoder direction: " + encoder.getDirection()+ ", Encoder distance per pulse: " + encoder.getDistancePerPulse()); 
      }
}
