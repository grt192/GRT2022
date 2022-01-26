package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ClimbConstants.*;

public class ClimbSubsystem extends SubsystemBase {
    private final CANSparkMax six;
    private final CANSparkMax ten;

    public ClimbSubsystem() {
        // Initialize six point arm motor
        six = new CANSparkMax(sixMotorPort, MotorType.kBrushless);
        six.restoreFactoryDefaults();

        // Initialize ten point arm motor
        ten = new CANSparkMax(tenMotorPort, MotorType.kBrushless);
        ten.restoreFactoryDefaults();
    }

    /**
     * Start the climb sequence!
     */
    public void climb() {
        // TODO
    }
}
