package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ClimbConstants.*;

public class ClimbSubsystem extends SubsystemBase {
    private final CANSparkMax six;
    private final CANSparkMax ten;

    private final WPI_TalonSRX fifteenMain;
    private final WPI_TalonSRX fifteenFollow;

    public ClimbSubsystem() {
        // Initialize six point arm NEO
        six = new CANSparkMax(sixMotorPort, MotorType.kBrushless);
        six.restoreFactoryDefaults();

        // Initialize ten point arm NEO
        ten = new CANSparkMax(tenMotorPort, MotorType.kBrushless);
        ten.restoreFactoryDefaults();

        // Initialize fifteen point arm 775 and follower
        fifteenMain = new WPI_TalonSRX(fifteenLeftPort);
        fifteenMain.configFactoryDefault();

        fifteenFollow = new WPI_TalonSRX(fifteenRightPort);
        fifteenFollow.configFactoryDefault();
        fifteenFollow.follow(fifteenMain);
    }

    /**
     * Start the climb sequence!
     */
    public void climb() {
        // TODO
    }
}
