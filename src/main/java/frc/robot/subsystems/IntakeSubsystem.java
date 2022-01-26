package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.commands.intake.IntakePosition;
import frc.robot.jetson.JetsonConnection;
import static frc.robot.Constants.IntakeConstants.*;

public class IntakeSubsystem extends SubsystemBase {
    private final JetsonConnection jetson;

    private final CANSparkMax intake;
    private final WPI_TalonSRX deploy;

    private IntakePosition currentPosition;

    public IntakeSubsystem(JetsonConnection jetson) {
        this.jetson = jetson;

        intake = new CANSparkMax(intakePort, MotorType.kBrushless);
        intake.restoreFactoryDefaults();

        deploy = new WPI_TalonSRX(deploymentPort);
        deploy.configFactoryDefault();
        deploy.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void periodic() {
        // If the jetson detects a ball and the intake is deployed, run the motors
        intake.set(jetson.ballDetected() && currentPosition == IntakePosition.DEPLOYED ? 0.5 : 0);
    }

    /**
     * Sets the position of the intake mechanism.
     * @param position The position to set the intake to.
     */
    public void setPosition(IntakePosition position) {
        currentPosition = position;
        deploy.set(ControlMode.Position, position.value);
    }
}
