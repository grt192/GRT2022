package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.PWMSparkMax;

// note: if you don't like this class that's okay, use CTR's "follower" features
// https://docs.ctre-phoenix.com/en/latest/ch13_MC.html#follower

/**
 * A collection of TalonSRX motors that must turn in sync.
 */
public class TalonGroup {
    private TalonSRX[] motors;
    private boolean inverted; // invert motor direction?

    public TalonGroup(TalonSRX[] motors) {
        this(motors, false);
    }

    public TalonGroup(TalonSRX[] motors, boolean inverted) {
        this.motors = motors;
        this.inverted = inverted;
    }

    public void set(ControlMode mode, double demand) {
        for (TalonSRX motor : motors) {
            if (inverted) {
                motor.set(mode, -demand);
            } else {
                motor.set(mode, demand);
            }
        }
    }

    public void setNeutralMode(NeutralMode mode) {
        for (TalonSRX motor : motors) {
            motor.setNeutralMode(mode);
        }
    }

    public void setNeutralDeadband(double percentDeadband) {
        for (TalonSRX motor : motors) {
            motor.configNeutralDeadband(percentDeadband);
        }
    }
}
