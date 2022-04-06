package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.brownout.PowerController;

/**
 * A convenience class wrapping SubsystemBase for brownout and other shared subsystem logic.
 */
public abstract class GRTSubsystem extends SubsystemBase {
    protected final double minCurrent;
    private final int[] motorPorts;

    /**
     * Creates a GRTSubsystem from the given minimum current and motor ports to be checked for subsystem current draw.
     * @param minCurrent The minimum current.
     */
    public GRTSubsystem(double minCurrent, int... motorPorts) {
        this.minCurrent = minCurrent;
        this.motorPorts = motorPorts;
    }

    /**
     * Gets the subsystem's minimum current.
     * @return The minimum current.
     */
    public final double getMinCurrent() {
        return minCurrent;
    }

    /**
     * Gets the total current being drawn by the subsystem by summing the current drawn by the supplied motor controller
     * ports on the PDH.
     * @param powerController The PowerController instance, for calling `getCurrentDrawnFromPDH`.
     * @return The total current being drawn by the subsystem.
     */
    public final double getTotalCurrentDrawn(PowerController powerController) {
        return powerController.getCurrentDrawnFromPDH(motorPorts);
    }

    /**
     * Sets the subsystem's current limit. Implementers should split the subsystem's limit between their motor controllers and 
     * call `setSmartCurrentLimit()` or the Talon equivalent to enforce the limit.
     * @param limit The current limit (in amps) for the subsystem.
     */
    abstract public void setCurrentLimit(double limit);

    /**
     * Contains logic for cleaning up the subsystem for climb.
     * TODO: this might not be strictly necessary; perhaps it would be useful to call `climbInit()` on all subsystems before climb
     * and leave the cleanup logic to be implemented (or left blank) in each subsystem, but it's also probably fine to call
     * `climbInit()` on only the subsystems that need to clean up (and not include it in this class). I guess it depends on how
     * many subsystems need to implement climb cleanup logic.
     */
    public void climbInit() { }
}
