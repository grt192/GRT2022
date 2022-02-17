package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A convenience class wrapping SubsystemBase for brownout and other shared subsystem logic.
 */
public abstract class GRTSubsystem extends SubsystemBase {
    protected final double minCurrent;

    /**
     * Creates a GRTSubsystem from the given minimum current.
     * @param minCurrent The minimum current.
     */
    public GRTSubsystem(double minCurrent) {
        this.minCurrent = minCurrent;
    }

    /**
     * Gets the subsystem's minimum current.
     * @return The minimum current.
     */
    public double getMinCurrent() {
        return minCurrent;
    }

    /**
     * Gets the total current being drawn by the subsystem. Implementers should use `PowerController.getCurrentDrawnFromPDH`
     * with all the subsystem's motor controller ports.
     * @return The total current being drawn by the subsystem.
     */
    abstract public double getTotalCurrentDrawn();

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
