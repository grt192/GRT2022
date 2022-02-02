package frc.robot.brownout;

import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.GRTSubsystem;

/**
 * A power controller which dynamically sets current limits for subsystems to avoid brownout situations.
 */
public class PowerController {
    private static PowerDistribution PDH = new PowerDistribution();

    private GRTSubsystem[] subsystems;

    // TODO: Calculate total sustainable current
    private static final double totalSustainableCurrent = 200.0;

    public PowerController(GRTSubsystem... subsystems) {
        this.subsystems = subsystems;

        // Start the power controller thread
        Thread powerControllerThread = new Thread(new PowerControllerThread(this));
        powerControllerThread.start();
    }

    /**
     * Calculates the current limits for each subsystem based on their usage.
     * This method ensures that all subsystem limits sum to `totalSustainableCurrent`, with dynamic limit distribution
     * based on usage ratios.
     */
    public void calculateLimits() {
        // Calculate the "ideal ratio" from the total drawn current
        double idealRatio = totalSustainableCurrent / PDH.getTotalCurrent();

        // Limit each subsystem by scaling their current draw by the ideal ratio.
        // This acts to distribute unused limit to subsystems which are approaching their limits while maintaining that the
        // sum of all the current limits adds up to the total sustainable threshold.
        // As the ideal ratio approaches 1, no distribution can occur until a subsystem drops its current usage.
        for (GRTSubsystem subsystem : subsystems) {
            double limit = subsystem.getTotalCurrentDrawn() * idealRatio;
            // TODO: minCurrent enforcement? This would only be applicable in the case where idealRatio = 1 and a subsystem
            // which was using less power than its minimum gets "locked" at that low usage until other subsystems stop using current
            subsystem.setCurrentLimit(limit);
        }
    }

    /**
     * Gets the total current drawn from the PDH by the specified channels.
     * @param channels The channels to sum.
     * @return The total current drawn by the provided channels.
     */
    public static double getCurrentDrawnFromPDH(int... channels) {
        double sum = 0;

        for (int channel : channels) {
            sum += PDH.getCurrent(channel);
        }

        // Is this necessary?
        if (channels.length == 16 && (sum != PDH.getTotalCurrent())) {
            System.out.println("something is wrong, total current does not match");
        }

        return sum;
    }
}
