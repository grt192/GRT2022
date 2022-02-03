package frc.robot.brownout;

import java.util.HashSet;

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
        double totalCurrentLimit = totalSustainableCurrent;
        double totalCurrentDrawn = PDH.getTotalCurrent();
        double idealRatio = totalCurrentLimit / totalCurrentDrawn;

        // Check each subsystem to see if they cannot be scaled (if the resultant limit would be below their minimum current).
        // If this is the case, limit the system by its minimum and adjust the ideal ratio accordingly.
        HashSet<GRTSubsystem> belowMin = new HashSet<>();
        for (GRTSubsystem subsystem : subsystems) {
            double drawn = subsystem.getTotalCurrentDrawn();
            double min = subsystem.getMinCurrent();

            if (drawn * idealRatio < min) {
                // Adjust the ratio to as if the below-minimum subsystem were excluded entirely from the calculation
                totalCurrentDrawn -= drawn;
                totalCurrentLimit -= min;
                idealRatio = totalCurrentLimit / totalCurrentDrawn;

                subsystem.setCurrentLimit(min);
                belowMin.add(subsystem);
            }
        }

        // For all other subsystems, limit each subsystem by scaling their current draw by the ideal ratio.
        // This acts to distribute unused limit to subsystems which are approaching their limits while maintaining that the
        // sum of all the current limits adds up to the total sustainable threshold.
        for (GRTSubsystem subsystem : subsystems) {
            if (belowMin.contains(subsystem)) continue;
            subsystem.setCurrentLimit(subsystem.getTotalCurrentDrawn() * idealRatio);
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
