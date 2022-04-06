package frc.robot.brownout;

import java.util.HashSet;
import java.util.Hashtable;
import java.util.Set;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import frc.robot.GRTSubsystem;
import frc.robot.shuffleboard.GRTNetworkTableEntry;
import frc.robot.shuffleboard.GRTShuffleboardTab;

/**
 * A power controller which dynamically sets current limits for subsystems to avoid brownout situations.
 */
public class PowerController {
    private final PowerDistribution PDH;

    private final GRTSubsystem[] subsystems;

    private final GRTShuffleboardTab shuffleboardTab;
    private final Hashtable<String, GRTNetworkTableEntry> limitEntries;
    private final Hashtable<String, GRTNetworkTableEntry> drawEntries;

    // TODO: Calculate total sustainable current
    private static final double totalSustainableCurrent = 200.0;

    public PowerController(PowerDistribution PDH, GRTSubsystem... subsystems) {
        this.PDH = PDH;
        this.subsystems = subsystems;

        /**
         * Drive right trigger -> freeze turret
         */

        // Dynamically initialize shuffleboard limit entries
        shuffleboardTab = new GRTShuffleboardTab("Brownout");
        this.limitEntries = new Hashtable<>();
        this.drawEntries = new Hashtable<>();

        for (int i = 0; i < subsystems.length; i++) {
            GRTSubsystem subsystem = subsystems[i];
            limitEntries.put(
                subsystem.getName(), 
                shuffleboardTab.addEntry(subsystem.getName() + " limit", subsystem.getMinCurrent())
                    .at(i, 0)
                    .widget(BuiltInWidgets.kNumberBar)
            );
            drawEntries.put(
                subsystem.getName(), 
                shuffleboardTab.addEntry(subsystem.getName() + " draw", subsystem.getTotalCurrentDrawn(this))
                    .at(i, 1)
                    .widget(BuiltInWidgets.kVoltageView)
            );
        }
    }

    /**
     * Calculates the current limits for each subsystem based on their usage.
     * This method ensures that all subsystem limits sum to `totalSustainableCurrent`, with dynamic limit distribution
     * based on usage ratios and minimum current values.
     */
    public void calculateLimits() {
        double totalDraw = PDH.getTotalCurrent();
        calculateLimits(totalSustainableCurrent, totalDraw, Set.of(subsystems));
    }

    /**
     * Calculates the current limits for each subsystem based on their usage.
     * This method ensures that all subsystem limits sum to `totalSustainableCurrent`, with dynamic limit distribution
     * based on usage ratios and minimum current values.
     * 
     * @param totalCurrent The total current limit.
     * @param totalDraw The total current draw.
     * @param remaining The remaining (non-scaled) subsystems.
     */
    private void calculateLimits(double totalCurrent, double totalDraw, Set<GRTSubsystem> remaining) {
        // If the total draw is 0, avoid `NaN` by setting all remaining subsystems to their minimums
        if (totalDraw == 0) {
            remaining.forEach(subsystem -> subsystem.setCurrentLimit(subsystem.getMinCurrent()));
            return;
        }

        // Calculate the "ideal ratio" from the total drawn current
        double idealRatio = totalCurrent / totalDraw;

        // Check each subsystem to see if they cannot be scaled (if the resultant limit would be below their minimum current).
        // If this is the case, limit the system by its minimum and adjust the ideal ratio accordingly.
        for (GRTSubsystem subsystem : remaining) {
            double drawn = subsystem.getTotalCurrentDrawn(this);
            double min = subsystem.getMinCurrent();

            drawEntries.get(subsystem.getName()).setValue(drawn);

            if (drawn * idealRatio < min) {
                subsystem.setCurrentLimit(min);
                limitEntries.get(subsystem.getName()).setValue(min);

                // Adjust the ratio to as if the below-minimum subsystem were excluded entirely from the calculation
                HashSet<GRTSubsystem> cloned = new HashSet<GRTSubsystem>(remaining);
                cloned.remove(subsystem);
                calculateLimits(totalCurrent - min, totalDraw - drawn, cloned);
                return;
            }
        }

        // For all other subsystems, limit each subsystem by scaling their current draw by the ideal ratio.
        // This acts to distribute unused limit to subsystems which are approaching their limits while maintaining that the
        // sum of all the current limits adds up to the total sustainable threshold.
        for (GRTSubsystem subsystem : remaining) {
            double limit = subsystem.getTotalCurrentDrawn(this) * idealRatio;
            subsystem.setCurrentLimit(limit);
            limitEntries.get(subsystem.getName()).setValue(limit);
        }
    }

    /**
     * Gets the total current drawn from the PDH by the specified channels.
     * @param channels The channels to sum.
     * @return The total current drawn by the provided channels.
     */
    public double getCurrentDrawnFromPDH(int... channels) {
        double sum = 0;
        for (int channel : channels) {
            sum += PDH.getCurrent(channel);
        }
        return sum;
    }
}
