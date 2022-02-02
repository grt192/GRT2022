package frc.robot;

import java.util.HashSet;
import java.util.Hashtable;

import edu.wpi.first.wpilibj.PowerDistribution;

import frc.robot.subsystems.tank.TankSubsystem;

public class PowerController {

    private static PowerDistribution PDH = new PowerDistribution();

    private GRTSubsystem[] subsystems;
    private final Hashtable<GRTSubsystem, Double> dynamicPriorities = new Hashtable<>();

    // TODO: find current max value
    // TODO: Calculate total sustainable current
    private static final double totalSustainableCurrent = 200.0;

    public PowerController(GRTSubsystem... subsystems) {
        this.subsystems = subsystems;

        // Initialize dynamic priority list with default values
        for (GRTSubsystem subsystem : subsystems) {
            dynamicPriorities.put(subsystem, 5.0);
        }
    }

    /**
     * Checks the drawn current for a potential brownout.
     * If the total drawn current is above the sustainable amount, scales down subsystems by priority.
     * If the voltage is near brownout, triggers the harsher brownout scaling.
     */
    public void check() {
        System.out.println("Checking for a brownout...");

        // If the PDH is close to a brownout, trigger brownout scaling
        if (PDH.getVoltage() < 7.0) {
            System.out.println("close to brownout!");
            setBrownoutScaling();
        }

        // Sum up total current drawn from all subsystems
        double totalCurrent = PDH.getTotalCurrent();
        System.out.println("total current drawn: " + totalCurrent);

        // If current goes over sustainable current, scale subsystems down by the difference
        if (totalCurrent > totalSustainableCurrent) {
            scaleSubsystemsDownByCurrent(totalCurrent - totalSustainableCurrent);
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

    /**
     * Scales down subsystems by a given current based on weighted priority averages.
     * @param current The current to decrease usage by.
     */
    private void scaleSubsystemsDownByCurrent(double current) {
        scaleSubsystemsDownByCurrent(current, new HashSet<>());
    }

    private void scaleSubsystemsDownByCurrent(double current, HashSet<GRTSubsystem> excluded) {
        Hashtable<GRTSubsystem, Double> weightedPriorities = new Hashtable<>();
        double totalPriority = 0;

        // Populate weighted table and total priority from non-excluded subsystems
        for (GRTSubsystem subsystem : subsystems) {
            if (excluded.contains(subsystem)) continue;

            // Subsystem weight: base * dynamic
            double weight = basePriority(subsystem) * dynamicPriorities.get(subsystem);
            weightedPriorities.put(subsystem, weight);
            totalPriority += weight;
        }

        // Distribute current with scaled weights
        double deficit = 0;
        for (var entry : weightedPriorities.entrySet()) {
            GRTSubsystem subsystem = entry.getKey();

            double desired = current * entry.getValue() / totalPriority;
            double scaled = Math.max(desired, subsystem.getMinCurrent());

            // If we want to scale by more than the minimum, set it to the minimum instead and add the difference
            // to the deficit
            if (desired > scaled) {
                deficit += desired - scaled;
                excluded.add(subsystem);
            }
            subsystem.setCurrentLimit(scaled);
        }

        // If there's current deficit, scale down again
        if (deficit > 0) scaleSubsystemsDownByCurrent(deficit, excluded);
    }

    /**
     * Recursively scales up previously scaled down subsystems to redistribute extra current.
     * @param extraCurrent The extra current to distribute.
     */
    public void distributeExtraCurrent(double extraCurrent) {
        for (GRTSubsystem subsystem : subsystems) {
            // We can do this smartly later on
            if ((subsystem.getCurrentLimit() - subsystem.getMinCurrent()) >= 25) {

                if (extraCurrent >= 0) {
                    double currentChange = extraCurrent >= 100
                        ? extraCurrent / 2
                        : extraCurrent;

                    subsystem.setCurrentLimit(subsystem.getCurrentLimit() + currentChange);
                    extraCurrent -= currentChange;
                }
            }
        }

        if (extraCurrent > 25) distributeExtraCurrent(extraCurrent);
    }

    /**
     * Scales all subsystems by 80% to prevent brownout.
     * Call this as a last resort when the voltage drops to near brownout or if subsystem scaling fails to 
     * maintain the sustainable current.
     */
    private void setBrownoutScaling() {
        for (GRTSubsystem subsystem : subsystems) {
            subsystem.setCurrentLimit(Math.max(subsystem.getCurrentLimit() * 0.8, subsystem.getMinCurrent()));
        }
    }

    /**
     * Returns whether a given subsystem is higher in priority than another.
     * @param inQuestion The subsystem to check.
     * @param baseline The subsystem to compare to.
     * @return Whether `inQuestion` is higher priority than `baseline`.
     */
    public boolean higherPriority(GRTSubsystem inQuestion, GRTSubsystem baseline) {
        // If the dynamic priorities are the same, break ties with baseline priority
        if (dynamicPriorities.get(inQuestion) == dynamicPriorities.get(baseline))
            return basePriority(inQuestion) > basePriority(baseline);
        // Otherwise, use dynamic priority
        return dynamicPriorities.get(inQuestion) > dynamicPriorities.get(baseline);
    }

    /**
     * Increase or decrease a subsystem's priority by an amount.
     * @param subsystem The subsystem to increase or decrease the priority of.
     * @param change The amount to change the priority by.
     */
    public void setPriority(GRTSubsystem subsystem, double change) {
        double priority = dynamicPriorities.get(subsystem);

        // Constrain the new priority within [0, 10]
        double newPriority = Math.max(Math.min(priority + change, 10), 0);
        dynamicPriorities.put(subsystem, newPriority);
    }

    /**
     * Returns the base priority assigned to a subsystem.
     * @param subsystem The subsystem to check.
     * @return An int describing the subsystem's base priority (higher = better).
     */
    private double basePriority(GRTSubsystem subsystem) {
        if (subsystem instanceof TankSubsystem) return 2.4;
        /*
        if (subsystem instanceof IntakeSubsystem) return 2.2;
        if (susbsystem instanceof InternalSubsystem) return 2.0;
        if (subsystem instanceof TurretSubsystem) return 1.9;
        if (subsystem instanceof ClimbSubsystem) return 1.7;
        */
        return 0.0;
    }
}
