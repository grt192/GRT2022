package frc.robot;

import java.util.HashSet;
import java.util.Hashtable;

import edu.wpi.first.wpilibj.PowerDistribution;

import frc.robot.subsystems.tank.TankSubsystem;

public class PowerController {

    private static PowerDistribution PDH = new PowerDistribution();

    // TODO: find current max value
    // TODO: Calculate total sustainable current
    private final double currentCurrentLimit = 350.0;
    private final double totalSustainableCurrent = 200.0;

    private GRTSubsystem[] subsystems;

    private final Hashtable<GRTSubsystem, Double> priorityList = new Hashtable<>();
    private final Hashtable<GRTSubsystem, Double> dynamicPriorityList = new Hashtable<>();

    public PowerController(GRTSubsystem... subsystems) {
        this.subsystems = subsystems;

        // Initialize priority lists with default and dynamic priorities
        for (GRTSubsystem subsystem : subsystems) {
            priorityList.put(subsystem, checkPriority(subsystem));
            dynamicPriorityList.put(subsystem, 5.0);
        }
    }

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

        // If current goes over sustainable current, scale subsystems down
        if (totalCurrent > totalSustainableCurrent) {
            scaleDown(totalCurrent, new HashSet<>());
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
     * Scales all subsystems by 80% to prevent brownout.
     * Call this as a last resort when the voltage drops to near brownout or if subsystem scaling fails to 
     * maintain the sustainable current.
     */
    private void setBrownoutScaling() {
        for (GRTSubsystem subsystem : subsystems) {
            subsystem.setCurrentLimit(subsystem.getCurrentLimit() * 0.8);
        }
    }

    /**
     * Recursively scales down the lowest priority subsystem until the total drawn current falls below the
     * sustainable threshold.
     * @param current The current total current.
     * @param checked A set of already scaled subsystems.
     */
    private void scaleDown(double current, HashSet<GRTSubsystem> checked) {
        GRTSubsystem lowest = null;

        // If we've already scaled many subsystems, just scale more dramatically for everything
        if (checked.size() > 3) {
            setBrownoutScaling();
            return;
        }

        for (GRTSubsystem subsystem : subsystems) {
            if (checked.contains(subsystem)) continue;

            if (lowest == null || higherPriority(lowest, subsystem)) {
                lowest = subsystem;
                continue;
            }
        }
        
        // Check the current drawn from the lowest priority subsystem and set the subsystem's current limit to either 
        // their minimum current requirement or their drawn current scaled to 80% of what it was, whichever is higher
        // (so we don't go below minimum required current)
        double currDrawn = lowest.getTotalCurrentDrawn();
        double newCurrDrawn = Math.max(lowest.getMinCurrent(), currDrawn * 0.8);
        lowest.setCurrentLimit(newCurrDrawn);

        // If the change in expected current will not be enough, scale again with the next lowest priority mech
        double newCurrent = current - (currDrawn - newCurrDrawn);
        if (newCurrent > totalSustainableCurrent) {
            checked.add(lowest);
            scaleDown(newCurrent, checked);
        }
    }

    /**
     * Recursively scales up previously scaled down subsystems to redistribute extra current.
     * @param extraCurrent The extra current to distribute.
     */
    public void scaleUp(double extraCurrent) {
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

        if (extraCurrent > 25) {
            scaleUp(extraCurrent);
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
        if (getDynamicPriority(inQuestion) == getDynamicPriority(baseline))
            return getBasePriority(inQuestion) > getBasePriority(baseline);
        // Otherwise, use dynamic priority
        return getDynamicPriority(inQuestion) > getDynamicPriority(baseline);
    }

    /**
     * Increase or decrease a subsystem's priority by 2.5.
     * @param subsystem The subsystem to increase or decrease the priority of.
     * @param increase Whether to increase or decrease the priority.
     */
    public void changePriority(GRTSubsystem subsystem, boolean increase) {
        double priority = dynamicPriorityList.get(subsystem);
        double newPriority = increase ? priority + 2.5 : priority - 2.5;

        if (Math.abs(10.0 - newPriority) > 0) {
            if (newPriority > 0) {
                newPriority = 0.0;
            } else {
                newPriority = 10.0;
            }
        }

        dynamicPriorityList.put(subsystem, newPriority);
    }

    private double getBasePriority(GRTSubsystem subsystem) {
        return priorityList.get(subsystem);
    }

    private double getDynamicPriority(GRTSubsystem subsystem) {
        return dynamicPriorityList.get(subsystem);
    }

  /**
   * Returns the priority assigned to a subsystem.
   * @param subsystem The subsystem to check.
   * @return An int rescribing the subsystem's priority.
   */
  private double checkPriority(GRTSubsystem subsystem) {
    if (subsystem instanceof TankSubsystem) return 1.0;
    /*
    if (subsystem instanceof IntakeSubsystem) return 2.0;
    if (subsystem instanceof ShooterSubsystem) return 3.0;
    if (subsystem instanceof ClimbSubsystem) return 4.0;
    */
    return 0.0;
  }
}
