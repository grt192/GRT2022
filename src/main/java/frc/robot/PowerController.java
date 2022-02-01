package frc.robot;

import java.util.HashSet;

import edu.wpi.first.wpilibj.PowerDistribution;

import frc.robot.subsystems.tank.TankSubsystem;

public class PowerController {

    private static PowerDistribution PDH = new PowerDistribution();

    // TODO: find current max value
    // TODO: Calculate total sustainable current
    private final double currentCurrentLimit = 350.0;
    private final double totalSustainableCurrent = 200.0;

    private ControllableSubsystem[] subsystems;

    public PowerController(ControllableSubsystem... subsystems) {
        this.subsystems = subsystems;

        //TODO Calculate total sustainable current
        totalSustainableCurrent = 200;

        //initialize our priority list with default pre-game priorities
        priorityList.put(tankSubsystem, 1.0);
        /*priorityList.put(intakeSubsystem, 2);
        priorityList.put(shooterSubsystem, 3);
        priorityList.put(climbSubsystem, 4);*/

        //initialize the dynamic priority list with all subsystems starting the same
        dynamicPriorityList.put(tankSubsystem, 5.0);
        /*dynamicpriorityList.put(intakeSubsystem, 5.0);
        dynamicpriorityList.put(shooterSubsystem, 5.0);
        dynamicpriorityList.put(climbSubsystem, 5.0);*/
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
     * Scales all subsystems by a set sensible amount to prevent brownout.
     * Call this as a last resort when the voltage drops to near brownout or if subsystem scaling fails to 
     * maintain the sustainable current.
     */
    private void setBrownoutScaling() {
        for (ControllableSubsystem subsystem : subsystems) {
            double currDrawn = subsystem.getTotalCurrentDrawn();
            int tempscale = (int) (currDrawn * 0.8);
            subsystem.setCurrentLimit(tempscale);
        }
    }

    /**
     * Recursively scales down the lowest priority subsystem until the total drawn current falls below the
     * sustainable threshold.
     * @param current The current total current.
     * @param checked A set of already scaled subsystems.
     */
    private void scaleDown(double current, HashSet<ControllableSubsystem> checked) {
        ControllableSubsystem lowestPriority = null;
        int lowPriority = 10; // This int should be higher than the number of subsystems

        // If we've already scaled many subsystems, just scale more dramatically for everything
        if (checked.size() > 3) {
            setBrownoutScaling();
            return;
        }

        for (ControllableSubsystem subsystem : subsystems) {
            int priority = checkPriority(subsystem);
            if (priority < lowPriority && !(checked.contains(subsystem))) {
                lowestPriority = subsystem;
                lowPriority = priority;
            }
        }
    }

    /**
     * Recursively scales up previously scaled down subsystems to redistribute extra current.
     * @param extraCurrent The extra current to distribute.
     */
    public void scaleUp(double extraCurrent) {
        for (ControllableSubsystem subsystem : subsystems) {
            // we can do this smartly later on
            if ((subsystem.getCurrentLimit() - subsystem.minCurrent()) >= 25) {

                if (extraCurrent >= 0) {
                    double currentChange;

                    if (extraCurrent >= 100) {
                        currentChange = extraCurrent/2;
                        subsystem.setCurrentLimit(subsystem.getCurrentLimit() + (int) Math.ceil(extraCurrent / 2));
                    } else {
                        currentChange = extraCurrent;
                        subsystem.setCurrentLimit(subsystem.getCurrentLimit() + (int) Math.ceil(extraCurrent));
                    }

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
    public boolean higherPriority(ControllableSubsystem inQuestion, ControllableSubsystem baseline) {
        // If the dynamic priorities are the same, break ties with baseline priority
        if (getDynamicPriority(inQuestion) == getDynamicPriority(baseline))
            return getBasePriority(inQuestion) > getBasePriority(baseline);
        // Otherwise, use dynamic priority
        return getDynamicPriority(inQuestion) > getDynamicPriority(baseline);
    }

    /**
     * 
     * @param subsystem
     * @param increase
     */
    public void changePriority(ControllableSubsystem subsystem, boolean increase) {
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

    private void setBrownoutScaling(boolean scaled) {

        //scale all subsystems by set (sensible) amount
        for (ControllableSubsystem subsystem : subsystems) {
            if (scaled) {
                double tempscale = subsystem.getCurrentLimit()*0.8;
                subsystem.setCurrentLimit((int) Math.ceil(tempscale));
                
            } else { 
                double currDrawn = subsystem.getTotalCurrentDrawn();
                double tempscale = currDrawn * 0.8;
                subsystem.setCurrentLimit((int) Math.ceil(tempscale));
            }
        }
    }

    private double getBasePriority(ControllableSubsystem subsystem) {
        return priorityList.get(subsystem);
    }

    private double getDynamicPriority(ControllableSubsystem subsystem) {
        return dynamicPriorityList.get(subsystem);
    }

  /**
   * Returns the priority assigned to a subsystem.
   * @param subsystem The subsystem to check.
   * @return An int rescribing the subsystem's priority.
   */
  private int checkPriority(ControllableSubsystem subsystem) {
    if (subsystem instanceof TankSubsystem) return 1;
    /*
    if (subsystem instanceof IntakeSubsystem) return 2;
    if (subsystem instanceof ShooterSubsystem) return 3;
    if (subsystem instanceof ClimbSubsystem) return 4;
    */
    return 0;
  }
}
