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
      scale(totalCurrent, new HashSet<>());
    }
  }

  /**
   * Gets the total current drawn from the PDH by the specified channels.
   * @param channels The channels to sum.
   * @return The total current drawn by the provided channels.
   */
  public static double getCurrentDrawnFromPDP(int... channels) {
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
  private void scale(double current, HashSet<ControllableSubsystem> checked) {
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

    double currDrawn = lowestPriority.getTotalCurrentDrawn();
    lowestPriority.setCurrentLimit((int) lowestPriority.minCurrent());
    
    if (current - currDrawn + lowestPriority.minCurrent() > totalSustainableCurrent) {
      checked.add(lowestPriority);
      scale(current - currDrawn + lowestPriority.minCurrent(), checked);
    }
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
