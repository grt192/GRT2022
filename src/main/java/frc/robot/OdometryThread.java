package frc.robot;

import frc.robot.subsystems.tank.TankSubsystem;

/**
 * Runnable thread to continually perform robot odometry
 */
public class OdometryThread implements Runnable {
  private final TankSubsystem tankSubsystem;

  public OdometryThread(TankSubsystem tankSubsystem) {
    this.tankSubsystem = tankSubsystem;
  }

  // Continually call update() while running
  public void run() {
    while (true) {
      update();

      // Tick every 1ms
      try {
        Thread.sleep(1);
      } catch (InterruptedException e) {
        System.out.println("Odometry thread interrupted");
      }
    }
  }

  public void update() {
    // TODO
  }
}