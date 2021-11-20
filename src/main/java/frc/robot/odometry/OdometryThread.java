package frc.robot.odometry;

/**
 * Runnable thread to continually perform robot odometry
 */
public class OdometryThread implements Runnable {
  private final Odometry odometry;

  public OdometryThread(Odometry odometry) {
    this.odometry = odometry;
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