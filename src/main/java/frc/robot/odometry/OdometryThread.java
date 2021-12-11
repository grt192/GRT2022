package frc.robot.odometry;

public class OdometryThread implements Runnable {
  private final Odometry odometry;

  public OdometryThread(Odometry odometry) {
    this.odometry = odometry;
  }

  // Continually update odometry's current position on a 1ms loop
  public void run() {
    while (true) {
      odometry.updateCurrentPosition();

      // Tick every 1ms
      try {
        Thread.sleep(1);
      } catch (InterruptedException e) {
        System.out.println("Odometry thread interrupted");
      }
    }
  }
}
