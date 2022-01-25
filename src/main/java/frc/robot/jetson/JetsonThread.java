package frc.robot.jetson;

/**
 * A thread to update the JetsonConnection periodically.
 * This calls `jetson.periodic()` every 1 second.
 */
public class JetsonThread implements Runnable {
  private final JetsonConnection jetson;

  public JetsonThread(JetsonConnection jetson) {
    this.jetson = jetson;
  }

  @Override
  public void run() {
    while (true) {
      jetson.periodic();

      try {
        Thread.sleep(1000);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }
  }
}
