package frc.robot.odometry;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import static frc.robot.Constants.TankConstants.*;

public class Odometry {
  private Point location;

  private final WPI_TalonSRX leftMain = new WPI_TalonSRX(fLeftMotorPort);
  private final WPI_TalonSRX leftFollow = new WPI_TalonSRX(bLeftMotorPort);

  private final WPI_TalonSRX rightMain = new WPI_TalonSRX(fRightMotorPort);
  private final WPI_TalonSRX rightFollow = new WPI_TalonSRX(bRightMotorPort);

  public Odometry() {
    OdometryThread odoThread = new OdometryThread(this);

    // Start odometry thread
    // Do we want to keep this here or only start the thread after some init method has been called?
    new Thread(odoThread).start();
  }

  /**
   * Get the robot's current location
   * @return a Point representing the robot's current location
   */
  public Point getRobotLocation() {
    return location;
  }
}

// Point data class to represent (x, y) position
class Point {
  public int x;
  public int y;

  public Point(int x, int y) {
    this.x = x;
    this.y = y;
  }
}
