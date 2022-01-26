package frc.robot.commands.intake;

// TODO: perhaps think of a better place to put this file
public enum IntakePosition {
  // TODO: measure these positions
  RETRACTED(0), DEPLOYED(0.5), RAISED(2);

  public final double value;

  private IntakePosition(double value) {
    this.value = value;
  }
}
