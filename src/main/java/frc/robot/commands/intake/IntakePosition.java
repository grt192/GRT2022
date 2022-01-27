package frc.robot.commands.intake;

/**
 * An enum representing the position of the intake, with `IntakePosition.value` representing the 
 * counterclockwise angle from straight upwards.
 * TODO: perhaps think of a better place to put this file
 */
public enum IntakePosition {
    RAISED(0), DEPLOYED(115);

    public final double value;

    private IntakePosition(double value) {
        this.value = value;
    }
}
