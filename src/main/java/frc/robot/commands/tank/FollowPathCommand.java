package frc.robot.commands.tank;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.tank.TankSubsystem;

public class FollowPathCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final TankSubsystem tankSubsystem;

    public FollowPathCommand(TankSubsystem tankSubsystem) { // TODO add parameters
        this.tankSubsystem = tankSubsystem;

        addRequirements(tankSubsystem);
    }

    // TODO

    @Override
    public void initialize() {
      tankSubsystem.followPathCommand(); // TODO: add parameters whenever the method is fixed
    }
  
    @Override
    public boolean isFinished() {
      return true;
    }
}
