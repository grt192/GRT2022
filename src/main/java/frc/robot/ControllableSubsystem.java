package frc.robot;

public interface ControllableSubsystem {
  double getTotalCurrentDrawn();
  double minCurrent();
  void setCurrentLimit(int limit);
}
