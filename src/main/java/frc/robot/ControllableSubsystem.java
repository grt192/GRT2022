package frc.robot;

public interface ControllableSubsystem {
  int getTotalCurrentDrawn();
  int minCurrent();
  void setCurrentLimit(double limit);
  double getCurrentLimit();
}
