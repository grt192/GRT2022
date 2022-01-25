package frc.robot;

public interface ControllableSubsystem {
  int getTotalCurrentDrawn();
  int minCurrent();
  void setCurrentLimit(int limit);
}
