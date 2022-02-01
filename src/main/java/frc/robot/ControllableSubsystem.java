package frc.robot;

public interface ControllableSubsystem {
    double getTotalCurrentDrawn();
    double minCurrent();

    void setCurrentLimit(double limit);
    int getCurrentLimit();
}
