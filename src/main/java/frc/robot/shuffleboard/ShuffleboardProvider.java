package frc.robot.shuffleboard;

import java.util.ArrayList;

public interface ShuffleboardProvider {
    /**
     * This method should return all the network table entries that the subsystem
     * would like the ShuffleboardManager to update. See GRTNetworkTableEntry.
     * 
     * @return
     */
    public default ArrayList<GRTNetworkTableEntry> shuffleboardEntries() {
        return new ArrayList<>();
    }

    /**
     * Override this method to add commands to the shuffleboard. Only runs once.
     */
    public default void setUpShuffleboardCommands() {
    }
}
