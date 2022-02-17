package frc.robot.shuffleboard;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableValue;

/**
 * A wrapper for NetworkTableEntry to allow different classes to do the
 * accessing and the updating. This is so that subsystems can get/set
 * values from Shuffleboard inexpensively thanks to a separate thread (such as
 * one created by the ShuffleboardManager) which pushes or pulls the actual
 * value from Shuffleboard.
 * 
 * An alternative to using this class is to use listeners:
 * https://docs.wpilib.org/en/stable/docs/software/networktables/listening-for-change.html
 */
public class GRTNetworkTableEntry {
    public enum GRTEntryType {
        GET, SET
    }

    private final GRTEntryType type;

    private final NetworkTableEntry tableEntry;
    private NetworkTableValue buffer;

    public GRTNetworkTableEntry(GRTEntryType type, NetworkTableEntry tableEntry) {
        this.type = type;
        this.tableEntry = tableEntry;

        ShuffleboardManager.registerEntry(this);
    }

    public void update() {
        switch (this.type) {
            case GET:
                buffer = tableEntry.getValue();
                break;
            case SET:
                tableEntry.setValue(buffer);
                break;
        }
        ;
    }

    public NetworkTableValue getValue() {
        return buffer;
    }

    public void setValue(NetworkTableValue value) {
        this.buffer = value;
    }
}
