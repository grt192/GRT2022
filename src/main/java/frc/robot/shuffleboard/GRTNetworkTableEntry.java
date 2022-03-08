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

    private GRTEntryType type;

    private final NetworkTableEntry tableEntry;
    private Object buffer;

    public GRTNetworkTableEntry(NetworkTableEntry tableEntry) {
        this.type = GRTEntryType.GET;
        this.tableEntry = tableEntry;

        ShuffleboardManager.registerEntry(this);
    }

    public void update() {
        switch (this.type) {
            case GET:
                buffer = tableEntry.getValue().getValue();
                break;
            case SET:
                tableEntry.setValue(buffer);
                break;
        };
    }

    public Object getValue() {
        this.type = GRTEntryType.GET;
        return buffer;
    }

    public void setValue(Object value) {
        this.type = GRTEntryType.SET;
        this.buffer = value;
    }
}
