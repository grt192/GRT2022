package frc.robot.shuffleboard;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

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


    /**
     * Creates a GRTNetworkTableEntry from a NetworkTableEntry.
     * @param tableEntry The NetworkTableEntry to wrap.
     */
    public GRTNetworkTableEntry(NetworkTableEntry tableEntry) {
        this.type = GRTEntryType.GET;
        this.tableEntry = tableEntry;

        ShuffleboardManager.registerEntry(this);
    }

    /**
     * Creates a GRTNetworkTableEntry from a shuffleboard tab, entry name, and initial value.
     * 
     * @param shuffleboardTab The tab to add the entry to.
     * @param name The name of the entry.
     * @param value The value of the entry.
     * @return The GRTNetworkTableEntry.
     */
    public GRTNetworkTableEntry(ShuffleboardTab shuffleboardTab, String name, Object value) {
        this(shuffleboardTab.add(name, value).getEntry());
    }

    public GRTNetworkTableEntry(ShuffleboardLayout shuffleboardLayout, String name, Object value) {
        this(shuffleboardLayout.add(name, value).getEntry());
    }

    /**
     * Creates a GRTNetworkTableEntry from a shuffleboard tab, entry name, and initial value, positioning
     * the entry at the supplied column and row. Columns and rows are 0-indexed starting from the left and
     * top respectively.
     * 
     * @param shuffleboardTab The tab to add the entry to.
     * @param name The name of the entry.
     * @param value The value of the entry.
     * @param col The column of the top left cell of the entry.
     * @param row The row of the top left cell of the entry.
     */
    public GRTNetworkTableEntry(ShuffleboardTab shuffleboardTab, String name, Object value, int col, int row) {
        this(shuffleboardTab.add(name, value).withPosition(col, row).getEntry());
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
