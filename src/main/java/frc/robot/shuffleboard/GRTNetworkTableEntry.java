package frc.robot.shuffleboard;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

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

    private final SimpleWidget tableEntryWidget;
    private final NetworkTableEntry tableEntry;
    private Object buffer;

    /**
     * Creates a GRTNetworkTableEntry from a SimpleWidget (shuffleboardTab.add()).
     * @param tableEntry The SimpleWidget to wrap.
     */
    public GRTNetworkTableEntry(SimpleWidget tableEntryWidget) {
        this.type = GRTEntryType.GET;
        this.tableEntryWidget = tableEntryWidget;
        this.tableEntry = tableEntryWidget.getEntry();

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
        this(shuffleboardTab.add(name, value));
    }

    public GRTNetworkTableEntry(ShuffleboardLayout shuffleboardLayout, String name, Object value) {
        this(shuffleboardLayout.add(name, value));
    }

    /**
     * Positions this entry at the specified column and row.
     * 
     * @param col The column of the top left cell of the layout.
     * @param row The row of the top left cell of the layout.
     * @return The shuffleboard entry, for call chaining.
     */
    public GRTNetworkTableEntry at(int col, int row) {
        tableEntryWidget.withPosition(col, row);
        return this;
    }

    /**
     * Sizes this entry to the specified width and height.
     * 
     * @param width The width of the layout.
     * @param height The height of the layout.
     * @return The shuffleboard entry, for call chaining.
     */
    public GRTNetworkTableEntry withSize(int width, int height) {
        tableEntryWidget.withSize(width, height);
        return this;
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
