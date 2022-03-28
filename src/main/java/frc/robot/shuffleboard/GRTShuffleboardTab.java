package frc.robot.shuffleboard;

import java.util.function.Consumer;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * A utility class to wrap common shuffleboard patterns. Contains methods for adding listener
 * and regular network table entries for reading and displaying data on Shuffleboard.
 */
public class GRTShuffleboardTab {
    private final ShuffleboardTab shuffleboardTab;

    public GRTShuffleboardTab(String name) {
        shuffleboardTab = Shuffleboard.getTab(name);
    }

    /**
     * Adds a network table shuffleboard entry to the tab with the supplied name and value.
     * This is typically for displaying data. For reading data, see `addListener`.
     * 
     * @param name The name of the entry.
     * @param value The initial value of the entry.
     * @return The created GRTNetworkTableEntry.
     */
    public GRTNetworkTableEntry addEntry(String name, Object value) {
        return GRTNetworkTableEntry.from(shuffleboardTab, name, value);
    }

    /**
     * Adds a listener shuffleboard entry to the tab with the supplied name and value.
     * The entry will call the callback according to the set flags (defaults to every value update).
     * Returns this tab for call chaining.
     * 
     * @param name The name of the entry.
     * @param value The initial value of the entry.
     * @param callback The callback for when the value changes.
     * @param flags The flags controlling when to call the callback. Defaults to `kUpdate`.
     * @return The shuffleboard tab, for call chaining.
     */
    public GRTShuffleboardTab addListener(String name, Object value, Consumer<EntryNotification> callback, int flags) {
        shuffleboardTab.add(name, value).getEntry().addListener(callback, flags);
        return this;
    }

    public GRTShuffleboardTab addListener(String name, Object value, Consumer<EntryNotification> callback) {
        return addListener(name, value, callback, EntryListenerFlags.kUpdate);
    }
}
