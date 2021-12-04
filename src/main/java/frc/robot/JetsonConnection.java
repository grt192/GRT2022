package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class JetsonConnection {

    public static String TABLE_NAME = "jetson";
    private NetworkTable table;

    public JetsonConnection() {
        table = NetworkTableInstance.getDefault().getTable(TABLE_NAME);
    }

    public void run() {

    }

    public double getDouble(String key) {
        System.out.println("/" + TABLE_NAME + "/" + key + " = " + table.getEntry(key).getDouble(0));
        return table.getEntry(key).getDouble(0);
    }

}
