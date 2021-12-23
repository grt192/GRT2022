package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class JetsonConnection {

    public static String TABLE_NAME = "jetson";
    private NetworkTable table;

    public JetsonConnection() {
        table = NetworkTableInstance.getDefault().getTable(TABLE_NAME);
    }

    /**
     * Runs periodically in a Thread. Put print lines in here or whatever else you
     * want.
     */
    public void periodic() {

        System.out.println("test: " + getString("test"));
        System.out.println("xCentroids len: " + getDoubleArray("xCentroids").length);
        System.out.println("yCentroids len: " + getDoubleArray("yCentroids").length);
    }

    public String getString(String key) {
        return table.getEntry(key).getString("");
    }

    public double getDouble(String key) {
        System.out.println("/" + TABLE_NAME + "/" + key + " = " + table.getEntry(key).getDouble(0));
        return table.getEntry(key).getDouble(0);
    }

    public double[] getDoubleArray(String key) {
        Number[] numArr = table.getEntry(key).getNumberArray(new Number[0]);

        double[] arr = new double[0];
        for (int i = 0; i < numArr.length; i++) {
            arr[i] = numArr[i].doubleValue();
        }
        return arr;
    }

}
