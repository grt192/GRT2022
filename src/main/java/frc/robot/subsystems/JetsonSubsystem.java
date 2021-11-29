package frc.robot.subsystems;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.stream.Collectors;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.JetsonConstants.*;

public class JetsonSubsystem extends SubsystemBase {

    private boolean jetsonConnected;

    // Socket that is connected to jetson
    private Socket socket;

    // Reader that reads from socket
    private BufferedReader stdIn;

    private Map<String, String> cameraData;

    public JetsonSubsystem() {
        jetsonConnected = false;
        cameraData = new HashMap<String, String>();
    }

    @Override
    public void periodic() {
        try {
            // If jetson not connected
            if (!jetsonConnected) {
                jetsonConnected = connect();
                if (!jetsonConnected) {
                    System.out.println("UNABLE TO CONNECT TO CAMERA");

                    // If we don't connect, wait before trying to connect again
                    Thread.sleep(500);
                }
            } else { // If connected
                cameraData();
            }

        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public boolean connect() {
        boolean connected = false;
        try {
            socket = new Socket(jetsonAddress, jetsonCameraPort);
            stdIn = new BufferedReader(new InputStreamReader(socket.getInputStream()));
            System.out.println("Connected to jetson address=" + jetsonAddress + " at port=" + jetsonCameraPort);
            connected = true;
        } catch (UnknownHostException e1) {
            socket = null;
            stdIn = null;
        } catch (IOException e1) {
            socket = null;
            stdIn = null;
        } catch (Exception e) {
            socket = null;
            stdIn = null;
            System.out.println("UNKNOWN ERROR: SOMETHING WENT SERIOUSLY WRONG WHILE CONNECTING CAMERA!");
        }
        return connected;
    }

    public void cameraData() throws InterruptedException {
        try {
            String in = stdIn.readLine();
            if (in != null) {

                cameraData = convertWithStream(in);
            }
        } catch (IOException e) {
            e.printStackTrace();
        } catch (NullPointerException e) {
            System.out.println("NullPointerException: unable to parse camera data.");
        } catch (NumberFormatException e) {
            System.out.println("NumberFormatException: unable to parse camera data.");
        }
    }

    /**
     * Retrieves numerical value from camera data.
     * 
     * @param key the key
     * @return the corresponding double value
     */
    public double getDouble(String key) {
        return Double.parseDouble((String) cameraData.get(key));
    }

    /**
     * Converts a string formatted as key1=value1,key2=value2 (etc.) into a Map.
     * 
     * @param mapAsString the String to convert
     * @return the Map
     */
    private static Map<String, String> convertWithStream(String mapAsString) {
        Map<String, String> map = Arrays.stream(mapAsString.split(",")).map(entry -> entry.split("="))
                .collect(Collectors.toMap(entry -> entry[0], entry -> entry[1]));
        return map;
    }
}