package frc.robot.sensors;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.Socket;
import java.util.Arrays;

/**
 * A connection with a NVIDIA Jetson Nano using a socket connection. Reads data
 * from the connection continuosly on a new thread.
 */
public class JetsonCamera implements Runnable {
    private Socket socket;
    private BufferedReader stream;

    private Thread thread;

    private double azimuth, range, x, y;

    private String host;
    private int port;

    /**
     * Establish a connection with the Jetson and start polling data on a new
     * thread.
     * 
     * @param port
     */
    public JetsonCamera(String host, int port) {
        this.host = host;
        this.port = port;

        // Begin thread, which will automatically connect
        thread = new Thread(this);
    }

    @Override
    public void run() {
        while (true) {
            if (thread.isInterrupted()) {
                return;
            } else if (stream == null || socket.isClosed() || socket.isConnected()) {
                // Attempt to reconnect
                boolean connected = connect();

                // Sleep if connection failed
                if (!connected) {
                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            } else {
                // Get the latest data from the Jetson
                String[] data = getLatestLineFromStream().replace("(", "").replace(")", "").split(",");
                System.out.println("Jetson data: " + Arrays.toString(data));

                // TODO: clarify what 'x' and 'y' are, perhaps redo the jetson code entirely
                this.range = Double.parseDouble(data[0]);
                this.azimuth = Double.parseDouble(data[1]);
                this.x = Double.parseDouble(data[2]);
                this.y = Double.parseDouble(data[3]);
            }
        }
    }

    private String getLatestLineFromStream() {
        String latestLine = "";
        try {
            String nextLine;
            // get the latest line
            while ((nextLine = stream.readLine()) != null) {
                latestLine = nextLine;
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
        return latestLine;
    }

    /**
     * Connect to the Jetson camera.
     * 
     * @return true if the connection was successful, false if not.
     */
    private boolean connect() {
        boolean socketConnected = false;
        try {
            socket = new Socket(host, port);
            stream = new BufferedReader(new InputStreamReader(socket.getInputStream()));

            socketConnected = true;
        } catch (IOException e) {
            socket = null;
            stream = null;

            System.out.println("Jetson socket failed to connect to " + host + " at port " + port + ".");
            e.printStackTrace();
        }

        return socketConnected;
    }

    // TODO figure out why positive is to the left?????
    /**
     * Get the last retrieved calculated azimuth (angle to target) in degrees, where
     * positive is to the left
     * 
     * @return
     */
    public double getAzimuth() {
        return azimuth;
    }

    /**
     * Get the last retrieved calculated range (distance to target), in inches
     * 
     * @return
     */
    public double getRange() {
        return range;
    }

    /**
     * TODO: figure out what 'x' even means
     * 
     * @return
     */
    public double getX() {
        return x;
    }

    /**
     * TODO: figure out what 'y' even means
     * 
     * @return
     */
    public double getY() {
        return y;
    }
}
