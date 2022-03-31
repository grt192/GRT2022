package frc.robot.jetson;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.Arrays;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.Pair;

import static frc.robot.Constants.JetsonConstants.*;

public class JetsonConnection {
    private final JetsonConnectionRunnable runnable;

    public JetsonConnection() {
        // Start turret HTTP stream
        HttpCamera turretCam = createCamera(jetsonIP, 5801);
        CameraServer.startAutomaticCapture(turretCam);

        runnable = new JetsonConnectionRunnable();
        Thread thread = new Thread(runnable);
        thread.setDaemon(true);
        thread.start();
    }

    /**
     * Returns whether the turret camera can see vision targets, in case there are momentary lapses in vision or the
     * hub is currently in the turntable blind spot.
     * @return Whether turret vision is working.
     */
    public boolean turretVisionWorking() { 
        return runnable.turretVisionWorking(); 
    }

    /**
     * Gets the jetson data as a pair representing [hub dist, turret theta].
     * Calling this method automatically marks the current jetson data as consumed and stale.
     * @return The jetson data, in the form of [hub dist (in), turret theta (rads)].
     */
    public Pair<Double, Double> getData() {
        return runnable.getData();
    }

    /**
     * Gets whether a ball is in range of the intake camera.
     * @return Whether a ball is in intake range.
     */
    public boolean ballDetected() {
        return runnable.ballDetected();
    }

    /**
     * Gets whether the current jetson data is stale (already consumed).
     * @return Whether the jetson data is stale.
     */
    public boolean getConsumed() {
        return runnable.getConsumed();
    }

    /**
     * Creates an MJPEG camera over the Jetson connection.
     * @param port The port to connect to.
     * @return The camera object.
     */
    private HttpCamera createCamera(String name, int port) {
        return new HttpCamera(name + " - Port " + port, "http://" + jetsonIP + ":" + port + "/?action=stream");
    }
    
    class JetsonConnectionRunnable implements Runnable {
        private Socket socket;
        private BufferedReader stdIn;
    
        // Bool for connection status
        private boolean isConnected = false;
        private boolean consumed = false;

        // Camera data
        private boolean turretVisionStatus = false; // Can the camera provide turret vision data?
        private double turretTheta = 0; // Delta theta of turret to target
        private double hubDistance = 0; // Distance to hub
        private boolean ballDetected = false; // If intake camera has detected a ball
    
        @Override
        public void run() {
            while (true) {
                try {
                    // If thread interrupted
                    if (Thread.interrupted()) return;

                    // Check if we need to connect
                    if (stdIn == null || socket == null || socket.isClosed() || !socket.isConnected() || !socket.isBound()) {
                        System.out.println("Connecting to jetson...");
                        isConnected = connect();

                        if (!isConnected) {
                            System.out.println("Unable to connect to Jetson");
                            // Wait before trying to connect again
                            Thread.sleep(2000);
                        }
                    } else {
                        readCameraData();
                    }

                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    // If interrupted, exit
                    return;
                } catch (Exception e) {
                    System.out.println("Outer exception caught in CAMERA code. Unknown error. Will keep trying to connect to socket");
                }
            }
        }

        /**
         * Attempt to connect to the jetson socket.
         * @return If the connection was successful.
         */
        private boolean connect() {
            try {
                socket = new Socket(jetsonIP, jetsonPort);
                stdIn = new BufferedReader(new InputStreamReader(socket.getInputStream()));
                System.out.println("Connected to Jetson address=" + jetsonIP + " at port=" + jetsonPort);

                return true;
            } catch (UnknownHostException e) {
                socket = null;
                stdIn = null;
            } catch (IOException e) {
                socket = null;
                stdIn = null;
            } catch (Exception e) {
                socket = null;
                stdIn = null;
                System.out.println("UNKNOWN ERROR: something went wrong in camera connection!");
            }
            return false;
        }

        /**
         * Read the camera data and update stored values.
         * data: (bool -- turret vision status, double -- turret theta, double -- hub distance, boolean -- ball detected)
         */
        private void readCameraData() throws InterruptedException {
            try {
                String in = null;
                while (stdIn.ready()) in = stdIn.readLine();

                if (in != null) {
                    String[] data = in.replace("(", "").replace(")", "").split(",");

                    System.out.println(Arrays.toString(data));

                    turretVisionStatus = strToBool(data[0]);
                    turretTheta = Double.parseDouble(data[1]);
                    hubDistance = Double.parseDouble(data[2]);
                    ballDetected = strToBool(data[3]);
                    consumed = false;
                }
            } catch (IOException e) {
                e.printStackTrace();
            } catch (NullPointerException e) {
                System.out.println("Unable to parse camera data: NullPointerException");
            } catch (NumberFormatException e) {
                System.out.println("Unable to parse camera data: NumberFormatException");
            }
        }

        /**
         * Converts string to a boolean. Either "False" or "True".
         * @param str The string to convert.
         * @return The boolean value.
         */
        private boolean strToBool(String str) {
            return str.equalsIgnoreCase("true");
        }

        /**
         * Returns whether the turret camera can see vision targets, in case there are momentary lapses in vision or the
         * hub is currently in the turntable blind spot.
         * @return Whether turret vision is working.
         */
        public boolean turretVisionWorking() {
            return isConnected && turretVisionStatus; 
        }

        public Pair<Double, Double> getData() {
            consumed = true;
            return new Pair<>(hubDistance, turretTheta);
        }

        public boolean ballDetected() {
            return ballDetected; 
        }

        public boolean getConsumed() {
            return consumed;
        }
    }
}
