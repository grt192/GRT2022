package frc.robot.jetson;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;

import static frc.robot.Constants.JetsonConstants.*;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.Arrays;

public class JetsonConnection implements Runnable {

    // Instance variables
    private Thread thread;
    private Socket socket;
    private BufferedReader stdIn;

    // Bool for connection status
    public boolean isConnected;

    // Camera data
    private boolean turretVisionStatus; // Can the camera provide turret vision data?
    private double turretTheta; // Delta theta of turret to target
    private double hubDistance; // Distance to hub
    private boolean ballDetected; // If intake camera has detected ball

    public JetsonConnection() {
        this.thread = new Thread(this);
        this.thread.start();

        // Init vars
        socket = null;
        stdIn = null;
        isConnected = false;

        // Init data
        turretVisionStatus = false;
        turretTheta = 0;
        hubDistance = 0;
        ballDetected = false;

        // Start camera streams
        HttpCamera turretCamera = createCamera("Turret", turretCameraPort);
        CameraServer.startAutomaticCapture(turretCamera);

        HttpCamera intakeCamera = createCamera("Intake", intakeCameraPort);
        CameraServer.startAutomaticCapture(intakeCamera);
    }

    @Override
    public void run() {
        while (true) {
            try {
                // If thread interrupted
                if (Thread.interrupted()) {
                    return;
                }

                // Check if we need to connect
                if (stdIn == null || socket == null || socket.isClosed() || !socket.isConnected()
                        || !socket.isBound()) {

                    if (!connect()) {
                        System.out.println("Unable to connect to Jetson");
                        isConnected = false;
                        // Wait before trying to connect again
                        Thread.sleep(500);
                    }   
                } else {
                    isConnected = true;
                     readCameraData();
                }
            } catch (InterruptedException e) {
                // If interrupted, exit
                return;
            } catch (Exception e) {
                System.out.println("Outer exception caught in CAMERA code. Unknown error. Will keep trying to connect to socket");
            }
        }
    }

    /**
     * Attempt to connect to socket.
     * @return connected  if connection was successful
     */
    public boolean connect() {
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
    public void readCameraData() throws InterruptedException {
        try {
            String in = stdIn.readLine();
            if (in != null) {
                String[] data = in.replace("(", "").replace(")", "").split(",");
                
                System.out.println(Arrays.toString(data));

                turretVisionStatus = strToBool(data[0]);
                turretTheta = Double.parseDouble(data[1]);
                hubDistance = Double.parseDouble(data[2]);
                ballDetected = strToBool(data[3]);
            
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
     * @param x string to convert
     * @return the bool value
     */
    private boolean strToBool(String x)
    {
        return x.equalsIgnoreCase("true") ? true : false;
    }

    /**
     * Returns true if the turret camera can see vision targets.
     * This is in case there are momentary lapses in vision.
     * @return Whether turret vision is working
     */
    public boolean getTurretVisionStatus() { return turretVisionStatus; }

    /**
     * Gets the calculated turret angle.
     * @return The desired turntable angle.
     */
    public double getTurretTheta() { return turretTheta; }

    /**
     * Gets the calculated hub distance.
     * @return The distance from the camera to the hub.
     */
    public double getHubDistance() { return hubDistance; }

    /**
     * Gets whether a ball is in range of the intake camera.
     * @return Whether a ball is in intake range.
     */
    public boolean ballDetected() { return ballDetected; }

    /**
     * Creates an MJPEG camera over the Jetson connection.
     * @param port The port to connect to.
     * @return The camera object.
     */
    public HttpCamera createCamera(String name, int port) {
        return new HttpCamera(name + " - Port " + port, "http://" + jetsonIP + ":" + port + "/?action=stream");
    }
}