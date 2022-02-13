package frc.robot.shuffleboard;

import static frc.robot.Constants.ShuffleboardConstants.UPDATE_TIME;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;

public class ShuffleboardManager {
    private static final ArrayList<ShuffleboardProvider> providers = new ArrayList<>();

    public static void registerProvider(ShuffleboardProvider provider) {
        synchronized (providers) {
            providers.add(provider);
            provider.setUpShuffleboardCommands();
        }
    }

    public static void removeProvider(ShuffleboardProvider provider) {
        synchronized (providers) {
            providers.remove(provider);
        }
    }

    public ShuffleboardManager() {
        ShuffleboardRunnable runnable = new ShuffleboardRunnable();

        Thread thread = new Thread(runnable);
        thread.setDaemon(true);
        thread.start();
    }

    class ShuffleboardRunnable implements Runnable {
        @Override
        public void run() {
            while (true) {
                double nextLoop = Timer.getFPGATimestamp() + UPDATE_TIME;

                synchronized (providers) {
                    for (ShuffleboardProvider provider : providers) {
                        for (GRTNetworkTableEntry entry : provider.shuffleboardEntries()) {
                            entry.updateValue();
                        }
                    }
                }

                double currentTime = Timer.getFPGATimestamp();
                if (currentTime <= nextLoop) {
                    try {
                        Thread.sleep((long) ((nextLoop - currentTime) * 1000));
                    } catch (InterruptedException e) {
                        System.out.println(e.toString());
                        // idk what to do here
                    }
                }
            }
        }

    }
}
