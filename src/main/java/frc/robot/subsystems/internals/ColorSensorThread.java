package frc.robot.subsystems.internals;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.util.Color;

public class ColorSensorThread {
    private ColorSensorRunnable runnable;

    public ColorSensorThread(ColorSensorV3 storage, ColorSensorV3 staging) {
        this.runnable = new ColorSensorRunnable(storage, staging);

        Thread thread = new Thread(runnable);
        thread.setDaemon(true);
        thread.start();
    }

    /**
     * Gets the last storage sensor color.
     * @return The last storage sensor color.
     */
    public Color getLastStorage() {
        return runnable.lastStorage;
    }

    /**
     * Gets the last staging sensor color.
     * @return The last staging sensor color.
     */
    public Color getLastStaging() {
        return runnable.lastStaging;
    }

    class ColorSensorRunnable implements Runnable {
        ColorSensorV3 storage;
        ColorSensorV3 staging;

        Color lastStorage;
        Color lastStaging;

        public ColorSensorRunnable(ColorSensorV3 storage, ColorSensorV3 staging) {
            this.storage = storage;
            this.staging = staging;
        }

        @Override
        public void run() {
            while (true) {
                this.lastStorage = storage.getColor();
                this.lastStaging = staging.getColor();
            }
        }
    }
}
