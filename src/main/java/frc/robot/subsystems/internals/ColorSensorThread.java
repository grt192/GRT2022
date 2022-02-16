package frc.robot.subsystems.internals;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.util.Color;

public class ColorSensorThread {
    private ColorSensorRunnable runnable;

    public ColorSensorThread(ColorSensorV3 storage) {
        this.runnable = new ColorSensorRunnable(storage);

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

    class ColorSensorRunnable implements Runnable {
        ColorSensorV3 storage;
        Color lastStorage;

        public ColorSensorRunnable(ColorSensorV3 storage) {
            this.storage = storage;
        }

        @Override
        public void run() {
            while (true) {
                this.lastStorage = storage.getColor();
            }
        }
    }
}
