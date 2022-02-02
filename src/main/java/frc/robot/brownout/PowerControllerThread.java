package frc.robot.brownout;

/**
 * A thread to continuously call `PowerController.calculateLimits()` every second.
 */
public class PowerControllerThread implements Runnable {
    private final PowerController powerController;

    public PowerControllerThread(PowerController powerController) {
        this.powerController = powerController;
    }

    @Override
    public void run() {
        while (true) {
            powerController.calculateLimits();

            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                System.out.println("PowerController thread sleep interrupted");
            }
        }
    }
}
