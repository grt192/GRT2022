package frc.robot.brownout;

/**
 * A thread to continuously call `PowerController.calculateLimits()` every millisecond.
 * This time interval needs to be small as `PowerController` needs to be incredibly responsive to current draw changes.
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
                Thread.sleep(1);
            } catch (InterruptedException e) {
                System.out.println("PowerController thread sleep interrupted");
            }
        }
    }
}
