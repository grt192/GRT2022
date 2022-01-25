package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.subsystems.tank.TankSubsystem;
import static frc.robot.Constants.TankConstants.*;

public class PowerController {

    private static PowerDistribution PDP = new PowerDistribution();

    private TankSubsystem tankSubsystem;

//    private ClimbSubsystem climbSubsystem;

   // private IntakeSubsystem intakeSubsystem;

   // private shooterSubsystem shooterSubsystem;

    private int currentCurrentLimit;

    private List<Object> subsystems;

    public PowerController(TankSubsystem tankSubsystem) {
        subsystems = new ArrayList<Object>();

        subsystems.add(tankSubsystem);

        this.tankSubsystem = tankSubsystem;

        currentCurrentLimit = 350;
        //should be same as limit in tank subsystem

    }

    public void check() {

        System.out.println("Checking for a brownout...");
        // Check PDP voltage; if close to a brownout:
        if (PDP.getVoltage() < 7) {
            System.out.println("close to brownout!");
            setBrownoutScaling();
        }

        //TODO Calculate total sustainable current
        double totalSustainableCurrent = 200;

        // Sum up total current drawn from all subsystems
        double totalCurrent = 0;
        double nonTankCurrent = 0;


        totalCurrent = PDP.getTotalCurrent();
        nonTankCurrent = totalCurrent - getCurrentDrawnFromPDP(fLeftMotorPort,fRightMotorPort,bLeftMotorPort,bRightMotorPort);
        System.out.println("total current drawn: " + totalCurrent + "; non tank current drawn: " + nonTankCurrent);


        //if current goes over sustainable current...
        // Set scale of the current drawn by TankSubsystem
        if (totalCurrent > totalSustainableCurrent) {

            //see if a subsystem is drawing more than its current limit of power
            //subsystem.getMinCurrentRequired

            // New scale = current available after other components draw divided by current
            // drawn by tank
            tankSubsystem.setCurrentLimit((int)(totalSustainableCurrent - nonTankCurrent));
            System.out.println("New limit for tank subsystem: " + currentCurrentLimit);
        }

      
    }

    public static double getCurrentDrawnFromPDP(int... PDPChannel) {
        double sum = 0;
        int channels = 0;

        for (int channel : PDPChannel) {
            sum += PDP.getCurrent(channel);
            channels++;
        }

        if ((channels == 16) && (sum != PDP.getTotalCurrent())) {
            System.out.println("something is wrong, total current does not match");
        }

        return sum;
    }

    private void setBrownoutScaling() {

        //scale all subsystems by set (sensible) amount

    }
}