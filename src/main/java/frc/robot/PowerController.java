package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.subsystems.tank.TankSubsystem;
import static frc.robot.Constants.TankConstants.*;

public class PowerController {

    private static PowerDistribution PDP = new PowerDistribution();

    private ControllableSubsystem tankSubsystem;

//    private ControllableSubsystem climbSubsystem;

   // private ControllableSubsystem intakeSubsystem;

   // private ControllableSubsystem shooterSubsystem;

    private int currentCurrentLimit;

    private int totalSustainableCurrent;

    //TODO find current max value


    private List<ControllableSubsystem> subsystems;

    public PowerController(ControllableSubsystem tankSubsystem) {
        subsystems = new ArrayList<ControllableSubsystem>();

        subsystems.add(tankSubsystem);

        this.tankSubsystem = tankSubsystem;

        currentCurrentLimit = 350;
        //should be same as limit in tank subsystem

        //TODO Calculate total sustainable current
        totalSustainableCurrent = 200;

    }

    public void check() {

        System.out.println("Checking for a brownout...");
        // Check PDP voltage; if close to a brownout:
        if (PDP.getVoltage() < 7) {
            System.out.println("close to brownout!");
            setBrownoutScaling();
        }

        // Sum up total current drawn from all subsystems
        int totalCurrent = (int)PDP.getTotalCurrent();
        int nonTankCurrent = totalCurrent - getCurrentDrawnFromPDP(fLeftMotorPort,fRightMotorPort,bLeftMotorPort,bRightMotorPort);
        System.out.println("total current drawn: " + totalCurrent + 
                            "; non tank current drawn: " + nonTankCurrent);

        //if current goes over sustainable current...
        // Set scale of the current drawn by TankSubsystem
        if (totalCurrent > totalSustainableCurrent) {

            scale(totalCurrent, null);

            /* // New scale = current available after other components draw divided by current
            // drawn by tank
            tankSubsystem.setCurrentLimit((totalSustainableCurrent - nonTankCurrent));
            System.out.println("New limit for tank subsystem: " + currentCurrentLimit);
            */
        }

      
    }

    public static int getCurrentDrawnFromPDP(int... PDPChannel) {
        int sum = 0;
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
        for (ControllableSubsystem subsystem : subsystems) {
            int currDrawn = subsystem.getTotalCurrentDrawn();
            int tempscale = currDrawn - (int)(currDrawn*0.2);
            subsystem.setCurrentLimit(tempscale);

        }

    }

    private void scale(int current, ArrayList<ControllableSubsystem> checked) {
        //scale lowest priority subsystem down to sustainable current from current current

        ControllableSubsystem lowestPriority = null;
        int lowPriority = 10; //this int should be higher than the number of subsystems

        if (checked == null) {

            checked = new ArrayList<ControllableSubsystem>();

        }

        //if we've already scaled many subsystems, 
        //just scale more dramatically for everything
        if (checked.size() > 3) {
            setBrownoutScaling();
            return;
        }

        for (ControllableSubsystem subsystem : subsystems) {
            int priority = checkPriority(subsystem);
            if ((priority < lowPriority) && !(checked.contains(subsystem))) {
                lowestPriority = subsystem;
                lowPriority = priority;
            }

        }
    
        
        int currDrawn = lowestPriority.getTotalCurrentDrawn();
        lowestPriority.setCurrentLimit(lowestPriority.minCurrent());
        
        if ((current - currDrawn + lowestPriority.minCurrent()) > totalSustainableCurrent) {
            checked.add(lowestPriority);
            scale(current - currDrawn + lowestPriority.minCurrent(), checked);
        }
        

    }

    private int checkPriority(Object subsystem) {
        if (subsystem instanceof TankSubsystem) {
            return 1;
        }
        /*if (subsystem instanceof IntakeSubsystem) {
            return 2;
        }
        if (subsystem instanceof ShooterSubsystem) {
            return 3;
        }
        if (subsystem instanceof ClimbSubsystem) {
            return 4;
        }
        if (subsystem instanceof TankSubsystem) {
            return 1;
        }
        */

        return 0;

    }
}