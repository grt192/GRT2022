package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Enumeration;
import java.util.Hashtable;

import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.subsystems.tank.TankSubsystem;
import static frc.robot.Constants.TankConstants.*;
import java.lang.Math;

public class PowerController {

    private static PowerDistribution PDP = new PowerDistribution();

    private ControllableSubsystem tankSubsystem;

//    private ControllableSubsystem climbSubsystem;

   // private ControllableSubsystem intakeSubsystem;

   // private ControllableSubsystem shooterSubsystem;

    private double totalSustainableCurrent;
    //TODO find current max value

    private List<ControllableSubsystem> subsystems;

    private Hashtable<ControllableSubsystem, Double> priorityList;


    public PowerController(ControllableSubsystem tankSubsystem) {
        subsystems = new ArrayList<ControllableSubsystem>();

        subsystems.add(tankSubsystem);

        this.tankSubsystem = tankSubsystem;

        //TODO Calculate total sustainable current
        totalSustainableCurrent = 200;

        //initialize our priority list with default pre-game priorities
        priorityList.put(tankSubsystem, 1.0);
        /*priorityList.put(intakeSubsystem, 2);
        priorityList.put(shooterSubsystem, 3);
        priorityList.put(climbSubsystem, 4);*/

    }

    public void check() {

        System.out.println("Checking for a brownout...");
        // Check PDP voltage; if close to a brownout:
        if (PDP.getVoltage() < 7) {
            System.out.println("close to brownout!");
            setBrownoutScaling(false);
        }

        // Sum up total current drawn from all subsystems
        double totalCurrent = PDP.getTotalCurrent();
        double nonTankCurrent = totalCurrent - getCurrentDrawnFromPDP(fLeftMotorPort,fRightMotorPort,bLeftMotorPort,bRightMotorPort);
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


    private void setBrownoutScaling(boolean scaled) {

        //scale all subsystems by set (sensible) amount
        for (ControllableSubsystem subsystem : subsystems) {
            if (scaled) {
                double tempscale = subsystem.getCurrentLimit()*0.8;
                subsystem.setCurrentLimit((int)Math.ceil(tempscale));
                
            } else { 
                double currDrawn = subsystem.getTotalCurrentDrawn();
                double tempscale = currDrawn*0.8;
                subsystem.setCurrentLimit((int)Math.ceil(tempscale));
            }
        }
    }

    private void scale(double current, ArrayList<ControllableSubsystem> checked) {
        //scale lowest priority subsystem down to sustainable current from current current

        ControllableSubsystem lowestPriority = null;
        double lowPriority = 10; //this int should be higher than the number of subsystems

        if (checked == null) {

            checked = new ArrayList<ControllableSubsystem>();

        }

        //if we've already scaled many subsystems, 
        //just scale more dramatically for everything
        if (checked.size() > 3) {
            setBrownoutScaling(true);
            return;
        }

        for (ControllableSubsystem subsystem : subsystems) {
            double priority = checkPriority(subsystem);
            if ((priority < lowPriority) && !(checked.contains(subsystem))) {
                lowestPriority = subsystem;
                lowPriority = priority;
            }
        }
        
        //check current drawn from lowest priority subsystem,
        //set the subsystem's current limit to either their minimum current requirement
        //or their drawn current scaled to 80% of what it was, whichever is higher
        //(so we don't go below minimum required current)
        double currDrawn = lowestPriority.getTotalCurrentDrawn();
        double currentChange = Math.max(lowestPriority.minCurrent(),currDrawn*0.8);
        lowestPriority.setCurrentLimit(currentChange);
        
        //if the change in expected current will not be enough, 
        //scale again with the next lowest priority mech
        if ((current - (currDrawn - currentChange)) > totalSustainableCurrent) {
            checked.add(lowestPriority);
            scale(current - (currDrawn - currentChange), checked);
        }
    }

    private double checkPriority(ControllableSubsystem subsystem) {
        
        return priorityList.get(subsystem);

    }

    public void changePriority(ControllableSubsystem subsystem, boolean increase) {

        //make logic for changing priority up or down
        //and then resolving dictionary list to make sense
        Hashtable<ControllableSubsystem, Double> newList = new Hashtable<ControllableSubsystem, Double>();

        //multiply all by two then add 1 to subsystem that needs to be moved up

        //TODO fix logic

        //use a weighted average for total current??? and have a double 1-10 for each
        //then lowest average will get more current
        //from liang

        //so A gets A/A+B+C * totalcurrent
        //TODO make mincurrent an important factor in scale and if its at mincurrent scale again
        //TODO check each sub to make sure its at mincurrent
        for (Enumeration<ControllableSubsystem> subs = priorityList.keys(); subs.hasMoreElements();) {
            ControllableSubsystem sub = subs.nextElement();
            newList.put(sub, priorityList.get(sub)*2);
        }
        
        newList.put(subsystem, increase ? priorityList.get(subsystem)*2+3 : priorityList.get(subsystem)*2-3);
        priorityList = newList;

    }
}