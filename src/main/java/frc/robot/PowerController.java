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

   // private ControllableSubsystem climbSubsystem;

   // private ControllableSubsystem intakeSubsystem;

   // private ControllableSubsystem shooterSubsystem;

    private double totalSustainableCurrent;
    //TODO find current max value

    private List<ControllableSubsystem> subsystems;

    private Hashtable<ControllableSubsystem, Double> priorityList;
    private Hashtable<ControllableSubsystem, Double> dynamicPriorityList;


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

        //initialize the dynamic priority list with all subsystems starting the same
        dynamicPriorityList.put(tankSubsystem, 5.0);
        /*dynamicpriorityList.put(intakeSubsystem, 5.0);
        dynamicpriorityList.put(shooterSubsystem, 5.0);
        dynamicpriorityList.put(climbSubsystem, 5.0);*/

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

        }

        //if there is extra current, undo scaling/distribute more power in case systems are scaled down
        if ((totalSustainableCurrent - totalCurrent) >= 50) {

            scaleBack(totalSustainableCurrent - totalCurrent);

        }
      
    }


    private void scale(double current, ArrayList<ControllableSubsystem> checked) {
        //scale lowest priority subsystem down to sustainable current from current current

        //use a weighted average for total current??? and have a double 1-10 for each
        //then lowest average will get more current
        //so A gets A/A+B+C * totalcurrent perhaps
        //from liang
    
        if (checked == null) {
            checked = new ArrayList<ControllableSubsystem>();
        }

        //if we've already scaled many subsystems, 
        //just scale more dramatically for everything
        if (checked.size() > 3) {
            setBrownoutScaling(true);
            return;
        }

        ControllableSubsystem lowest = null;

        for (ControllableSubsystem subsystem : subsystems) {
            
            if (checked.contains(subsystem)) {
                continue;
            }
            if (lowest == null) {
                lowest = subsystem;
                continue;
            }
            if (higherPriority(lowest, subsystem)) {
                lowest = subsystem;
            }
            
        }
        
        //check current drawn from lowest priority subsystem,
        //set the subsystem's current limit to either their minimum current requirement
        //or their drawn current scaled to 80% of what it was, whichever is higher
        //(so we don't go below minimum required current)
        double currDrawn = lowest.getTotalCurrentDrawn();
        double currentChange = Math.max(lowest.minCurrent(),currDrawn*0.8);
        lowest.setCurrentLimit(currentChange);
        
        //if the change in expected current will not be enough, 
        //scale again with the next lowest priority mech
        if ((current - (currDrawn - currentChange)) > totalSustainableCurrent) {
            checked.add(lowest);
            scale(current - (currDrawn - currentChange), checked);
        }
    }

    public void scaleBack(double extraCurrent) {

        double currentTogo = extraCurrent;

        for (ControllableSubsystem subsystem : subsystems) {

            //if a subsystem's current current limit is close to its' minimum current,
            //allot it more power

            //we can do this smartly later on
            if ((subsystem.getCurrentLimit() - subsystem.minCurrent()) >= 25) {

                if (currentTogo >= 0) {

                    double currentChange;

                    if (extraCurrent >= 100) {
                        currentChange = extraCurrent/2;
                        subsystem.setCurrentLimit(subsystem.getCurrentLimit() + extraCurrent/2);
                    } else {
                        currentChange = extraCurrent;
                        subsystem.setCurrentLimit(subsystem.getCurrentLimit() + extraCurrent);
                    }

                    currentTogo -= currentChange;

                }

            }

        }

        if (currentTogo > 25) {
            scaleBack(currentTogo);
        }

    }


    // returns if inQuestion subsystenm is higher priority than baseline subsystem
    public boolean higherPriority(ControllableSubsystem inQuestion,ControllableSubsystem baseline){
        if(getDynamicPriority(inQuestion) > getDynamicPriority(baseline)) {
            return true;
        }
        else if (getDynamicPriority(inQuestion) == getDynamicPriority(baseline)) {
            if (getBasePriority(inQuestion) > getBasePriority(baseline)) {
               return true;
            
            }
            else { 
                return false;

            }
        } else {
            return false;
        }
    
    }

    public void changePriority(ControllableSubsystem subsystem, boolean increase) {

        double priority = dynamicPriorityList.get(subsystem);
        double newPriority = increase ? priority + 2.5 : priority - 2.5;

        if (Math.abs(10.0 - newPriority) > 0) {
            if (newPriority > 0) {
                newPriority = 0.0;
            } else {
                newPriority = 10.0;
            }
        }

        dynamicPriorityList.put(subsystem, newPriority);

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

    private double getBasePriority(ControllableSubsystem subsystem) {
        
        return priorityList.get(subsystem);

    }
    private double getDynamicPriority(ControllableSubsystem subsystem) {
        
        return dynamicPriorityList.get(subsystem);

    }
}