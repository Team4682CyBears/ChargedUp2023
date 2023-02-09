package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.control.PortSpy;

public class PowerDistributionPanelWatcher extends SubsystemBase {
    private static PowerDistribution distroPannel;
    private ArrayList<PortSpy> myList = new ArrayList<PortSpy>();

    public PowerDistributionPanelWatcher(ModuleType type) {
        PowerDistributionPanelWatcher.distroPannel = new PowerDistribution(2, type);
    }

    public void add(PortSpy spy) {
        myList.add(spy);
    }

    @Override
    public void periodic() {
        for (int counter = 0; counter < myList.size(); counter++) {
            PortSpy nextSpy = myList.get(counter);
            if(checkLimit(nextSpy.getPort(), nextSpy.getCurrentLimit()))
            {
                // lanunch the command
              CommandScheduler.getInstance().schedule(nextSpy.getAction());         
            }		
        }   	
    }

    private boolean checkLimit(int port, double currentLimit){
        double current = distroPannel.getCurrent(port);
        return current > currentLimit;
    }
}


