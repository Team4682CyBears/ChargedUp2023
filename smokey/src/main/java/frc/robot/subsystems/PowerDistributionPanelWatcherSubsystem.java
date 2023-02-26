// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: PowerDistributionPanelWatcherSubsystem.java
// Intent: Forms util class to watch PDP ports for overcrrent.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.PortSpy;

public class PowerDistributionPanelWatcherSubsystem extends SubsystemBase {
   private static PowerDistribution distroPannel;
    private ArrayList<PortSpy> myList = new ArrayList<PortSpy>();

    public PowerDistributionPanelWatcherSubsystem(ModuleType type) {
        PowerDistributionPanelWatcherSubsystem.distroPannel = new PowerDistribution(2, type);
    }

    public void add(PortSpy spy) {
        myList.add(spy);
    }

    @Override
    public void periodic() {
        for (int counter = 0; counter < myList.size(); counter++) {
            PortSpy nextSpy = myList.get(counter);

            double current = distroPannel.getCurrent(nextSpy.getPort());
            if(current > nextSpy.getCurrentLimit())
            {
                System.out.println(
                    "Overcurrent detected for port " + nextSpy.getPort() +
                    " with maximum of " + nextSpy.getCurrentLimit() + 
                    " and actual of " + current + 
                    ". -> " + nextSpy.getActionDescription());
                // lanunch the command
                CommandScheduler.getInstance().schedule(nextSpy.getAction());
            }
        }
    }
}
