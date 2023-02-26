// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: PortSpy.java
// Intent: Forms util class to contain PDP overcrrent watch stats.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.common;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class PortSpy {
    private int portToWatch = -1;
    private double currentLimit = 0.0;
    private CommandBase action = null;

    public PortSpy(int port, double limit, CommandBase action)
    {
        this.portToWatch = port;
        this.currentLimit = limit;
        this.action = action;
    }

    public void setPort(int port) {
        this.portToWatch = port;
    }

    public int getPort() {
        return this.portToWatch;
    }

    public void setCurrentLimit(Double limit) {
        this.currentLimit = limit;
    }

    public double getCurrentLimit() {
        return this.currentLimit;
    }

    public void setAction(CommandBase action) {
        this.action = action;
    }

    public CommandBase getAction() {
        return this.action;
    }
}