package frc.robot.control;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class PortSpy {
    private int portToWatch = -1;
    private double currentLimit = 0.0;
    CommandBase action = null;

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