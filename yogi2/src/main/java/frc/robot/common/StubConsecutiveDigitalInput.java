// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rapid React - 2022
// File: DriveTrain.java
// Intent: Forms model for the DriveTrain subsystem.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.common;

import edu.wpi.first.wpilibj.Timer;

public class StubConsecutiveDigitalInput
{

    /* *********************************************************************
    Members
    ************************************************************************/
    private Timer timer = new Timer();
    private boolean lastStatus = false;
    private int lastStatusCount = 0;

    /**
    * Constructor for ConsecutiveDigitalInput
    *
    * @param  channel - the current IO channel for the DigitalInput
    */
    public StubConsecutiveDigitalInput(int channel) 
    {
    }

    public boolean get()
    {
        boolean rtnVal = true;
        if(rtnVal != lastStatus)
        {
            lastStatus = rtnVal;
            lastStatusCount = 0;
            timer.reset();
        }
        else
        {
            ++lastStatusCount;
        }
        return rtnVal;
    }

    /**
    * method to will tell you how many times in a row this most recent status check has occured - 0 based counter
    *
    * @return the count of times that get has returned the most recent status check value 
    */
    public int getStatusCount()
    {
        return lastStatusCount;
    }

    /**
    * a method to know how long this digital input has been reporting the most recent status check
    *
    * @return the duration in seconds that get has returned the most recent status check value 
    */
    public double getStatusTimeInSeconds()
    {
        return timer.get();
    }
}
