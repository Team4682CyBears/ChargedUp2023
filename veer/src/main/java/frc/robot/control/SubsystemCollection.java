// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rapid React - 2022
// File: TelescopingArm.java
// Intent: Forms a container that stores references to the current subsystems.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.control;
import frc.robot.*;
import frc.robot.subsystems.*;

public class SubsystemCollection
{
    // declaring input classes
    private ManualInputInterfaces manualInput = null;

    // declaring and init subsystems  
    private BallHandler ballHandler = null;
    private DrivetrainSubsystem driveTrainSubsystem = null;

    /**
     * Default constructor
     */
    public SubsystemCollection() {}

    public BallHandler getBallHandlerSubsystem() { return ballHandler; }
    public void setBallHandlerSubsystem(BallHandler value) { ballHandler = value; }

    public DrivetrainSubsystem getDriveTrainSubsystem() { return driveTrainSubsystem; }
    public void setDriveTrainSubsystem(DrivetrainSubsystem value) { driveTrainSubsystem = value; }

    public ManualInputInterfaces getManualInputInterfaces() { return manualInput; }
    public void setManualInputInterfaces(ManualInputInterfaces value) { manualInput = value; }
}
