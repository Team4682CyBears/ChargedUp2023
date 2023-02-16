// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: TelescopingArm.java
// Intent: Forms a container that stores references to the current subsystems.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.control;
import frc.robot.subsystems.*;

public class SubsystemCollection
{
    // declaring input classes
    private ManualInputInterfaces manualInput = null;

    // declaring and init subsystems  
    private ArmSubsystem armSubsystem = null;
    private DrivetrainSubsystem driveTrainSubsystem = null;
    private GrabberSubsystem grabberSubsystem = null;
    private NavxSubsystem navxSubsystem = null;
    private StabilizerSubsystem stabilizerSubsystem = null;

    /**
     * Default constructor
     */
    public SubsystemCollection() {}

    public ArmSubsystem getArmSubsystem() { return armSubsystem; }
    public void setArmSubsystem(ArmSubsystem value) { armSubsystem = value; }

    public DrivetrainSubsystem getDriveTrainSubsystem() { return driveTrainSubsystem; }
    public void setDriveTrainSubsystem(DrivetrainSubsystem value) { driveTrainSubsystem = value; }

    public GrabberSubsystem getGrabberSubsystem() { return grabberSubsystem; }
    public void setGrabberSubsystem(GrabberSubsystem value) { grabberSubsystem = value; }

    public NavxSubsystem getNavxSubsystem() { return navxSubsystem; }
    public void setNavxSubsystem(NavxSubsystem value) { navxSubsystem = value; }

    public StabilizerSubsystem getStabilizerSubsystem() { return stabilizerSubsystem; }
    public void setStabilizerSubsystem(StabilizerSubsystem value) { stabilizerSubsystem = value; }

    public ManualInputInterfaces getManualInputInterfaces() { return manualInput; }
    public void setManualInputInterfaces(ManualInputInterfaces value) { manualInput = value; }

}
