// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rapid React - 2022
// File: TelescopingArm.java
// Intent: Forms a container that stores references to the current subsystems.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;
import frc.robot.*;

public class SubsystemCollection
{
    // declaring input classes
    private ManualInputInterfaces manualInput = null;
    private OnboardInputInterfaces onboardInput = null;

    // declaring and init subsystems  
    private AngleArms angleArms = null;
    private BallStorage ballStorage = null;
    private DriveTrain driveTrain = null;
    private Jaws jaws = null;
    private Shooter shooter = null;
    private TelescopingArms telescopingArms = null;
    private Camera camera = null;

    /**
     * Default constructor
     */
    public SubsystemCollection() {}

    public AngleArms getAngleArmsSubsystem() { return angleArms; }
    public void setAngleArmsSubsystem(AngleArms value) { angleArms = value; }
    
    public BallStorage getBallStorageSubsystem() { return ballStorage; }
    public void setBallStorageSubsystem(BallStorage value) { ballStorage = value; }

    public DriveTrain getDriveTrainSubsystem() { return driveTrain; }
    public void setDriveTrainSubsystem(DriveTrain value) { driveTrain = value; }

    public Jaws getJawsSubsystem() { return jaws; }
    public void setJawsSubsystem(Jaws value) { jaws = value; }

    public Shooter getShooterSubsystem() { return shooter; }
    public void setShooterSubsystem(Shooter value) { shooter = value; }

    public TelescopingArms getTelescopingArmsSubsystem() { return telescopingArms; }
    public void setTelescopingArmsSubsystem(TelescopingArms value) { telescopingArms = value; }

    public ManualInputInterfaces getManualInputInterfaces() { return manualInput; }
    public void setManualInputInterfaces(ManualInputInterfaces value) { manualInput = value; }

    public OnboardInputInterfaces getOnboardInputInterfaces() { return onboardInput; }
    public void setOnboardInputInterfaces(OnboardInputInterfaces value) { onboardInput = value; }

    public Camera getCameraSubsystem() { return camera; }
    public void setCameraSubsystem(Camera value) { camera = value; }
}
