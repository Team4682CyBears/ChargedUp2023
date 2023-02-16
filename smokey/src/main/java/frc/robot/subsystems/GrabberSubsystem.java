// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: GrabberSubsystem.java
// Intent: Subsystem to model the grabber pneumatics subsystem.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A class intended to model the BallHandler infrastructure on Veer.
 */
public class GrabberSubsystem extends SubsystemBase{

    private boolean currentHorizontalGrabberDeployed = false;
    private boolean currentVerticalGrabberDeployed = false;
    Compressor compressor = new Compressor(1, Constants.PneumaticsControlModuleType);
    DoubleSolenoid horizontalSolenoid = new DoubleSolenoid(
        Constants.PneumaticsControlModuleNumber,
        Constants.PneumaticsControlModuleType,
        Constants.GrabberHorizontalPneumaticsControlModuleForwardChannel,
        Constants.GrabberHorizontalPneumaticsControlModuleReverseChannel);
    DoubleSolenoid verticalSolenoid = new DoubleSolenoid(
      Constants.PneumaticsControlModuleNumber,
      Constants.PneumaticsControlModuleType,
      Constants.GrabberVerticalPneumaticsControlModuleForwardChannel,
      Constants.GrabberVerticalPneumaticsControlModuleReverseChannel);
    /**
     * No argument constructor for the BallHandler subsystem.
    */
    public GrabberSubsystem() {
      this.intitalizeGrabberState();
      CommandScheduler.getInstance().registerSubsystem(this);
    }

    /**
     * Method to obtain if the Horizontal grabber is in the retracted position.
     * @return boolean true if Horizontal grabber is retracted, otherwise false.
     */
    public boolean isHorizontalDeployed()
    {
        return currentHorizontalGrabberDeployed;
    }

    /**
     * Method to obtain if the Horizontal grabber is in the retracted position.
     * @return boolean true if Horizontal grabber is retracted, otherwise false.
     */
    public boolean isHorizontalRetracted()
    {
        return !currentHorizontalGrabberDeployed;
    }

    /**
     * Method to move the Horizontal grabber into a deployed (closed) position
     */
    public void deployHorizontalPosition(){
        this.horizontalSolenoid.set(DoubleSolenoid.Value.kForward);
        currentHorizontalGrabberDeployed = true;
    }

    /**
     * Method to move the Horizontal grabber into a retracted (open) position
     */
    public void retractHorizontalPosition(){
        this.horizontalSolenoid.set(DoubleSolenoid.Value.kReverse);
        currentHorizontalGrabberDeployed = false;
    }

    /**
     * Method to toggle the Horizontal grabber position from its current location.
     */
    public void toggleHorizontalPosition(){        
        if(currentHorizontalGrabberDeployed)
        {
            this.retractHorizontalPosition();
        }
        else
        {
            this.deployHorizontalPosition();
        }
    }

    /**
     * Method to obtain if the Vertical grabber is in the retracted position.
     * @return boolean true if Vertical grabber is retracted, otherwise false.
     */
    public boolean isVerticalDeployed()
    {
        return currentVerticalGrabberDeployed;
    }

    /**
     * Method to obtain if the Vertical grabber is in the retracted position.
     * @return boolean true if Vertical grabber is retracted, otherwise false.
     */
    public boolean isVerticalRetracted()
    {
        return !currentVerticalGrabberDeployed;
    }

    /**
     * Method to move the Vertical grabber into a deployed (upward) position
     */
    public void deployVerticalPosition(){
        this.verticalSolenoid.set(DoubleSolenoid.Value.kForward);
        currentVerticalGrabberDeployed = true;
    }

    /**
     * Method to move the Vertical grabber into a retracted (downward) position
     */
    public void retractVerticalPosition(){
        this.verticalSolenoid.set(DoubleSolenoid.Value.kReverse);
        currentVerticalGrabberDeployed = false;
    }

    /**
     * Method to toggle the Vertical grabber position from its current location.
     */
    public void toggleVerticalPosition(){        
        if(currentVerticalGrabberDeployed)
        {
            this.retractVerticalPosition();
        }
        else
        {
            this.deployVerticalPosition();
        }
    }

    /**
     * A method to handle periodic processing
     */
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("HorizontalGrabberDeployed", this.isHorizontalDeployed());
        SmartDashboard.putBoolean("VerticalGrabberDeployed", this.isVerticalDeployed());
    }
 
    private void intitalizeGrabberState()
    {
        // confirm that the double horizontalSolenoid has retracted the arm
        this.retractHorizontalPosition();
        // confirm that the double verticalSolenoid has retracted the arm
        this.retractVerticalPosition();
    }

}