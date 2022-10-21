// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rapid React - 2022
// File: ManualInputInterfaces.java
// Intent: Forms a class that grants access to driver controlled inputs.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.control;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.*;
import frc.robot.subsystems.*;

public class ManualInputInterfaces
{
  // sets joystick variables to joysticks
  private XboxController driverController = new XboxController(Constants.portDriverController); 
  private XboxController coDriverController = new XboxController(Constants.portCoDriverController); 

  // subsystems needed for inputs
  private SubsystemCollection subsystemCollection = null;

  /**
   * The constructor to build this 'manual input' conduit
   */
  public ManualInputInterfaces(SubsystemCollection currentCollection)
  {
    subsystemCollection = currentCollection;
  }

  /**
   * A method to access the driver controller
   * @return XboxController for the driver.
   */
  public XboxController getDriverController(){
    return driverController;
  }

  /**
   * A method to get the arcade drive X componet being input from humans
   * @return - a double value associated with the magnitude of the x componet
   */
  public double getInputArcadeDriveX()
  {
    return driverController.getLeftX();
  }

  /**
   * A method to get the arcade drive X componet being input from humans
   * @return - a double value associated with the magnitude of the x componet
   */
  public double getInputArcadeDriveY()
  {
    // need to invert the y for all xbox controllers due to xbox controler having up as negative y axis
    return driverController.getLeftY() * -1.0;
  }

  /**
   * A method to get the spin drive X componet being input from humans
   * @return - a double value associated with the magnitude of the x componet
   */
  public double getInputSpinDriveX()
  {
    return driverController.getRightX();
  }

  /**
   * A method to get the co-driver 'intake speed' input from humans
   * @return - a double value associated with the magnitude of left trigger 
   */
  public double getIntakeSpeed()
  {
    return coDriverController.getLeftTriggerAxis() * Constants.BallHandlerMotorStorageDirectionMultiplier;
  }

  /**
   * A method to get the co-driver 'layup speed' input from humans
   * @return - a double value associated with the magnitude of right trigger 
   */
  public double getLayupSpeed()
  {
    return coDriverController.getRightTriggerAxis() * Constants.BallHandlerMotorRetrievalDirectionMultiplier;
  }

  /**
   * A method to initialize various commands to the numerous buttons.
   * Need delayed bindings as some subsystems during testing won't always be there.
   */
  public void initializeButtonCommandBindings()
  {
    // Configure the driver xbox controller bindings
    if(InstalledHardware.driverXboxControllerInstalled)
    {
      this.bindCommandsToDriverXboxButtons();
    }

    // Configure the co-driver xbox controller bindings
    if(InstalledHardware.coDriverXboxControllerInstalled)
    {
      this.bindCommandsToCoDriverXboxButtons();
    }
  }

  /**
   * Will attach commands to the Driver XBox buttons 
   */
  private void bindCommandsToDriverXboxButtons()
  {
    if(InstalledHardware.driverXboxControllerInstalled)
    {
      // TODO - do we need anything here?
    }
  }

  /**
   * Will attach commands to the Co Driver XBox buttons 
   */
  private void bindCommandsToCoDriverXboxButtons()
  {
    if(InstalledHardware.coDriverXboxControllerInstalled)
    {
      JoystickButton bumperLeft = new JoystickButton(coDriverController, XboxController.Button.kLeftBumper.value);
      JoystickButton bumperRight = new JoystickButton(coDriverController, XboxController.Button.kRightBumper.value);
      JoystickButton buttonY = new JoystickButton(coDriverController, XboxController.Button.kY.value);
      JoystickButton buttonA = new JoystickButton(coDriverController, XboxController.Button.kA.value);
      JoystickButton buttonB = new JoystickButton(coDriverController, XboxController.Button.kB.value);

      if(subsystemCollection.getBallHandlerSubsystem() != null)
      {
        // do fully automated layup 
        buttonY.whenPressed(
          new ParallelCommandGroup(
            new BallHandlerLayup(subsystemCollection.getBallHandlerSubsystem()),
            new ButtonPress("coDriverController", "buttonY.whenPressed - layup ball")));
        // do fully automated intake
        buttonA.whenPressed(
          new ParallelCommandGroup(
            new BallHandlerIntake(subsystemCollection.getBallHandlerSubsystem()),
            new ButtonPress("coDriverController", "buttonA.whenPressed - intake ball")));
        // do a-la-carte operation of moving to intake position
        bumperLeft.whenPressed(
          new ParallelCommandGroup(
            new BallHandlerPositionIntake(subsystemCollection.getBallHandlerSubsystem()),
            new ButtonPress("coDriverController", "kLeftBumper.whenPressed - intake ball")));
        // do a-la-carte operation of moving to layup position
        bumperRight.whenPressed(
          new ParallelCommandGroup(
            new BallHandlerPositionLayup(subsystemCollection.getBallHandlerSubsystem()),
            new ButtonPress("coDriverController", "kRightBumper.whenPressed - layup ball")));
        // stop the handler
        buttonB.whenPressed(
          new ParallelCommandGroup(
            new BallHandlerAllStop(subsystemCollection.getBallHandlerSubsystem()),
            new ButtonPress("coDriverController", "buttonB.whenPressed - STOP")));
      }
    }
  }
}
