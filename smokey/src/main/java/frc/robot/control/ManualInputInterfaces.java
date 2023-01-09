// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: ManualInputInterfaces.java
// Intent: Forms a class that grants access to driver controlled inputs.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.control;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.*;

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
  public double getBallMotorSpeed()
  {
    double targetMotorSpeed = 0.0;
    double intakeValue = coDriverController.getLeftTriggerAxis();
    double layupValue = coDriverController.getRightTriggerAxis();
    if(intakeValue > layupValue && intakeValue > Constants.BallHandlerMotorMinimumAbsoluteInputValue){
      targetMotorSpeed = intakeValue * Constants.BallHandlerMotorStorageDirectionMultiplier;
    }
    else if(layupValue > Constants.BallHandlerMotorMinimumAbsoluteInputValue){
      targetMotorSpeed = layupValue * Constants.BallHandlerMotorRetrievalDirectionMultiplier;
    }
    return targetMotorSpeed;
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
      // TODO - do we need anything else needed here?
      
      if(subsystemCollection.getDriveTrainSubsystem() != null)
      {
        // Back button zeros the gyroscope
        new JoystickButton(driverController, XboxController.Button.kBack.value)
              // No requirements because we don't need to interrupt anything
              .onTrue(new InstantCommand(subsystemCollection.getDriveTrainSubsystem()::zeroGyroscope, subsystemCollection.getDriveTrainSubsystem()));
      }
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
      JoystickButton buttonStart = new JoystickButton(coDriverController, XboxController.Button.kStart.value);
      JoystickButton buttonBack = new JoystickButton(coDriverController, XboxController.Button.kBack.value);

      // TODO - when we determine what buttons are for what
    }
  }
}
