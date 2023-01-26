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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.*;
import frc.robot.Constants.*;
import frc.robot.commands.DriveToAprilTagCommand;
import frc.robot.commands.DriveToRelativeLocationCommand;

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
    return driverController.getLeftY();
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
                .onTrue(new InstantCommand(subsystemCollection.getNavxSubsystem()::zeroGyroscope));
        //new JoystickButton(driverController, XboxController.Button.kStart.value)
        //        .onTrue(new InstantCommand(subsystemCollection.getCameraSubsystem()::getCameraPositionFromAprilTag));

        // TODO - we should remove the deprecated button code above when the team decideds on the spec for button type input
//        new JoystickButton(driverController, XboxController.Button.kBack.value)
//              // No requirements because we don't need to interrupt anything
//              .onTrue(new InstantCommand(subsystemCollection.getDriveTrainSubsystem()::zeroGyroscope, subsystemCollection.getDriveTrainSubsystem()));
      
        // Drive the robot forward 1 meter
        new JoystickButton(driverController, XboxController.Button.kX.value)
          .onTrue(new DriveToRelativeLocationCommand(
            subsystemCollection.getDriveTrainSubsystem(), 
            subsystemCollection.getNavxSubsystem(), 
            new Transform2d(new Translation2d(1.0, 0.0), new Rotation2d(0.0))));
        // Drive the robot right 1 meter
        new JoystickButton(driverController, XboxController.Button.kY.value)
          .onTrue(new DriveToRelativeLocationCommand(
            subsystemCollection.getDriveTrainSubsystem(), 
            subsystemCollection.getNavxSubsystem(), 
            new Transform2d(new Translation2d(0.0, 1.0), new Rotation2d(0.0))));
        // Rotate the robot clockwise 90 degrees
        new JoystickButton(driverController, XboxController.Button.kA.value)
          .onTrue(new DriveToRelativeLocationCommand(
            subsystemCollection.getDriveTrainSubsystem(), 
            subsystemCollection.getNavxSubsystem(), 
            new Transform2d(new Translation2d(0.0, 0.0), new Rotation2d(Math.PI/2))));
          
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
