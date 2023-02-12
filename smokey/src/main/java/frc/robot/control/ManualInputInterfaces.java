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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.*;
import frc.robot.commands.AllStopCommand;
import frc.robot.commands.ArmToPointCommand;
import frc.robot.commands.ButtonPressCommand;

public class ManualInputInterfaces
{
  // sets joystick variables to joysticks
  private CommandXboxController driverController = new CommandXboxController(Constants.portDriverController); 
  private CommandXboxController coDriverController = new CommandXboxController(Constants.portCoDriverController); 

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
   * A method to get the arcade arm Y componet being input from humans
   * @return - a double value associated with the magnitude of the Y componet
   */
  public double getInputArcadeArmY()
  {
    // use the co drivers right X to represent the horizontal movement
    return -1.0 * coDriverController.getLeftY();
  }

  /**
   * A method to get the arcade arm Z componet being input from humans
   * @return - a double value associated with the magnitude of the Z componet
   */
  public double getInputArcadeArmZ()
  {
    // use the co drivers right Z to represent the vertical movement
    // and multiply by -1.0 as xbox reports values flipped
    return -1.0 * coDriverController.getRightY();
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
      // TODO - need to merge this code with other branch ... oh-my!!!
      if(subsystemCollection.getDriveTrainSubsystem() != null)
      {
        // Back button zeros the gyroscope
        this.driverController.back().onTrue(
          new InstantCommand(
            subsystemCollection.getNavxSubsystem()::zeroGyroscope));
      }

      // x button press will stop all
      this.driverController.x().onTrue(
        new ParallelCommandGroup(
          new AllStopCommand(
            this.subsystemCollection),
          new ButtonPressCommand(
            "driverController.x()",
            "!!!!!!!!!!!!!!!!!!!! ALL STOP !!!!!!!!!!!!!!!!!!!!!")
          )
        );      
    }
  }

  /**
   * Will attach commands to the Co Driver XBox buttons 
   */
  private void bindCommandsToCoDriverXboxButtons()
  {
    if(InstalledHardware.coDriverXboxControllerInstalled)
    {
      // right trigger stow preset for arm
      this.coDriverController.rightTrigger().onTrue(
        new ParallelCommandGroup(
          new ArmToPointCommand(
            this.subsystemCollection.getArmSubsystem(),
            Constants.armPresetPositionStowMetersY,
            Constants.armPresetPositionStowMetersZ),
          new ButtonPressCommand(
            "coDriverController.rightTrigger()",
            "arm stow")
          )
        );
      // left trigger grab preset for arm
      this.coDriverController.leftTrigger().onTrue(
        new ParallelCommandGroup(
          new ArmToPointCommand(
            this.subsystemCollection.getArmSubsystem(),
            Constants.armPresetPositionGrabMetersY,
            Constants.armPresetPositionGrabMetersZ),
          new ButtonPressCommand(
            "coDriverController.leftTrigger()",
            "arm grab")
          )
        );
      // y button press will move the arms to high score position
      this.coDriverController.y().onTrue(
        new ParallelCommandGroup(
          new ArmToPointCommand(
            this.subsystemCollection.getArmSubsystem(),
            Constants.armPresetPositionScoreHighMetersY,
            Constants.armPresetPositionScoreHighMetersZ),
          new ButtonPressCommand(
            "coDriverController.y()",
            "arm score high")
          )
        );
        // b button press will move the arms to medium score position
      this.coDriverController.b().onTrue(
        new ParallelCommandGroup(
          new ArmToPointCommand(
            this.subsystemCollection.getArmSubsystem(),
            Constants.armPresetPositionScoreMediumMetersY,
            Constants.armPresetPositionScoreMediumMetersZ),
          new ButtonPressCommand(
            "coDriverController.b()",
            "arm score medium")
          )
        );
      // a button press will drive the arms to low score position
      this.coDriverController.a().onTrue(
        new ParallelCommandGroup(
          new ArmToPointCommand(
            this.subsystemCollection.getArmSubsystem(),
            Constants.armPresetPositionScoreLowMetersY,
            Constants.armPresetPositionScoreLowMetersZ),
          new ButtonPressCommand(
            "coDriverController.a()",
            "arm score low")
          )
        );
      // x button press will stop all
      this.coDriverController.x().onTrue(
        new ParallelCommandGroup(
          new AllStopCommand(
            this.subsystemCollection),
          new ButtonPressCommand(
            "coDriverController.x()",
            "!!!!!!!!!!!!!!!!!!!! ALL STOP !!!!!!!!!!!!!!!!!!!!!")
          )
        );
    }
  }
}
