// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: ManualInputInterfaces.java
// Intent: Forms a class that grants access to driver controlled inputs.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.control;

import java.util.*;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.*;
import frc.robot.commands.ButtonPress;
import frc.robot.commands.DriveToPointCommand;
import frc.robot.commands.DriveTrajectoryCommand;
import frc.robot.common.SwerveTrajectoryGenerator;
import frc.robot.common.TestTrajectories;

public class ManualInputInterfaces{
  // sets joystick variables to joysticks
  private XboxController driverController = new XboxController(Constants.portDriverController); 
  private XboxController coDriverController = new XboxController(Constants.portCoDriverController); 

  // subsystems needed for inputs
  private SubsystemCollection subsystemCollection = null;

  /**
   * The constructor to build this 'manual input' conduit
   */
  public ManualInputInterfaces(SubsystemCollection currentCollection){
    subsystemCollection = currentCollection;
  }

  /**
   * A method to get the arcade drive X componet being input from humans
   * @return - a double value associated with the magnitude of the x componet
   */
  public double getInputArcadeDriveX(){
    return driverController.getLeftX();
  }

  /**
   * A method to get the arcade drive X componet being input from humans
   * @return - a double value associated with the magnitude of the x componet
   */
  public double getInputArcadeDriveY(){
    return driverController.getLeftY();
  }

  /**
   * A method to get the spin drive X componet being input from humans
   * @return - a double value associated with the magnitude of the x componet
   */
  public double getInputSpinDriveX(){
    return driverController.getRightX();
  }

  /**
   * A method to initialize various commands to the numerous buttons.
   * Need delayed bindings as some subsystems during testing won't always be there.
   */
  public void initializeButtonCommandBindings()
  {
    // Configure the driver xbox controller bindings
    if(InstalledHardware.driverXboxControllerInstalled){
      this.bindCommandsToDriverXboxButtons();
    }

    // Configure the co-driver xbox controller bindings
    if(InstalledHardware.coDriverXboxControllerInstalled){
      this.bindCommandsToCoDriverXboxButtons();
    }
  }

  /**
   * Will attach commands to the Driver XBox buttons 
   */
  private void bindCommandsToDriverXboxButtons(){
    if(InstalledHardware.driverXboxControllerInstalled){
      // TODO - do we need anything else needed here?
      
      if(subsystemCollection.getDriveTrainSubsystem() != null){
        // Back button zeros the gyroscope
        new Button(driverController::getBackButton)
                // No requirements because we don't need to interrupt anything
                .whenPressed(subsystemCollection.getNavxSubsystem()::zeroGyroscope);
        // cancel all commands bound to left trigger
        BooleanSupplier bs = () -> driverController.getLeftTriggerAxis()>0.1;
        new Trigger(bs).onTrue(new InstantCommand(CommandScheduler.getInstance()::cancelAll));

        // TODO - we should remove the deprecated button code above when the team decideds on the spec for button type input
//        new JoystickButton(driverController, XboxController.Button.kBack.value)
//              // No requirements because we don't need to interrupt anything
//              .onTrue(new InstantCommand(subsystemCollection.getDriveTrainSubsystem()::zeroGyroscope, subsystemCollection.getDriveTrainSubsystem()));

        if(InstalledHardware.applyBasicDriveToPointButtonsToDriverXboxController){
          this.bindBasicDriveToPointButtonsToDriverXboxController();          
        }
        if(InstalledHardware.applyDriveTrajectoryButtonsToDriverXboxController){
          this.bindDriveTrajectoryButtonsToDriverXboxController();
        }
        if(InstalledHardware.applyDriveZeroPositionButtonToDriverXboxController){
          this.bindDriveZeroPositionButtonToDriverXboxController();
        }
      }
    }
  }

  /**
   * A method that will bind buttons for basic drive to point commands to driver controller
   */
  private void bindBasicDriveToPointButtonsToDriverXboxController(){
      JoystickButton buttonX = new JoystickButton(driverController, XboxController.Button.kX.value);
      JoystickButton buttonY = new JoystickButton(driverController, XboxController.Button.kY.value);
      JoystickButton buttonA = new JoystickButton(driverController, XboxController.Button.kA.value);
      JoystickButton buttonB = new JoystickButton(driverController, XboxController.Button.kB.value);
      JoystickButton buttonLeftBumper = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);
      JoystickButton buttonRightBumper = new JoystickButton(driverController, XboxController.Button.kRightBumper.value);
      
      buttonA.whenReleased(
        new ParallelCommandGroup(
          new DriveToPointCommand(
            this.subsystemCollection.getDriveTrainSubsystem(),
            this.getTargetPosition(-1.0, 0.0, 0.0)),
          new ButtonPress("driverController", "kA.whenReleased")).withTimeout(10.0)
      );

      buttonY.whenReleased(
        new ParallelCommandGroup(
          new DriveToPointCommand(
            this.subsystemCollection.getDriveTrainSubsystem(),
            this.getTargetPosition(1.0, 0.0, 0.0)),
          new ButtonPress("driverController", "kY.whenReleased")).withTimeout(10.0)
      );

      buttonB.whenReleased(
        new ParallelCommandGroup(
          new DriveToPointCommand(
            this.subsystemCollection.getDriveTrainSubsystem(),
            this.getTargetPosition(0.0, -1.0, 0.0)),
          new ButtonPress("driverController", "kB.whenReleased")).withTimeout(10.0)
      );

      buttonX.whenReleased(
        new ParallelCommandGroup(
          new DriveToPointCommand(
            this.subsystemCollection.getDriveTrainSubsystem(),
            this.getTargetPosition(0.0, 1.0, 0.0)),
          new ButtonPress("driverController", "kX.whenReleased")).withTimeout(10.0)
      );

      buttonLeftBumper.whenReleased(
        new ParallelCommandGroup(
          new DriveToPointCommand(
            this.subsystemCollection.getDriveTrainSubsystem(),
            this.getTargetPosition(0.0, 0.0, 90.0)),
          new ButtonPress("driverController", "kLeftBumper.whenReleased")).withTimeout(10.0)
      );

      buttonRightBumper.whenReleased(
        new ParallelCommandGroup(
          new DriveToPointCommand(
            this.subsystemCollection.getDriveTrainSubsystem(),
            this.getTargetPosition(0.0, 0.0, -90.0)),
          new ButtonPress("driverController", "kRightBumper.whenReleased")).withTimeout(10.0)
      );
  }

    /**
   * A method that will bind buttons for basic drive to point commands to driver controller
   */
  private void bindDriveTrajectoryButtonsToDriverXboxController(){
      JoystickButton buttonB = new JoystickButton(driverController, XboxController.Button.kB.value);
      JoystickButton buttonA = new JoystickButton(driverController, XboxController.Button.kA.value);
      JoystickButton buttonX = new JoystickButton(driverController, XboxController.Button.kX.value);
      JoystickButton buttonY = new JoystickButton(driverController, XboxController.Button.kY.value);
      JoystickButton buttonLeftBumper = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);
      JoystickButton buttonRightBumper = new JoystickButton(driverController, XboxController.Button.kRightBumper.value);
      TestTrajectories testTrajectories = new TestTrajectories(subsystemCollection.getDriveTrainSubsystem().getTrajectoryConfig());
      
      buttonA.whenReleased(
        new ParallelCommandGroup(
          new DriveTrajectoryCommand(
            this.subsystemCollection.getDriveTrainSubsystem(),
            testTrajectories.traverseForwardArc),
          new ButtonPress("driverController", "kA.whenReleased"))
      );

      buttonB.whenReleased(
        new ParallelCommandGroup(
          new DriveTrajectoryCommand(
            this.subsystemCollection.getDriveTrainSubsystem(),
            testTrajectories.traverseBackwardArc),
          new ButtonPress("driverController", "kB.whenReleased"))
      );

      buttonX.whenReleased(
        new ParallelCommandGroup(
          new DriveTrajectoryCommand(
            this.subsystemCollection.getDriveTrainSubsystem(),
            testTrajectories.traverseSimpleForward),
          new ButtonPress("driverController", "kX.whenReleased"))
      );

      buttonY.whenReleased(
        new ParallelCommandGroup(
          new DriveTrajectoryCommand(
            this.subsystemCollection.getDriveTrainSubsystem(),
            testTrajectories.traverseSimpleLeft),
          new ButtonPress("driverController", "kY.whenReleased"))
      );

      buttonLeftBumper.whenReleased(
        new ParallelCommandGroup(
          new DriveTrajectoryCommand(
            this.subsystemCollection.getDriveTrainSubsystem(),
            testTrajectories.traverseTurn270),
          new ButtonPress("driverController", "kLeftBumper.whenReleased"))
      );

      buttonRightBumper.whenReleased(
        new ParallelCommandGroup(
          new DriveTrajectoryCommand(
            this.subsystemCollection.getDriveTrainSubsystem(),
            testTrajectories.turn90),
          new ButtonPress("driverController", "kRightBumper.whenReleased"))
      );
  }

  /**
   * A method that will bind zero button to driver controller
   */
  private void bindDriveZeroPositionButtonToDriverXboxController(){
    // Back button zeros the gyroscope
    new Button(driverController::getStartButton)
            // Require drivetrain susbystem because this will cause crazy behavior if pressed when 
            // the drivetrain is executing a field-oriented drive command
            .whenPressed(subsystemCollection.getDriveTrainSubsystem()::zeroRobotPosition, 
            this.subsystemCollection.getDriveTrainSubsystem());
    /*
    JoystickButton buttonStart = new JoystickButton(driverController, XboxController.Button.kStart.value);
      
    buttonStart.whenReleased(
      new ParallelCommandGroup(
        new RunCommand(() -> subsystemCollection.getDriveTrainSubsystem().setRobotPosition(new Pose2d(0,0,Rotation2d.fromDegrees(0)))),
        new ButtonPress("driverController", "kStart.whenReleased"))
    );
     */
  }
  /**
   * A method to do the transformation of current robot position to another position
   * @param xTranslation
   * @param yTranslation
   * @param rotationDegrees
   * @return
   */
  private Pose2d getTargetPosition(double xTranslation, double yTranslation, double rotationDegrees){
    Pose2d startPos = this.subsystemCollection.getDriveTrainSubsystem().getRobotPosition();
    Translation2d theTranslation = new Translation2d(xTranslation, yTranslation);
    Rotation2d theRotation = Rotation2d.fromDegrees(rotationDegrees);
    Transform2d theTransform = new Transform2d(theTranslation, theRotation);
    return startPos.transformBy(theTransform);
  }
  /**
   * Will attach commands to the Co Driver XBox buttons 
   */
  private void bindCommandsToCoDriverXboxButtons(){
    if(InstalledHardware.coDriverXboxControllerInstalled){
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
