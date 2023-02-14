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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

import frc.robot.*;
import frc.robot.commands.DriveToPointCommand;
import frc.robot.commands.DriveTrajectoryCommand;
import frc.robot.common.TestTrajectories;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AllStopCommand;
import frc.robot.commands.ArmToPointCommand;
import frc.robot.commands.ButtonPressCommand;

public class ManualInputInterfaces {

  // sets joystick variables to joysticks
  private CommandXboxController driverController = new CommandXboxController(Constants.portDriverController); 
  private CommandXboxController coDriverController = new CommandXboxController(Constants.portCoDriverController); 

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
      
      if(subsystemCollection.getDriveTrainSubsystem() != null){
        if(InstalledHardware.applyBasicDriveToPointButtonsToDriverXboxController){
          this.bindBasicDriveToPointButtonsToDriverXboxController();          
        }
        if(InstalledHardware.applyDriveTrajectoryButtonsToDriverXboxController){
          this.bindDriveTrajectoryButtonsToDriverXboxController();
        }
      }

      if(subsystemCollection.getNavxSubsystem() != null){
        // Back button zeros the gyroscope
        this.driverController.back().onTrue(
          new ParallelCommandGroup(
            new InstantCommand(
              subsystemCollection.getNavxSubsystem()::zeroGyroscope),
            new ButtonPressCommand(
              "driverController.back()",
              "zero gyroscope")
            )
          );
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
   * A method that will bind buttons for basic drive to point commands to driver controller
   */
  private void bindBasicDriveToPointButtonsToDriverXboxController() {
    // basic x translation negative one meter
    this.driverController.a().onTrue(
      new ParallelCommandGroup(
        new DriveToPointCommand(
          this.subsystemCollection.getDriveTrainSubsystem(),
          this.getTargetPosition(-1.0, 0.0, 0.0)),
        new ButtonPressCommand(
          "driverController.a()",
          "-1.0m X translation DriveToPointCommand")).withTimeout(10.0)
    );
    // basic x translation positive one meter
    this.driverController.y().onTrue(
      new ParallelCommandGroup(
        new DriveToPointCommand(
          this.subsystemCollection.getDriveTrainSubsystem(),
          this.getTargetPosition(1.0, 0.0, 0.0)),
        new ButtonPressCommand(
          "driverController.y()",
          "1.0m X translation DriveToPointCommand")).withTimeout(10.0)
    );      
    // basic y translation negative one meter
    this.driverController.b().onTrue(
      new ParallelCommandGroup(
        new DriveToPointCommand(
          this.subsystemCollection.getDriveTrainSubsystem(),
          this.getTargetPosition(0.0, -1.0, 0.0)),
        new ButtonPressCommand(
          "driverController.b()",
          "-1.0m y translation DriveToPointCommand")).withTimeout(10.0)
    );
    // basic y translation positive one meter
    this.driverController.x().onTrue(
      new ParallelCommandGroup(
        new DriveToPointCommand(
          this.subsystemCollection.getDriveTrainSubsystem(),
          this.getTargetPosition(0.0, 1.0, 0.0)),
        new ButtonPressCommand(
          "driverController.x()",
          "1.0m y translation DriveToPointCommand")).withTimeout(10.0)
    );
    // basic rotation of 90 degrees
    this.driverController.leftBumper().onTrue(
      new ParallelCommandGroup(
        new DriveToPointCommand(
          this.subsystemCollection.getDriveTrainSubsystem(),
          this.getTargetPosition(0.0, 0.0, 90.0)),
        new ButtonPressCommand(
          "driverController.leftBumper()",
          "90 degree rotation DriveToPointCommand")).withTimeout(10.0)
    );
    // basic rotation of -90 degrees
    this.driverController.rightBumper().onTrue(
      new ParallelCommandGroup(
        new DriveToPointCommand(
          this.subsystemCollection.getDriveTrainSubsystem(),
          this.getTargetPosition(0.0, 0.0, -90.0)),
        new ButtonPressCommand(
          "driverController.rightBumper()",
          "-90 degree rotation DriveToPointCommand")).withTimeout(10.0)
    );
  }

  /**
   * A method that will bind buttons to have the robot flow through various trajectories
   */
  private void bindDriveTrajectoryButtonsToDriverXboxController() {
    // trajectories
    TestTrajectories testTrajectories = new TestTrajectories(subsystemCollection.getDriveTrainSubsystem().getTrajectoryConfig());
    Trajectories trajectories = new Trajectories(subsystemCollection.getDriveTrainSubsystem()); 

    // traverse forward arc trajectory
    this.driverController.a().onTrue(
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new InstantCommand(subsystemCollection.getDriveTrainSubsystem()::zeroRobotPosition),
          new DriveTrajectoryCommand(
            this.subsystemCollection.getDriveTrainSubsystem(),
            testTrajectories.traverseForwardArc)),
        new ButtonPressCommand(
          "driverController.a()",
          "testTrajectories.traverseForwardArc")).withTimeout(10.0)
    );
    // traverse backward arc trajectory
    this.driverController.b().onTrue(
      new ParallelCommandGroup(
        new DriveTrajectoryCommand(
          this.subsystemCollection.getDriveTrainSubsystem(),
          testTrajectories.traverseBackwardArc),
        new ButtonPressCommand(
          "driverController.b()",
          "testTrajectories.traverseBackwardArc")).withTimeout(10.0)
    );
    // traverse simple forward trajectory
    this.driverController.x().onTrue(
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new InstantCommand(subsystemCollection.getDriveTrainSubsystem()::zeroRobotPosition),
          new DriveTrajectoryCommand(
            this.subsystemCollection.getDriveTrainSubsystem(),
            testTrajectories.traverseSimpleForward)),
        new ButtonPressCommand(
          "driverController.x()",
          "testTrajectories.traverseSimpleForward")).withTimeout(10.0)
    );
    // traverse simple left trajectory
    this.driverController.y().onTrue(
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new InstantCommand(subsystemCollection.getDriveTrainSubsystem()::zeroRobotPosition),
          new DriveTrajectoryCommand(
            this.subsystemCollection.getDriveTrainSubsystem(),
            testTrajectories.traverseSimpleLeft)),
        new ButtonPressCommand(
          "driverController.y()",
          "testTrajectories.traverseSimpleLeft")).withTimeout(10.0)
    );
    // traverse turn 270 trajectory
    this.driverController.leftBumper().onTrue(
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new InstantCommand(subsystemCollection.getDriveTrainSubsystem()::zeroRobotPosition),
          new DriveTrajectoryCommand(
            this.subsystemCollection.getDriveTrainSubsystem(),
            testTrajectories.traverseTurn270)),
        new ButtonPressCommand(
          "driverController.leftBumper()",
          "testTrajectories.traverseTurn270")).withTimeout(10.0)
    );
    // traverse testTrajectories.turn90
    this.driverController.rightBumper().onTrue(
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new InstantCommand(subsystemCollection.getDriveTrainSubsystem()::zeroRobotPosition),
          new DriveTrajectoryCommand(
            this.subsystemCollection.getDriveTrainSubsystem(),
            testTrajectories.turn90)),
        new ButtonPressCommand(
          "driverController.rightBumper()",
          "trajectories.BluStart")).withTimeout(10.0)
    );
  }

  /**
   * A method to do the transformation of current robot position to another position
   * @param xTranslation
   * @param yTranslation
   * @param rotationDegrees
   * @return
   */
  private Pose2d getTargetPosition(double xTranslation, double yTranslation, double rotationDegrees) {
    Pose2d startPos = this.subsystemCollection.getDriveTrainSubsystem().getRobotPosition();
    Translation2d theTranslation = new Translation2d(xTranslation, yTranslation);
    Rotation2d theRotation = Rotation2d.fromDegrees(rotationDegrees);
    Transform2d theTransform = new Transform2d(theTranslation, theRotation);
    return startPos.transformBy(theTransform);
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
