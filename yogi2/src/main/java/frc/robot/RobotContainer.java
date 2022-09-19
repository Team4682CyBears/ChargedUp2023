// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rapid React - 2022
// File: RobotContainer.java
// Intent: Forms the key command initiation logic of the robot.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.builders.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer
{
  // declaring input classes
  private ManualInputInterfaces m_manualInput = null;
  private OnboardInputInterfaces m_onboardInput = null;

  // declaring and init subsystems  
  private AngleArms m_angleArms = null;
  private BallStorage m_ballStorage = null;
  private DriveTrain m_driveTrain = null;
  private Jaws m_jaws = null;
  private Shooter m_shooter = null;
  private TelescopingArms m_telescopingArms = null;
  private Camera m_camera = null;

  private SubsystemCollection m_collection = new SubsystemCollection();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer()
  {   
    // create the manual input interface
    this.initializeManualInputInterfaces();

    // create the onboard input interface
    this.initializeOnboardInputInterfaces();

    // initalize the Angle Arms
    this.initializeAngleArms();

    // initialize the ball storage
    this.initializeBallStorage();

    // initialize the drive train
    this.initializeDriveTrain();

    // initialize the jaws
    this.initializeJaws();

    // initialize the shooter
    this.initializeShooter();

    // initialize the telescoping arms
    this.initializeTelescopingArms();

    // initialize the camera system
    this.initializeCamera();
    
    // assemble all of the constructed content and insert the references into the subsystem collection
    m_collection.setAngleArmsSubsystem(m_angleArms);
    m_collection.setBallStorageSubsystem(m_ballStorage);
    m_collection.setDriveTrainSubsystem(m_driveTrain);
    m_collection.setJawsSubsystem(m_jaws);
    m_collection.setShooterSubsystem(m_shooter);
    m_collection.setTelescopingArmsSubsystem(m_telescopingArms);
    m_collection.setManualInputInterfaces(m_manualInput);
    m_collection.setOnboardInputInterfaces(m_onboardInput);
    m_collection.setCameraSubsystem(m_camera);

    // make sure that all of the buttons have appropriate commands bound to them
    if(m_manualInput != null)
    {
      m_manualInput.initializeButtonCommandBindings();
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
//    return AutonomousCommandBuilder.buildSimpleReverseDrive(m_collection);
    // TODO - the commented out line below is something we would like to test once the robot is ready!
    return AutonomousCommandBuilder.buildSimpleShootHighAndReverseDrive(m_collection);
}

  public void resetRobotWhenFmsNotPresent()
  {
    if(DriverStation.isFMSAttached() == false)
    {
      System.out.println("************************************** WARNING ******************************************");
      System.out.println("************************************** WARNING ******************************************");
      System.out.println("************************************** WARNING ******************************************");
      System.out.println("!!FORCE SENSOR RESET!! BE VERY CAREFUL!! MUST PUT ROBOT SYSTEMS IN REFERENCE POSITION!!!!");
      System.out.println("************************************** WARNING ******************************************");
      System.out.println("************************************** WARNING ******************************************");
      System.out.println("************************************** WARNING ******************************************");

      // reset drive train sensors
      if(m_driveTrain != null)
      {
        m_driveTrain.forceSensorReset();
      }

      // reset jaws sensors
      if(m_jaws != null)
      {
        m_jaws.forceSensorReset();
      }

      // reset telescoping arms sensors
      if(m_telescopingArms != null)
      {
        m_telescopingArms.forceSensorReset();
      }

      // reset angle arms sensors
      if(m_angleArms != null)
      {
        m_angleArms.forceSensorReset();
      }      
    }
  }

  private void initializeManualInputInterfaces()
  {
    if(InstalledHardware.highLevelButtonBoardInstalled ||
      InstalledHardware.lowLevelButtonBoardInstalled ||
      /* InstalledHardware.coDriverXboxControllerInstalled || */
      InstalledHardware.driverXboxControllerInstalled)
    {
      m_manualInput = new ManualInputInterfaces(m_collection);
      System.out.println("SUCCESS: initializeManualInputInterfaces");
    }
    else
    {
      System.out.println("FAIL: initializeManualInputInterfaces");
    }
  }

  private void initializeOnboardInputInterfaces()
  {
    // TODO - when limelight comes online add it here
    if(InstalledHardware.navxInstalled /* && InstalledHardware.limelightInstalled */)
    {
      m_onboardInput = new OnboardInputInterfaces();
      System.out.println("SUCCESS: initalizeOnboardInputInterfaces");
    }
    else
    {
      System.out.println("FAIL: initializeOnboardInputInterfaces");
    }
  }

  private void initializeAngleArms()
  {
    if(InstalledHardware.angleArmsMotorInstalled)
    {
      m_angleArms = new AngleArms();
      // no need for a default command as buttons control this subsystem
      System.out.println("SUCCESS: initializeAngleArms");
    }
    else
    {
      System.out.println("FAIL: initializeAngleArms");
    }
  }

  private void initializeBallStorage()
  {
    if(InstalledHardware.bottomBallStorageMotorInstalled && 
      InstalledHardware.topBallStorageMotorInstalled)
    {
      m_ballStorage = new BallStorage();
      System.out.println("SUCCESS: initializeBallStorage");
    }
    else
    {
      System.out.println("FAIL: initializeBallStorage");
    }
  }

  private void initializeDriveTrain()
  {
    if(InstalledHardware.leftFrontDriveMotorInstalled && 
      InstalledHardware.leftRearDriveMotorInstalled && 
      InstalledHardware.rightFrontDriveMotorInstalled && 
      InstalledHardware.rightRearDriveMotorInstalled)
    {
      m_driveTrain = new DriveTrain();
      // TODO - decide if drivers would rather have 'modified GTA' or 'arcade style'
      // current vote from Simeon is he likes GTA
      // prior vote from John is that he likes GTA
      // not sure if this is final or not but for now we comment out arcade and keep GTA
      /*
      m_driveTrain.setDefaultCommand(
        new RunCommand(
          () ->
          m_driveTrain.arcadeDrive(
            m_manualInput.getInputArcadeDriveY() * -1.0,
            m_manualInput.getInputArcadeDriveX()),
          m_driveTrain));
      */
      m_driveTrain.setDefaultCommand(
        new RunCommand(
          () ->
          m_driveTrain.arcadeDrive(
            m_manualInput.getGtaInputArcadeDriveY(),
            m_manualInput.getGtaInputArcadeDriveX()
            ),
          m_driveTrain));
      System.out.println("SUCCESS: initializeDriveTrain");
    }
    else
    {
      System.out.println("FAIL: initializeDriveTrain");
    }
  }

  private void initializeJaws()
  {
    if(InstalledHardware.topJawsDriveMotorInstalled &&
      InstalledHardware.bottomJawsDriveMotorInstalled)
    {
      // JAWS!!!
      m_jaws = new Jaws();
      m_jaws.setDefaultCommand(
          new RunCommand(
            () ->
            m_jaws.setJawsSpeedManual(m_manualInput.getInputJaws()),
          m_jaws));
      System.out.println("SUCCESS: initializeJaws");
    }
    else
    {
      System.out.println("FAIL: initializeJaws");
    }
  }


  private void initializeShooter()
  {
    if(InstalledHardware.topShooterDriveMotorInstalled &&
      InstalledHardware.bottomShooterDriveMotorInstalled)
    {
      m_shooter = new Shooter();
      System.out.println("SUCCESS: initializeShooter");
    }
    else
    {
      System.out.println("FAIL: initializeShooter");
    }
  }

  private void initializeTelescopingArms()
  {
    if(InstalledHardware.leftTelescopingArmsDriveMotorInstalled &&
      InstalledHardware.rightTelescopingArmsDriveMotorInstalled)
    {
      m_telescopingArms = new TelescopingArms();
      m_telescopingArms.setDefaultCommand(
          new RunCommand(
            () ->
            m_telescopingArms.setTelescopingArmsSpeedManual(m_manualInput.getInputTelescopingArms()),
          m_telescopingArms));
      System.out.println("SUCCESS: initializeTelescopingArms");
    }
    else
    {
      System.out.println("FAIL: initializeTelescopingArms");
    }
  }

  private void initializeCamera()
  {
    if(InstalledHardware.limelightInstalled)
    {
      m_camera = new Camera();
      System.out.println("SUCCESS: initializeCamera");
    }
    else
    {
      System.out.println("FAIL: initializeCamera");
    }
  }

}
