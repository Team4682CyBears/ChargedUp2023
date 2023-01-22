// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rapid React - 2023
// File: DriveToAprilTagCommand.java
// Intent: Forms a command to drive the wheels to a position relative to an april tag
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveToAprilTagCommand extends CommandBase
{
  // TODO write DriveToRelativeLocation class 
  private DriveToRelativeLocationCommand driveCommand; 
  private DrivetrainSubsystem drivetrain;
  private Pose2d targetPose; 
  
  /** 
  * Creates a new driveCommand to drive to a given position offset relative to the april tag.
  * TODO specify the coordimate frame here.  
  * 
  * @param drivetrainSubsystem - the drive train subsystem
  * @param targetPose - the target pose relative to the april tag
  */
  public DriveToAprilTagCommand(
    DrivetrainSubsystem drivetrainSubsystem,
    Pose2d targetPose)
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrainSubsystem;
    addRequirements(drivetrainSubsystem);
    this.targetPose = targetPose;
  }

  /** 
  * Creates a new driveCommand to drive to the april tag with no position offset. 
  * @param drivetrainSubsystem - the drive train subsystem
  */
  public DriveToAprilTagCommand(
    DrivetrainSubsystem drivetrainSubsystem)
  {
    this(drivetrainSubsystem, new Pose2d(0.0d, 0.0d, new Rotation2d(0.0d)));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    // TODO read april tag location from limelight
    Pose2d startingPose = getRobotPositionFromAprilTag();
    // TODO calculate final relative location for robot 
    Pose2D deltaPose = targetPose.relativeTo(startingPose);
    driveCommand = new DriveToRelativeLocationCommand(drivetrain, deltaPose);
    driveCommand.initialize()
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    driveCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    driveCommand.end(interrupted);
    if(interrupted)
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return driveCommand.isFinished();
  }

