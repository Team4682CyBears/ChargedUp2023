// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rapid React - 2023
// File: DriveToAprilTagCommand.java
// Intent: Forms a command to drive the wheels to a position relative to an april tag
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.CameraSubsystem;

public class DriveToAprilTagCommand extends CommandBase
{
  // This command uses the DriveTrajectoryCommand but it cannot extend it 
  // because we don't know the trajectory at the time of construction.  
  private DriveTrajectoryCommand driveCommand; 
  private DrivetrainSubsystem drivetrain;
  private CameraSubsystem camera;
  private Transform2d relativeTransform; 
  
  /** 
  * Creates a new Command to drive to a given position offset relative to the april tag.
  * The coordinate frame uses the April Tag as the origin.  
  * 
  * @param drivetrainSubsystem - the drive train subsystem
  * @param cameraSubsystem - the camera subsystem
  * @param relativeTransform - the target pose relative to the april tag
  */
  public DriveToAprilTagCommand(
    DrivetrainSubsystem drivetrainSubsystem,
    CameraSubsystem cameraSubsystem,
    Transform2d relativeTransform)
  {
    this.drivetrain = drivetrainSubsystem;
    this.camera = cameraSubsystem;
    // camera subsystem does not need to be added as a requirement because it is read-only.
    addRequirements(drivetrainSubsystem);
    this.relativeTransform = relativeTransform;
  }

  /** 
  * Creates a new Command to drive to the april tag with no position offset. 
  * @param drivetrainSubsystem - the drive train subsystem
  * @param cameraSubsystem - the camera subsystem
  */
  public DriveToAprilTagCommand(
    DrivetrainSubsystem drivetrainSubsystem,
    CameraSubsystem cameraSubsystem)
  {
    this(drivetrainSubsystem, cameraSubsystem, 
    new Transform2d(new Translation2d(0.0, 0.0), new Rotation2d(0.0d)));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    // get pose of the robot relative to the april tag (e.g. April tag is x=0, y=0, omega=0)
    Pose2d startingPose = camera.getCameraPositionFromAprilTag();
    // calculate transform needed to get to final location. from robot -> April Tag -> final position
    Transform2d robotToAprilTagTransform = new Pose2d(0.0,0.0,new Rotation2d(0.0)).minus(startingPose);
    Transform2d robotToFinalLocationTransform = robotToAprilTagTransform.plus(relativeTransform);
    
    ArrayList<Pose2d> waypoints = new ArrayList<Pose2d>();
    waypoints.add(drivetrain.getRobotPosition());
    waypoints.add(drivetrain.getRobotPosition().transformBy(robotToFinalLocationTransform));
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints, drivetrain.getTrajectoryConfig());
    driveCommand = new DriveTrajectoryCommand(drivetrain, trajectory);
    driveCommand.initialize();
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
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return driveCommand.isFinished();
  }
}
