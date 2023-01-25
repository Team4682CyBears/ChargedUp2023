// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rapid React - 2023
// File: DriveToPointCommand.java
// Intent: Forms a command to drive the robot to a point
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveToPointCommand extends CommandBase
{
  private DrivetrainSubsystem drivetrain = null;
  private Timer timer = new Timer();
  private boolean done = false;
  private Pose2d startPosition = null;
  private Pose2d currentPosition = null;
  private Pose2d destinationPosition = null;
  private double totalDistanceMeters = 0.0;

  private double positionToleranceMeters = 0.1;
  private double rotationToleranceRadians = 1.5 * 0.01745329;

  private double accelerationMaximumDurationSeconds = 2.0;
  private double decelerationMaximumDurationSeconds = 2.0;
  private double accelerationThresholdDistanceMeters = 0.0;
  private double decelerationThresholdDistanceMeters = 0.0;
  private double targetMaximumVelocityMetersPerSecond = DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
  private double accelerationLinearRate = targetMaximumVelocityMetersPerSecond / accelerationMaximumDurationSeconds;
  private double decelerationLinearRate = -1.0 * targetMaximumVelocityMetersPerSecond / decelerationMaximumDurationSeconds;
  private double targetOperationDurationSeconds = 0.0;
  private double targetRotationRadiansPerSecond = 0.0;
  private double remainingRotationRadians = 0.0;
  
  private boolean isTrapazoidalProfile = false;

  private static final int CommandSchedulerPeriodMilliseconds = 20;
  private static final int CommandSchedulerCyclesPerSecond = 1000/CommandSchedulerPeriodMilliseconds;

  /** 
  * Creates a new driveCommand. 
  * 
  * @param drivetrainSubsystem - the drive train subsystem
  * @param targetPosition - the target destination position
  */
  public DriveToPointCommand(
    DrivetrainSubsystem drivetrainSubsystem,
    Pose2d targetDestination)
  {
    this.drivetrain = drivetrainSubsystem;
    this.destinationPosition = targetDestination;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    // init the positions
    this.startPosition = drivetrain.getRobotPosition();
    this.currentPosition = this.startPosition;

    // calculate distance thresholds
    totalDistanceMeters  = this.getRemainingDistanceMeters();

    double trapazoidalDistanceStartThresholdMeters = 
        0.5 * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * this.accelerationMaximumDurationSeconds;
    double trapazoidalDistanceEndThresholdMeters = 
        0.5 * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * this.decelerationMaximumDurationSeconds;
    double trapazoidalDistanceThresholdMeters = trapazoidalDistanceStartThresholdMeters + trapazoidalDistanceEndThresholdMeters;

    // establish if profile is trangular or trapazoidal
    isTrapazoidalProfile = (totalDistanceMeters > trapazoidalDistanceThresholdMeters);

    if(isTrapazoidalProfile)
    {
        accelerationThresholdDistanceMeters = totalDistanceMeters - trapazoidalDistanceStartThresholdMeters;
        decelerationThresholdDistanceMeters = trapazoidalDistanceEndThresholdMeters;
        targetMaximumVelocityMetersPerSecond = DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
        accelerationLinearRate = targetMaximumVelocityMetersPerSecond / accelerationMaximumDurationSeconds;
        decelerationLinearRate = targetMaximumVelocityMetersPerSecond / decelerationMaximumDurationSeconds;
        // get target duration of operation
        targetOperationDurationSeconds = 
            this.accelerationMaximumDurationSeconds + 
            this.decelerationMaximumDurationSeconds + 
            (totalDistanceMeters - trapazoidalDistanceStartThresholdMeters - trapazoidalDistanceEndThresholdMeters) / this.targetMaximumVelocityMetersPerSecond;
    }
    else
    {
        targetMaximumVelocityMetersPerSecond = (totalDistanceMeters / (0.5*this.accelerationMaximumDurationSeconds + 0.5*this.decelerationMaximumDurationSeconds));
        accelerationThresholdDistanceMeters = 0.5 * targetMaximumVelocityMetersPerSecond * this.accelerationMaximumDurationSeconds;
        decelerationThresholdDistanceMeters = accelerationThresholdDistanceMeters;
        accelerationLinearRate = targetMaximumVelocityMetersPerSecond / accelerationMaximumDurationSeconds;
        decelerationLinearRate = -1.0 * targetMaximumVelocityMetersPerSecond / decelerationMaximumDurationSeconds;
        // get target duration of operation
        targetOperationDurationSeconds = this.accelerationMaximumDurationSeconds + this.decelerationMaximumDurationSeconds;
    }


    // get remaining spin
    this.remainingRotationRadians = this.getRemainingRotationRadians();
    this.targetRotationRadiansPerSecond =
        (this.remainingRotationRadians / this.targetOperationDurationSeconds > DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND) ?
        DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND:
        this.remainingRotationRadians / (this.targetOperationDurationSeconds * CommandSchedulerCyclesPerSecond);

    // make this class start executing
    done = false;
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    // get current position
    this.currentPosition = drivetrain.getRobotPosition();
    // get recent velocity
    double recentVelocity = drivetrain.getRecentAverageVelocityInMetersPerSecond(CommandSchedulerPeriodMilliseconds * 10); // last 10 samples ... 200 ms
    // get current distance
    double remainingDistance = this.getRemainingDistanceMeters();

    // establish the resultant velocity
    double targetResultantVelocityVector = targetMaximumVelocityMetersPerSecond;
    if(remainingDistance <= this.positionToleranceMeters){
        targetResultantVelocityVector = 0.0;
    }
    else if(remainingDistance > accelerationThresholdDistanceMeters){
        targetResultantVelocityVector = recentVelocity + accelerationLinearRate;
    }
    else if(remainingDistance < decelerationThresholdDistanceMeters) {
        targetResultantVelocityVector = recentVelocity + decelerationLinearRate;
    }
    else{
        targetResultantVelocityVector = targetMaximumVelocityMetersPerSecond;
    }

    // establish the next spin
    this.remainingRotationRadians = this.getRemainingRotationRadians();
    double targetSpinRadiansPerSecond = (remainingRotationRadians <= this.rotationToleranceRadians) ? this.targetRotationRadiansPerSecond : 0.0;

    if(targetResultantVelocityVector != 0.0 || targetSpinRadiansPerSecond != 0.0)
    {
        // get transform for current state and final state
        double xVelocity = 0.0;
        double yVelocity = 0.0;
        if(targetResultantVelocityVector != 0.0)
        {
            // get transform for current state and final state so we can scale the x and y velocity
            Transform2d currentTransform = new Transform2d(this.currentPosition, this.destinationPosition);
            xVelocity = targetResultantVelocityVector * (currentTransform.getX()/remainingDistance);
            yVelocity = targetResultantVelocityVector * (currentTransform.getY()/remainingDistance);      
        }
        drivetrain.drive(new ChassisSpeeds(xVelocity, yVelocity, targetSpinRadiansPerSecond));
    }
    else
    {
        drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
        done = true;
        timer.stop();
    }   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    if(interrupted)
    {
        drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
        done = true;      
        timer.stop();
    }
    System.out.println("Target operation duration (seconds) = " + this.targetOperationDurationSeconds + " Actual operation duration (seconds) = " + timer.get());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return done;
  }

  private double getRemainingDistanceMeters()
  {
    return currentPosition.getTranslation().getDistance(destinationPosition.getTranslation());
  }

  private double getRemainingRotationRadians()
  {
    return currentPosition.getRotation().getRadians() - destinationPosition.getRotation().getRadians();
  }
}
