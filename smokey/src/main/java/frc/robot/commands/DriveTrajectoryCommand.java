// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rapid React - 2023
// File: DriveTrajectoryCommand.java
// Intent: Forms a command to drive the wheels according to a constructed trajectory.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;
import static java.lang.Math.abs;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveTrajectoryCommand extends CommandBase
{
  private DrivetrainSubsystem drivetrain;
  private Trajectory movementPlan;
  private Timer timer = new Timer();
  private boolean done = false;
  private double expectedDuration = 0.0;

  private PIDController xPidController = new PIDController(1.0,0.0,0.0);
  private PIDController yPidController = new PIDController(1.0,0.0,0.0);
  private Constraints movementConstraints = new TrapezoidProfile.Constraints(
    DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    DrivetrainSubsystem.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
  private ProfiledPIDController thetaPidController = new ProfiledPIDController(1.0, 0.0, 0.0, movementConstraints);
  private HolonomicDriveController controller = new HolonomicDriveController(xPidController, yPidController, thetaPidController);

  private Pose2d finalPosition = null;
  private Pose2d overTimeDelta = new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(5));
  /** 
  * Creates a new driveCommand. 
  * 
  * @param drivetrainSubsystem - the drive train subsystem
  * @param plan - the Trajectory that should be followed by the robot
  */
  public DriveTrajectoryCommand(
    DrivetrainSubsystem drivetrainSubsystem,
    Trajectory plan)
  {
    this.drivetrain = drivetrainSubsystem;
    this.movementPlan = plan;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    expectedDuration = movementPlan.getTotalTimeSeconds();
    this.finalPosition = movementPlan.sample(expectedDuration).poseMeters;
    //controller.setTolerance(new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(5)));
    timer.reset();
    timer.start();
    done = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    if(controller.atReference() == true || (timer.get() > expectedDuration && this.isDeltaReasonable()))
    {
        drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
        timer.stop();
        done = true;
    }
    else
    {
        double currentElapsedTimeInSeconds = timer.get();
        Pose2d currentLocation = drivetrain.getRobotPosition();
        Trajectory.State targetState = movementPlan.sample(currentElapsedTimeInSeconds);

        // based on docs it would appear this method is potentially the most complex one - there are multiple overloads - not sure which one to use
        // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/holonomic.html#getting-adjusted-velocities
        // https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/controller/HolonomicDriveController.html#calculate(edu.wpi.first.math.geometry.Pose2d,edu.wpi.first.math.geometry.Pose2d,double,edu.wpi.first.math.geometry.Rotation2d)
        ChassisSpeeds calculatedSpeed = controller.calculate(
            currentLocation,
            targetState.poseMeters,
            targetState.velocityMetersPerSecond,
            targetState.poseMeters.getRotation());
            
        drivetrain.drive(drivetrain.clampChassisSpeeds(calculatedSpeed));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    timer.stop();
    if(interrupted)
    {
      done = true;      
    }
    System.out.println("Movement Complete: expected duration (seconds) == " + this.expectedDuration + " actual duration (seconds) == " + timer.get());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return done;
  }

  private boolean isDeltaReasonable()
  {
    Pose2d currentPosition = this.drivetrain.getRobotPosition();
    Transform2d delta = new Transform2d(currentPosition, this.finalPosition);
    if( abs(delta.getX()) <= this.overTimeDelta.getX() &&
        abs(delta.getY()) <= this.overTimeDelta.getY() &&
        abs(MathUtil.angleModulus(delta.getRotation().getRadians())) <= MathUtil.angleModulus(this.overTimeDelta.getRotation().getRadians()))
    {
      System.out.println("isDeltaReasonable == true ...........");
      return true;
    }
    System.out.println("isDeltaReasonable == FALSE !!!!!!!!!!!!!!");
    return false;
  }
}
