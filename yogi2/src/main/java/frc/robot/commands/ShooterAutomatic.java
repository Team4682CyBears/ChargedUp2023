// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rapid React - 2022
// File: ShooterAutomatic.java
// Intent: Forms a command to perform shooter / intake activities based on orientation of jaws.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants;
import frc.robot.subsystems.BallStorage;
import frc.robot.subsystems.Jaws;

public class ShooterAutomatic extends CommandBase
{
  private static final double defaultVelocityBottomRpm = Constants.bottomMotorForwardHighGoalSpeedRpm;
  private static final double defaultVelocityTopRpm = Constants.topMotorForwardHighGoalSpeedRpm;
  private static final double defaultVelocityTolerance = 20.0;

  // target the shooter 
  // format: 
  // 0. minimum arm angle
  // 1. maximum arm angle
  // 2. bottom target shooter speed in rpm
  // 3. bottom shooter speed tolerance in rpm
  // 4. top target shooter speed in rpm
  // 5. top shooter speed tolerance in rpm
  private static final double shooterIntakeTargets[][] =
  {
    // format:
    // jaws position minimum, jaws position maximum, bottom wheel RPM target, bottom wheel RPM tolerance, top wheel RPM target, top wheel RPM tolerance
    {Constants.jawsIntakePositionAngle - 5.0, Constants.jawsIntakePositionAngle + 5.0, Constants.bottomMotorIntakeSpeedRpm, Constants.defaultMotorSpeedToleranceRpm, Constants.topMotorIntakeSpeedRpm, Constants.defaultMotorSpeedToleranceRpm}, // intake targets
    {Constants.jawsLowGoalPositionAngle - 5.0, Constants.jawsLowGoalPositionAngle + 5.0, Constants.bottomMotorForwardLowGoalSpeedRpm, Constants.defaultMotorSpeedToleranceRpm, Constants.topMotorForwardLowGoalSpeedRpm, Constants.defaultMotorSpeedToleranceRpm}, // low ball shooter targets
    {Constants.jawsHighGoalPositionAngle - 5.0, Constants.jawsHighGoalPositionAngle + 5.0, Constants.bottomMotorForwardHighGoalSpeedRpm, Constants.defaultMotorSpeedToleranceRpm, Constants.topMotorForwardHighGoalSpeedRpm, Constants.defaultMotorSpeedToleranceRpm}, // forward high ball shooter targets
    {Constants.jawsReverseHighGoalPositionAngle - 5.0, Constants.jawsReverseHighGoalPositionAngle + 5.0, Constants.bottomMotorReverseHighGoalSpeedRpm, Constants.defaultMotorSpeedToleranceRpm, Constants.topMotorReverseHighGoalSpeedRpm, Constants.defaultMotorSpeedToleranceRpm}, // reverse high ball shooter targets
    {Constants.jawsReverseLowGoalPositionAngle - 5.0, Constants.jawsReverseLowGoalPositionAngle + 5.0, Constants.bottomMotorReverseLowGoalSpeedRpm, Constants.defaultMotorSpeedToleranceRpm, Constants.topMotorReverseLowGoalSpeedRpm, Constants.defaultMotorSpeedToleranceRpm}, // reverse high ball shooter targets
  };

  private Shooter shooterSubsystem;
  private BallStorage ballStorageSubsystem;
  private Jaws jawsSubsystem;
  private boolean useShootingDirection = true;
  private Timer timer = new Timer();
  private boolean done = false;
  private boolean jawsTestMode = false;
  private double jawsTestAngle = 0.0;
  
  private double bottomShooterTargetVelocityRpm = ShooterAutomatic.defaultVelocityBottomRpm;
  private double bottomShooterVelocityToleranceRpm = ShooterAutomatic.defaultVelocityTolerance;
  private double topShooterTargetVelocityRpm = ShooterAutomatic.defaultVelocityTopRpm;
  private double topShooterVelocityToleranceRpm = ShooterAutomatic.defaultVelocityTolerance;

  /**
  * The two argument constructor for the shooter forward low shot
  *
  * @param ShooterSubsystem - The shooter subsystem in this robot
  * @param BallStorageSubsystem - The ball storage subsystem in this robot
  * @param JawsSubsystem - The jaws subsystem in this robot
  * @param directionIsShooting - When true the direction of ball movement in the system will be shooting/retrieval, if false ball movement is assumed to be intake/storage.
  * @param storageHasWorkingBeamBreakSensors - When true the logic will wait for ball storage subsystem to report that balls have been removed or added, else use a defined time to intake or retrieve.
  */
  public ShooterAutomatic(
      Shooter ShooterSubsystem,
      BallStorage BallStorageSubsystem,
      Jaws JawsSubsystem,
      boolean directionIsShooting)
  {
    this.shooterSubsystem = ShooterSubsystem;
    addRequirements(ShooterSubsystem);

    this.ballStorageSubsystem = BallStorageSubsystem;
    addRequirements(BallStorageSubsystem); 

    this.jawsSubsystem = JawsSubsystem;
    addRequirements(JawsSubsystem); 

    useShootingDirection = directionIsShooting;
  }

  // TODO - remove this after testing!!!
  /**
   * ONLY USE THIS CTOR DURING TESTING!!!!
   * @param ShooterSubsystem
   * @param BallStorageSubsystem
   * @param directionIsShooting
   * @param jawsTestAngle
   */
  public ShooterAutomatic(
      Shooter ShooterSubsystem,
      BallStorage BallStorageSubsystem,
      boolean directionIsShooting,
      double jawsTargetTestAngle)
  {
    this.shooterSubsystem = ShooterSubsystem;
    addRequirements(ShooterSubsystem);

    this.ballStorageSubsystem = BallStorageSubsystem;
    addRequirements(BallStorageSubsystem); 

    useShootingDirection = directionIsShooting;
    jawsTestMode = true;
    jawsTestAngle = jawsTargetTestAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
      boolean useDefault = true;
      done = false;
      timer.stop();
      timer.reset();
        // determine the target shooter velocities mapped to the current arm angle
      double currentJawsAngle = jawsTestMode ? jawsTestAngle : jawsSubsystem.getJawsAngle();
      for(int inx = 0; inx < shooterIntakeTargets.length; ++inx)
      {
          double lowBar = shooterIntakeTargets[inx][0];
          double highBar = shooterIntakeTargets[inx][1];
          if(currentJawsAngle >= lowBar && currentJawsAngle <= highBar)
          {
            double velocityFactor = useShootingDirection ? 1.0 : -1.0;
            bottomShooterTargetVelocityRpm = shooterIntakeTargets[inx][2] * velocityFactor;
            bottomShooterVelocityToleranceRpm = shooterIntakeTargets[inx][3];
            topShooterTargetVelocityRpm = shooterIntakeTargets[inx][4] * velocityFactor;
            topShooterVelocityToleranceRpm = shooterIntakeTargets[inx][5];
            useDefault = false;
            break;
          }
      }
      if(useDefault)
      {
        bottomShooterTargetVelocityRpm = ShooterAutomatic.defaultVelocityBottomRpm;
        bottomShooterVelocityToleranceRpm = ShooterAutomatic.defaultVelocityTolerance;
        topShooterTargetVelocityRpm = ShooterAutomatic.defaultVelocityTopRpm;
        topShooterVelocityToleranceRpm = ShooterAutomatic.defaultVelocityTolerance;
      }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    if(done == false)
    {
      shooterSubsystem.setShooterVelocityBottom(this.bottomShooterTargetVelocityRpm);
      shooterSubsystem.setShooterVelocityTop(this.topShooterTargetVelocityRpm);

      // when the PID's say speed is at setpoint / with tolerance then call retrieve
      if(shooterSubsystem.isShooterVelocityUpToSpeedBottom(this.bottomShooterTargetVelocityRpm, this.bottomShooterVelocityToleranceRpm) &&
         shooterSubsystem.isShooterVelocityUpToSpeedTop(this.topShooterTargetVelocityRpm, this.topShooterVelocityToleranceRpm))
      {
        timer.start();
        double targetElapsedTime = 0.0;
        if(this.useShootingDirection)
        {
          ballStorageSubsystem.retrieveBallManual();
          targetElapsedTime = Constants.ballStorageRetrieveTimingSeconds;
        }
        else
        {
          ballStorageSubsystem.storeBallManual();
          targetElapsedTime = Constants.ballStorageStoreTimingSeconds;
        }

        if (timer.hasElapsed(targetElapsedTime) || done == true)
        {
          done = true;
          ballStorageSubsystem.stopBallManual();
          shooterSubsystem.stopShooter();
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    if(interrupted == true)
    {
      done = true;
    }
    ballStorageSubsystem.stopBallManual();
    shooterSubsystem.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return done;
  }

}
