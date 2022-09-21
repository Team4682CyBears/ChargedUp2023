// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rapid React - 2022
// File: ShooterReverseHighShot.java
// Intent: Forms a command to shoot the ball at the high goal assuming the reverse position.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants;
import frc.robot.subsystems.BallStorage;

public class ShooterReverseHighShot extends CommandBase
{
 
  private Shooter shooterSubsystem;
  private BallStorage ballStorageSubsystem;
  private Timer timer = new Timer();

  /**
  * The two argument constructor for the shooter forward low shot
  *
  * @param ShooterSubsystem - The shooter subsystem in this robot
  * @param BallStorageSubsystem - The ball storage subsystem in this robot
  */
  public ShooterReverseHighShot(Shooter ShooterSubsystem, BallStorage BallStorageSubsystem)
  {
    this.shooterSubsystem = ShooterSubsystem;
    addRequirements(ShooterSubsystem);

    this.ballStorageSubsystem = BallStorageSubsystem;
    addRequirements(BallStorageSubsystem); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    timer.stop();
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    // when the shot method returns true it is up to sufficient speed
    if(shooterSubsystem.shootHighReverse())
    {
      timer.start();
      ballStorageSubsystem.retrieve();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    shooterSubsystem.stopShooter();
    ballStorageSubsystem.stopBallManual();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return timer.get() >= Constants.ballStorageRetrieveTimingSeconds;
  }
}