// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rapid React - 2022
// File: BallHandlerIntake.java
// Intent: Forms a command to intake the ball.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.BallHandler;

public class BallHandlerIntake extends CommandBase
{
 
  private BallHandler ballHandlerSubsystem;
  private Timer positionMovementTimer = new Timer();
  private Timer ballHandlingTimer = new Timer();
  private boolean positionAtIntake = false;
  private boolean ballHandlingStarted = false;
  private boolean completed = false;

  /**
  * The single argument constructor for the ball handler intake
  *
  * @param BallHandlerSubsystem - The ball storage subsystem in this robot
  */
  public BallHandlerIntake(BallHandler BallHandlerSubsystem)
  {
    this.ballHandlerSubsystem = BallHandlerSubsystem;
    addRequirements(BallHandlerSubsystem); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    this.positionMovementTimer.stop();
    this.positionMovementTimer.reset();
    this.ballHandlingTimer.stop();
    this.ballHandlingTimer.reset();
    this.ballHandlingStarted = false;
    this.positionAtIntake = false;
    this.completed = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    if(this.ballHandlerSubsystem.isDeployed())
    {
      this.positionMovementTimer.start();
      this.ballHandlerSubsystem.retractPosition();
    }
    else
    {
      this.positionAtIntake = true;
    }
   

    if(ballHandlingStarted == false &&
      (positionAtIntake || positionMovementTimer.hasElapsed(Constants.BallHandlerPneumaticsRetractCycleTimeSeconds)) )
    {
      this.ballHandlingTimer.start();
      this.ballHandlerSubsystem.storeBall();
      this.ballHandlingStarted = true;
    }

    if(ballHandlingStarted && ballHandlingTimer.hasElapsed(Constants.BallHandlerIntakeBallTimeSeconds))
    {
      this.completed = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    this.ballHandlerSubsystem.stopStoringBall();
    this.completed = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return this.completed;
  }
}
