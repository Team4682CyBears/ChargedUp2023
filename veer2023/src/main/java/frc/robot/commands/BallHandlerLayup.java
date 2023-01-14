// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rapid React - 2022
// File: BallHandlerLayup.java
// Intent: Forms a command to do a layup with the ball into the low goal.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.BallHandler;

public class BallHandlerLayup extends CommandBase
{
 
  private BallHandler ballHandlerSubsystem;
  private Timer positionMovementTimer = new Timer();
  private Timer ballHandlingTimer = new Timer();
  private boolean positionAtDeploy = false;
  private boolean ballHandlingStarted = false;
  private boolean completed = false;

  /**
  * The single argument constructor for the ball handler intake
  *
  * @param BallHandlerSubsystem - The ball storage subsystem in this robot
  */
  public BallHandlerLayup(BallHandler BallHandlerSubsystem)
  {
    this.ballHandlerSubsystem = BallHandlerSubsystem;
    addRequirements(BallHandlerSubsystem); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    positionMovementTimer.stop();
    positionMovementTimer.reset();
    ballHandlingTimer.stop();
    ballHandlingTimer.reset();
    ballHandlingStarted = false;
    positionAtDeploy = false;
    completed = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    if(this.ballHandlerSubsystem.isRetracted())
    {
      this.positionMovementTimer.start();
      this.ballHandlerSubsystem.deployPosition();
    }
    else
    {
      this.positionAtDeploy = true;
    }
   

    if(ballHandlingStarted == false &&
      (positionAtDeploy || positionMovementTimer.hasElapsed(Constants.BallHandlerPneumaticsDeployCycleTimeSeconds)) )
    {
      this.ballHandlingTimer.start();
      this.ballHandlerSubsystem.storeBall();
      this.ballHandlingStarted = true;
    }

    if(ballHandlingStarted && ballHandlingTimer.hasElapsed(Constants.BallHandlerLayupBallTimeSeconds))
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
