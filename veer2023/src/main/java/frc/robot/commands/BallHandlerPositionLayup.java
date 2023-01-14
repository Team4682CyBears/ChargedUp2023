// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rapid React - 2022
// File: BallHandlerPositionLayup.java
// Intent: Forms a command to intake the ball.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallHandler;

public class BallHandlerPositionLayup extends CommandBase
{
 
  private BallHandler ballHandlerSubsystem;

  /**
  * The single argument constructor for the ball handler intake
  *
  * @param BallHandlerSubsystem - The ball storage subsystem in this robot
  */
  public BallHandlerPositionLayup(BallHandler BallHandlerSubsystem)
  {
      this.ballHandlerSubsystem = BallHandlerSubsystem;
      addRequirements(BallHandlerSubsystem); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
      // force the deploy
      this.ballHandlerSubsystem.deployPosition();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return true;
  }
}
