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
import frc.robot.Constants;
import frc.robot.subsystems.BallHandler;

public class BallHandlerMotorManual extends CommandBase
{
 
  private BallHandler ballHandlerSubsystem;
  private double motorInput = 0.0;

  /**
  * The single argument constructor for the ball handler intake
  *
  * @param BallHandlerSubsystem - The ball storage subsystem in this robot
  */
  public BallHandlerMotorManual(BallHandler BallHandlerSubsystem, double motorInputValue)
  {
    this.ballHandlerSubsystem = BallHandlerSubsystem;
    this.motorInput = motorInputValue;
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
      if(Math.abs(motorInput) >= Constants.BallHandlerMotorMinimumAbsoluteInputValue)
      {
        this.ballHandlerSubsystem.setBallMotor(motorInput);
      }
      else
      {
          this.ballHandlerSubsystem.stopStoringBall();
      }
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
