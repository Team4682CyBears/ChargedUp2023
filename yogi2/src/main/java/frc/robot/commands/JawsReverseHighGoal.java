// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rapid React - 2022
// File: JawsReverseHighGoal.java
// Intent: Forms a command to drive the Jaws to the reverse high goal position.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Jaws;

public class JawsReverseHighGoal extends CommandBase {

  private Jaws jawsSubsystem;
  private boolean done;
  
  /**
   * Constructor for the jaws reverse high goal score
   * @param JawsSubsystem
   */
  public JawsReverseHighGoal(Jaws JawsSubsystem)
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.jawsSubsystem = JawsSubsystem;
    addRequirements(JawsSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
   done = false; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    if(jawsSubsystem.setJawsAngle(Constants.jawsReverseHighGoalPositionAngle, Constants.jawsPositionAngleTolerance))
    {
      jawsSubsystem.suspendJawMovement();
      done = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    jawsSubsystem.suspendJawMovement();
    done = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return done;
  }
}
