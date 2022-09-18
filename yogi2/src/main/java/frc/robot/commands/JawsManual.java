// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rapid React - 2022
// File: JawsManual.java
// Intent: Forms a command to stop the Jaws.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Jaws;

public class JawsManual extends CommandBase
{
  private Jaws jawsSubsystem;
  private double targetSpeed = 0.0;
  private boolean done = false;
  
  /**
   * The ctor
   * @param JawsSubsystem
   * @param speed
   */
  public JawsManual(Jaws JawsSubsystem, double speed)
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.jawsSubsystem = JawsSubsystem;
    addRequirements(JawsSubsystem);

    targetSpeed = speed;
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
    jawsSubsystem.setJawsSpeedManual(targetSpeed);
    done = true;
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
    return done;
  }
}
