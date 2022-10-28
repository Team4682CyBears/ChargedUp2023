// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rapid React - 2022
// File: TelescopingArmAllStop.java
// Intent: Forms a command to drive the telescoping arms to stop.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.TelescopingArm;

public class TelescopingArmAllStop extends CommandBase
{
  private TelescopingArm telescopingArmSubsystem;
  private boolean done = false;

  public TelescopingArmAllStop(TelescopingArm TelescopingArmSubsystem)
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.telescopingArmSubsystem = TelescopingArmSubsystem;
    addRequirements(TelescopingArmSubsystem);
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
    telescopingArmSubsystem.setTelescopingArmSpeedManual(Constants.TelescopingArmStopSpeed);
    done = true;
//    System.out.println("Telescoping arm stop all");
  }
 
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return done;
  }
}