// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rapid React - 2022
// File: DriveSystemToggleDirection.java
// Intent: Forms a command to flip the drive system.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveSystemToggleDirection extends CommandBase
{

  private DriveTrain driveTrain;
  
  /** 
  * Creates a new driveCommand. 
  * 
  * @param driveTrainSubsystem - the drive train subsystem
  */
  public DriveSystemToggleDirection(DriveTrain driveTrainSubsystem)
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrainSubsystem;
    addRequirements(driveTrain);
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
    driveTrain.toggleDriveDirection();
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
