// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: TelescopingArmExtendVariable.java
// Intent: Forms a command to drive the telescoping arms to their extended position.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.TelescopingArms;


public class TelescopingArmExtendVariable extends CommandBase
{
  private TelescopingArms telescopingArmSubsystem;
  private boolean done = false;
  private double targetHeight;

  public TelescopingArmExtendVariable(TelescopingArms telescopingArmSubsystem, double extendedHeightInInches)
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.telescopingArmSubsystem = telescopingArmSubsystem;
    addRequirements(telescopingArmSubsystem);

    targetHeight = extendedHeightInInches;
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
    if(telescopingArmSubsystem.setTelescopingArmsHeight(targetHeight, Constants.telescopingArmsToleranceInches))
    {
      done = true;
    }
//    System.out.println("Telescoping arms height (extend variable): " + telescopingArmSubsystem.getTelescopingArmsHeight() + " encoder pos = " + telescopingArmSubsystem.getTelescopingArmsAverageEncoderPosition());
  }
 
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    telescopingArmSubsystem.setTelescopingArmsSpeedManual(Constants.telescopingArmsStopSpeed);
    done = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return done;
  }
}