// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rapid React - 2022
// File: TelescopingArmsSingleManual.java
// Intent: Forms a command to drive the telescoping arms in the direction pressed.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TelescopingArms;
import frc.robot.common.TelescopingArmSide;

public class TelescopingArmsSingleManual extends CommandBase
{
  private TelescopingArms telescopingArmSubsystem;
  private double targetMotorSpeed = 0.0;
  private TelescopingArmSide targetSide = TelescopingArmSide.Right;
  private boolean done = false;

  public TelescopingArmsSingleManual(
    TelescopingArms telescopingArmSubsystem,
    TelescopingArmSide side,
    double motorSpeed)
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.telescopingArmSubsystem = telescopingArmSubsystem;
    addRequirements(telescopingArmSubsystem);

    targetSide = side;
    targetMotorSpeed = motorSpeed;
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
      if(this.targetSide == TelescopingArmSide.Left)
      {
        telescopingArmSubsystem.setTelescopingArmLeftSpeedManual(targetMotorSpeed);
      }
      else if(this.targetSide == TelescopingArmSide.Right)
      {
        telescopingArmSubsystem.setTelescopingArmRightSpeedManual(targetMotorSpeed);
      }
      done = true;
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