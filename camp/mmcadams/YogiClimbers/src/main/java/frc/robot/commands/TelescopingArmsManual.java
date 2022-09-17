// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: TelescopingArmsManual.java
// Intent: Forms a command to drive the telescoping arms in the direction pressed.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TelescopingArms;

public class TelescopingArmsManual extends CommandBase
{
  private TelescopingArms telescopingArmSubsystem;
  private double targetMotorSpeed = 0.0;
  private boolean done = false;

  public TelescopingArmsManual(TelescopingArms telescopingArmSubsystem, double motorSpeed)
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.telescopingArmSubsystem = telescopingArmSubsystem;
    addRequirements(telescopingArmSubsystem);

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
      telescopingArmSubsystem.setTelescopingArmsSpeedManual(targetMotorSpeed);
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