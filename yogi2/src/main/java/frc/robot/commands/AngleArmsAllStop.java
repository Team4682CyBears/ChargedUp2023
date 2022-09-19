// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rapid React - 2022
// File: AngleArmsAllStop.java
// Intent: Forms a manual command to have the AngleArms move in a manually selected direction based on motor speed directive.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AngleArms;
import frc.robot.*;

public class AngleArmsAllStop extends CommandBase {

  private AngleArms angleArmSubsystem;

  /**
   * The constructor 
   * @param AngleArmSubsystem - must hand in the enabled angle arms subsystem
   */
  public AngleArmsAllStop(AngleArms AngleArmSubsystem)
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.angleArmSubsystem = AngleArmSubsystem;
    addRequirements(AngleArmSubsystem);
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
    angleArmSubsystem.setAngleArmsSpeedManual(Constants.angleArmsManualMotorStopSpeed);
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    angleArmSubsystem.setAngleArmsSpeedManual(Constants.angleArmsManualMotorStopSpeed);
}

  // Always returns false - must be interrupted
  @Override
  public boolean isFinished()
  {
    return true;
  }
}
