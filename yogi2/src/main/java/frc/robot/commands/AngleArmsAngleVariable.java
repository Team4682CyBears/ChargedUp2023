// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rapid React - 2022
// File: AngleArmsAngleVariable.java
// Intent: Forms a command to have the AngleArm angle to be set to a variable angle within its range of motion.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.AngleArms;
import frc.robot.subsystems.Jaws;

public class AngleArmsAngleVariable extends CommandBase {

  private AngleArms angleArmSubsystem;
  private double angleArmsTargetAngleSetpoint;
  private boolean done;

  /**
   * The constructor 
   * @param AngleArmSubsystem - must hand in the enabled angle arms subsystem
   * @param JawsSubsystem - must hand in the enabled jaws subsystem
   */
  public AngleArmsAngleVariable(AngleArms AngleArmSubsystem, double angleArmsTargetAngle)
  {
      // Use addRequirements() here to declare subsystem dependencies.
      this.angleArmSubsystem = AngleArmSubsystem;
      addRequirements(AngleArmSubsystem);
      angleArmsTargetAngleSetpoint = angleArmsTargetAngle;
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
    if(done == false)
    {
      done = angleArmSubsystem.setAngleArmsAngle(this.angleArmsTargetAngleSetpoint, Constants.angleArmsSetpointTolerance);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    done = true;
    angleArmSubsystem.setAngleArmsSpeedManual(Constants.angleArmsManualMotorStopSpeed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return done;
  }
}
