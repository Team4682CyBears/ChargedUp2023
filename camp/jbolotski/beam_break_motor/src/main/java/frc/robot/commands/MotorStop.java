// ************************************************************
// Bishop Blanchet Robotics
// Home of the CyBears
// FRC - Charged Up - 2023
// File: MotorStop.java
// Intent: Forms a command to drive a single test motor to the stopped state.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BeamBreakMotor;

public class MotorStop extends CommandBase
{
  private BeamBreakMotor beamBreakMotorSubsystem;
  private boolean done = false;

  public MotorStop(BeamBreakMotor motorSubsystem)
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.beamBreakMotorSubsystem = motorSubsystem;
    addRequirements(motorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
   done = false; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    beamBreakMotorSubsystem.stopMotor();
    done = true;
  }
 
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    done = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}