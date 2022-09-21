// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rapid React - 2022
// File: ShooterManual.java
// Intent: Forms a command to make the shooter spin speeds manual values.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants;
import frc.robot.Constants;
import frc.robot.subsystems.BallStorage;

public class ShooterManual extends CommandBase
{
  private Shooter shooterSubsystem;
  private double topRpm = 0.0;
  private double bottomRpm = 0.0;

  /**
  * The two argument constructor for the shooter intake
  *
  * @param ShooterSubsystem - The shooter subsystem in this robot
  * @param BallStorageSubsystem - The ball storage subsystem in this robot
  */
  public ShooterManual(Shooter ShooterSubsystem, double topMotorRpm, double bottomMotorRpm)
  {
    this.shooterSubsystem = ShooterSubsystem;
    addRequirements(ShooterSubsystem);
    topRpm = topMotorRpm;
    bottomRpm = bottomMotorRpm;
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
      shooterSubsystem.setShooterVelocityBottom(this.bottomRpm);
      shooterSubsystem.setShooterVelocityTop(this.topRpm);
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
