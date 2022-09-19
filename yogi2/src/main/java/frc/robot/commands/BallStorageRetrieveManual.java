// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rapid React - 2022
// File: BallStorageRetrieveManual.java
// Intent: Forms a manual command to have the BallStorage to retrieve a ball.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.ManualInputInterfaces;
import frc.robot.subsystems.BallStorage;

public class BallStorageRetrieveManual extends CommandBase {

  private BallStorage ballStorageSubsystem;
  private boolean done = false;

  /**
   * The constructor 
   * @param BallStorageSubsystem - must hand in the enabled ball storage subsystem
   */
  public BallStorageRetrieveManual(BallStorage BallStorageSubsystem)
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.ballStorageSubsystem = BallStorageSubsystem;
    addRequirements(BallStorageSubsystem);
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
    ballStorageSubsystem.retrieveBallManual();
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
