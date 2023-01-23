package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.common.QuaternionUtils;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.NavxSubsystem;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

import java.lang.Math;


public class AutoBalanceCommand extends CommandBase{
  private Timer timer = new Timer();
  private boolean done = false;
  private double xVelocity = 0.0;
  private double yVelocity = 0.0;
  private double rotVelocity = 0.0;
  // TODO hardcoded values eventually replaced when we implement PID controller drive command
  private double durationSecondsValue = 0.25;  
  private double velocityValue = 0.4;

  NavxSubsystem navxsubsystem = null; 
  DrivetrainSubsystem drivetrainsubsystem = null;


    public AutoBalanceCommand(DrivetrainSubsystem drivetrainSubsystem, NavxSubsystem navxsubsystem) {
      this.navxsubsystem = navxsubsystem;
      this.drivetrainsubsystem = drivetrainSubsystem;

      // do not need to add Navx as a requirement because it is read-only
      addRequirements(drivetrainSubsystem);
    }

      // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Translation2d angleOfSteepestAscent = QuaternionUtils.getAngleOfSteepestAscent(navxsubsystem.getQuaterion());
    double x = angleOfSteepestAscent.getX();
    double y = angleOfSteepestAscent.getY();
    double h = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    xVelocity = (x/h) * velocityValue;
    yVelocity = (y/h) * velocityValue;

    drivetrainsubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    timer.reset();
    timer.start();
    done = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    drivetrainsubsystem.drive(
      new ChassisSpeeds(xVelocity, yVelocity, rotVelocity));
    if (timer.hasElapsed(this.durationSecondsValue))
    {
      done = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    drivetrainsubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    if(interrupted)
    {
      done = true;      
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return done;
  }
}
