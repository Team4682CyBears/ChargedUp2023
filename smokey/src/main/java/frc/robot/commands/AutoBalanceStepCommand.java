package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.common.QuaternionUtils;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.NavxSubsystem;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

import java.lang.Math;


public class AutoBalanceStepCommand extends CommandBase{
  private Timer driveTimer = new Timer();
  private Timer waitTimer = new Timer();
  private boolean done = false;
  private double xVelocity = 0.0;
  private double yVelocity = 0.0;
  private double rotVelocity = 0.0;
  private double waitDurationSecondsValue = .5;
  // TODO hardcoded values eventually replaced when we implement PID controller drive command
  private double driveDurationSecondsValue = 0.25;  
  private double velocityValue = 0.4;

  NavxSubsystem navxsubsystem = null; 
  DrivetrainSubsystem drivetrainsubsystem = null;


    public AutoBalanceStepCommand(DrivetrainSubsystem drivetrainSubsystem, NavxSubsystem navxsubsystem) {
      this.navxsubsystem = navxsubsystem;
      this.drivetrainsubsystem = drivetrainSubsystem;

      // do not need to add Navx as a requirement because it is read-only
      addRequirements(drivetrainSubsystem);
    }

      // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Translation2d angleOfSteepestAscent = QuaternionUtils.getAngleOfSteepestAscent(navxsubsystem.getQuaterion());
    Translation2d angleOfSteepestAscent = QuaternionUtils.getAngleOfSteepestAscent(navxsubsystem.getPitchRollYaw());
    Translation2d velocityVec = normalizeXYVelocities(angleOfSteepestAscent);
    xVelocity = velocityVec.getX();
    yVelocity = velocityVec.getY();

    drivetrainsubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    driveTimer.reset();
    waitTimer.reset();
    driveTimer.start();
    done = false;

    System.out.println("I AM PRINTING HERE ******************************");
    System.out.println("This is the X velocity ------>" + xVelocity);
    System.out.println("This is the Y velocity ------>" + yVelocity);
    navxsubsystem.printState();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    // The angle of steepest ascent above is w.r.t. the gyro's frame of reference
    // therefore the X and Y velocities calculated are also w.r.t. the gyro's FoR.
    // So, we have to command the robot in gyro(field)-oriented drive. 
    // Even if the gyro is not properly oriented to the field, this will still 
    // drive the robot up the ramp.
    drivetrainsubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
      xVelocity, yVelocity, rotVelocity, navxsubsystem.getGyroscopeRotation()));
    // TODO wondering if these short bursts of driving make the ramp more wobbly
    // would smooth slow driving be better?
    if (driveTimer.hasElapsed(this.driveDurationSecondsValue))
    {
      drivetrainsubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
      waitTimer.start();
    }
    if (waitTimer.hasElapsed(this.waitDurationSecondsValue))
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

  /**
   * Scales XY values to the desired velocity 
   * @param vec - Translation2d vector to be scaled
   * @return - Translation2d scaled vector
   */
  private Translation2d normalizeXYVelocities(Translation2d vec)
  {
    double h = Math.sqrt(Math.pow(vec.getX(), 2) + Math.pow(vec.getY(), 2));
    return new Translation2d((vec.getX()/h) * velocityValue, (vec.getY()/h) * velocityValue);
  }

}
