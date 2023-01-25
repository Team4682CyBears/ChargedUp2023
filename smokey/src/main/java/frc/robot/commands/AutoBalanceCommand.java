package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.common.QuaternionUtils;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.NavxSubsystem;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.lang.Math;


public class AutoBalanceCommand extends CommandBase{
  private double targetVelocity = 0.3;

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
    drivetrainsubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  { 
    // Not sure if it's OK to read the NavX on every time tick.  
    // If this costs too much, could read on every Nth time through the loop   
    //Translation2d angleOfSteepestAscent = QuaternionUtils.getAngleOfSteepestAscent(navxsubsystem.getQuaterion());
    Translation2d angleOfSteepestAscent = QuaternionUtils.getAngleOfSteepestAscent(navxsubsystem.getPitchRollYaw());
    Translation2d velocityVec = normalizeXYVelocities(angleOfSteepestAscent);
    // The angle of steepest ascent above is w.r.t. the gyro's frame of reference
    // therefore the X and Y velocities calculated are also w.r.t. the gyro's FoR.
    // So, we have to command the robot in gyro(field)-oriented drive. 
    // Even if the gyro is not properly oriented to the field, this will still 
    // drive the robot up the ramp.
    drivetrainsubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
      velocityVec.getX(), velocityVec.getY(), 0.0d, navxsubsystem.getGyroscopeRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    drivetrainsubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return navxsubsystem.isLevel();
  }

  /**
   * Scales XY values to the desired velocity 
   * @param vec - Translation2d vector to be scaled
   * @return - Translation2d scaled vector
   */
  private Translation2d normalizeXYVelocities(Translation2d vec)
  {
    double h = Math.sqrt(Math.pow(vec.getX(), 2) + Math.pow(vec.getY(), 2));
    return new Translation2d((vec.getX()/h) * targetVelocity, (vec.getY()/h) * targetVelocity);
  }

}
