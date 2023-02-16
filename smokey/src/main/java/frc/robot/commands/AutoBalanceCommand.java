package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.common.VectorUtils;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.NavxSubsystem;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.lang.Math;

/**
 * Implements a command to perform a auto balancing routine. 
 */
public class AutoBalanceCommand extends CommandBase{
  private double targetVelocity = 0.3;

  NavxSubsystem navxsubsystem = null; 
  DrivetrainSubsystem drivetrainsubsystem = null;

  /**
   * Constructor for auto balance command.
   */
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
    // Not sure if it's too computationally expensive to read the NavX on every time tick.  
    // If this costs too much, could read on every Nth time through the loop   
    Translation2d angleOfSteepestAscent = VectorUtils.getAngleOfSteepestAscent(navxsubsystem.getEulerAngle());
    Translation2d velocityVec = normalizeXYVelocities(angleOfSteepestAscent);
    drivetrainsubsystem.drive(new ChassisSpeeds(velocityVec.getX(), velocityVec.getY(), 0.0d));
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
