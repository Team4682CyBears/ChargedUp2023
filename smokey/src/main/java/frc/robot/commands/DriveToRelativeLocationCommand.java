// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: ExampleArmCommand.java
// Intent: Drives to a given location relative to starting point
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.NavxSubsystem;
import frc.robot.Constants;

import java.lang.Math;

public class DriveToRelativeLocationCommand extends CommandBase{

  private final DrivetrainSubsystem drivetrain;
  private final NavxSubsystem navx;

  private final Transform2d relativeMovement;
  private Pose2d finalPose = new Pose2d();
  private final PIDController xPID = new PIDController(0.05, 0.05, 0.0);
  private final PIDController yPID = new PIDController(0.05, 0.05, 0.0);
  private final PIDController rotPID = new PIDController(0.05, 0.05, 0.0);

  /**
   * Creates a new DriveToRelativeLocationCommand.
   * @param drivetrainSubsystem
   * @param navxSubsystem
   * @param relativePose
   */
  public DriveToRelativeLocationCommand(
    DrivetrainSubsystem drivetrainSubsystem, 
    NavxSubsystem navxSubsystem,
    Transform2d relativeMovement) 
    {
    this.relativeMovement = relativeMovement;
    rotPID.enableContinuousInput(-1 * Math.PI, Math.PI);  

    this.drivetrain = drivetrainSubsystem;
    this.navx = navxSubsystem;
    // Do not need to add navx as a requirement, since it is read only
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // set final pose relative to current position + realtivePose
    this.finalPose = drivetrain.getRobotPosition().plus(relativeMovement);
    // set final pose as setpoints for the PID controllers
    xPID.setSetpoint(finalPose.getX());
    yPID.setSetpoint(finalPose.getY());
    rotPID.setSetpoint(finalPose.getRotation().getRadians());

    // setup PID tolerences
    xPID.setTolerance(Constants.positinonToleranceMeters);
    yPID.setTolerance(Constants.positinonToleranceMeters);
    rotPID.setTolerance(Constants.rotationToleranceRadians);
    // reset the PIDs
    xPID.reset();
    yPID.reset();
    rotPID.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getRobotPosition();
    drivetrain.drive(
      ChassisSpeeds.fromFieldRelativeSpeeds(
      clamp(xPID.calculate(currentPose.getX())),
      clamp(yPID.calculate(currentPose.getY())),
      clamp(rotPID.calculate(currentPose.getRotation().getRadians())),
      navx.getGyroscopeRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // we are done when all three PIDs are at their setpoints
    return xPID.atSetpoint() && yPID.atSetpoint() && rotPID.atSetpoint();
  }

  /**
   * Clamp velocities between [-1..1]
   * @param velocity
   * @return new velocity
   */
  public double clamp(double velocity)
  {
    // Velocities clamped between [-1..1]
    // TODO restore these to full speed (-1,1) once testing is complete
    return MathUtil.clamp(velocity, -0.25,0.25);
  }

}
