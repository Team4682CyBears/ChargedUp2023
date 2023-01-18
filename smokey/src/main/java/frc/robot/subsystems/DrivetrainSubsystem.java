// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: DrivetrainSubsystem.java
// Intent: Forms the prelminary code for drive train subsystem.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import java.io.Console;

import static frc.robot.Constants.*;
import frc.robot.control.SubsystemCollection;
import frc.robot.swerveHelpers.SwerveModuleHelper;
import frc.robot.swerveHelpers.SwerveModule;
import frc.robot.swerveHelpers.WcpModuleConfigurations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {
  /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 12.0;

  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight line.
   * From: https://docs.wcproducts.com/wcp-swervex/general-info/ratio-options
   * Gear ratio: 7.85:1. Free speed of 14.19 ft/s = 4.3251 m/s
   */
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.3251;
  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);


  private final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
          // Front left
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );

  // These are our modules. We initialize them in the constructor.
  private final SwerveModule frontLeftModule;
  private final SwerveModule frontRightModule;
  private final SwerveModule backLeftModule;
  private final SwerveModule backRightModule;

  private SwerveDriveOdometry swerveOdometry = null;
  private SubsystemCollection currentCollection = null;
  private Pose2d currentPosition = new Pose2d();

  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  public DrivetrainSubsystem(SubsystemCollection collection) {
    currentCollection = collection;
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    frontLeftModule = SwerveModuleHelper.createFalcon500(
            // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
            tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0),
            // This can either be STANDARD or FAST depending on your gear configuration 
            WcpModuleConfigurations.SWERVEX,
            // This is the ID of the drive motor
            FRONT_LEFT_MODULE_DRIVE_MOTOR,
            // This is the ID of the steer motor
            FRONT_LEFT_MODULE_STEER_MOTOR,
            // This is the ID of the steer encoder
            FRONT_LEFT_MODULE_STEER_ENCODER,
            // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
            FRONT_LEFT_MODULE_STEER_OFFSET
    );

    // We will do the same for the other modules
    frontRightModule = SwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0),
            WcpModuleConfigurations.SWERVEX,
            FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            FRONT_RIGHT_MODULE_STEER_MOTOR,
            FRONT_RIGHT_MODULE_STEER_ENCODER,
            FRONT_RIGHT_MODULE_STEER_OFFSET
    );

    backLeftModule = SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(4, 0),
            WcpModuleConfigurations.SWERVEX,
            BACK_LEFT_MODULE_DRIVE_MOTOR,
            BACK_LEFT_MODULE_STEER_MOTOR,
            BACK_LEFT_MODULE_STEER_ENCODER,
            BACK_LEFT_MODULE_STEER_OFFSET
    );

    backRightModule = SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(6, 0),
            WcpModuleConfigurations.SWERVEX,
            BACK_RIGHT_MODULE_DRIVE_MOTOR,
            BACK_RIGHT_MODULE_STEER_MOTOR,
            BACK_RIGHT_MODULE_STEER_ENCODER,
            BACK_RIGHT_MODULE_STEER_OFFSET
    );

  }

  /**
   * Method avaliable so that callers can update the chassis speeds to induce changes in robot movement
   * @param updatedChassisSpeeds - the updated chassis speeds (x, y and rotation)
   */
  public void drive(ChassisSpeeds updatedChassisSpeeds) {
    chassisSpeeds = updatedChassisSpeeds;
    double omega = Math.abs(chassisSpeeds.omegaRadiansPerSecond);
    double theX = Math.abs(chassisSpeeds.vxMetersPerSecond);
    double theY = Math.abs(chassisSpeeds.vyMetersPerSecond);
    if(omega > 0.02 || theX > 0.02 || theY > 0.02 )
    {
      String strToPrint = "UPDATED chassis speed: omega == " + chassisSpeeds.omegaRadiansPerSecond +
      " x == " + chassisSpeeds.vxMetersPerSecond + 
      " y == " + chassisSpeeds.vyMetersPerSecond;
      System.out.println(strToPrint);
    }

  }

  /**
   * A method to get the current position of the robot
   * @return the current position of the current robot
   */
  public Pose2d getRobotPosition()
  {
    return currentPosition;
  }

  /**
   * A method to set the current position of the robot
   * @param updatedPosition - the new position of the robot
   */
  public void setRobotPosition(Pose2d updatedPosition)
  {
    // initialize the odometry goo
    currentPosition = updatedPosition;
    this.initializeSwerveOdometry(currentPosition);
  }

  @Override
  public void periodic() {

    // refresh the position of the robot
    this.refreshRobotPosition();

    // take the current 'requested' chassis speeds and ask the ask the swerve modules to attempt this
    // first we build a theoretical set of individual module states that the chassisSpeeds would corespond to
    SwerveModuleState[] states = swerveKinematics.toSwerveModuleStates(chassisSpeeds);
    // next we take the theoretical values and bring them down (if neecessary) to incorporate physical constraints (like motor maximum speeds)
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

    // now we take the four states and ask that the modules attempt to perform the wheel speed and direction built above
    frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
    frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
    backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
    backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
  }

  /**
   * Method used to initialize the Odometry for the robot 
   * @param currentRobotPosition - the centroid of the robot using the proper game coordinate system, see:
   * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#field-coordinate-system
   */
  private void initializeSwerveOdometry(Pose2d currentRobotPosition) {
    frontLeftModule.setDriveDistance(0.0);
    frontRightModule.setDriveDistance(0.0);
    backLeftModule.setDriveDistance(0.0);
    backRightModule.setDriveDistance(0.0);
    swerveOdometry = new SwerveDriveOdometry(
        swerveKinematics,
        this.getGyroAngle(),
        this.getSwerveModulePositions(),
        currentRobotPosition); 
  }

  /**
   * A method to attempt to more safely obtain the navx subsystem based gyro rotation (yaw)
   * @return the Rotation2d of the Gyroscope
   */
  private Rotation2d getGyroAngle() {
    if(currentCollection != null && currentCollection.getNavxSubsystem() != null){
        return currentCollection.getNavxSubsystem().getGyroscopeRotation();
    }
    else{
        System.out.println("!!!!!!!!!!!!!!!!!!!! NAVX IS MISSING. !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        return new Rotation2d();
    }
  }

  /**
   * Helper method to obtain the SwerveModulePostion array from an existing SwerveModules in this class
   * @return - a SwerveModulePosition array for the modules
   */
  private SwerveModulePosition [] getSwerveModulePositions() {
    return new SwerveModulePosition[] {
      this.getSwerveModulePositionFromModule(frontLeftModule),
      this.getSwerveModulePositionFromModule(frontRightModule),
      this.getSwerveModulePositionFromModule(backLeftModule),
      this.getSwerveModulePositionFromModule(backRightModule)
    };
  }

  /**
   * Helper method to obtain the SwerveModulePostion from an existing SwerveModule
   * @param module - the module to extract info from
   * @return - a SwerveModulePosition class that wraps the orentiation
   */
  private SwerveModulePosition getSwerveModulePositionFromModule(SwerveModule module) {
    return new SwerveModulePosition(
      module.getDriveDistance(),
      new Rotation2d(module.getSteerAngle()));
  }

  /**
   * A function intended to be called from perodic to update the robots centroid position on the field.
   */
  private void refreshRobotPosition() {
    // Update the position of the robot
    Rotation2d angle = this.getGyroAngle();
    SwerveModulePosition[] positions = this.getSwerveModulePositions();
    currentPosition = swerveOdometry.update(
      angle,
      positions);

    SmartDashboard.putNumber("RobotFieldHeadingDegrees", currentPosition.getRotation().getDegrees());
    SmartDashboard.putNumber("RobotFieldXCoordinateMeters", currentPosition.getX());
    SmartDashboard.putNumber("RobotFieldYCoordinateMeters", currentPosition.getY());
    SmartDashboard.putNumber("FrontLeftAngleDegrees", positions[0].angle.getDegrees());
    SmartDashboard.putNumber("FrontLeftDistanceMeters", positions[0].distanceMeters);
    SmartDashboard.putNumber("FrontRightAngleDegrees", positions[1].angle.getDegrees());
    SmartDashboard.putNumber("FrontRightDistanceMeters", positions[1].distanceMeters);
    SmartDashboard.putNumber("BackLeftAngleDegrees", positions[2].angle.getDegrees());
    SmartDashboard.putNumber("BackLeftDistanceMeters", positions[2].distanceMeters);
    SmartDashboard.putNumber("BackRightAngleDegrees", positions[3].angle.getDegrees());
    SmartDashboard.putNumber("BackRightDistanceMeters", positions[3].distanceMeters);
  }
}