// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: DrivetrainSubsystem.java
// Intent: Forms the prelminary code for drive train subsystem.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import java.util.*;

import static frc.robot.Constants.*;

import frc.robot.Constants;
import frc.robot.common.MotorUtils;
import frc.robot.control.SubsystemCollection;
import frc.robot.swerveHelpers.SwerveModuleHelper;
import frc.robot.swerveHelpers.SwerveModule;
import frc.robot.swerveHelpers.WcpModuleConfigurations;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
  public static final double MAX_VOLTAGE = 12.0 * Constants.DriveVoltageScalar;

  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight line.
   * From: https://docs.wcproducts.com/wcp-swervex/general-info/ratio-options
   * Gear ratio: 7.85:1. Free speed of 14.19 ft/s = 4.3251 m/s
   */
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.3251;
  public static final double MIN_VELOCITY_BOUNDARY_METERS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND * 0.14; // 0.14 a magic number based on testing
  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);
  public static final double MIN_ANGULAR_VELOCITY_BOUNDARY_RADIANS_PER_SECOND = MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.06; // 0.06 a magic number based on testing

  private static final int PositionHistoryWindowTimeMilliseconds = 5000;
  private static final int CommandSchedulerPeriodMilliseconds = 20;
  private static final int CommandSchedulerCyclesPerSecond = 1000/CommandSchedulerPeriodMilliseconds;
  private static final int PositionHistoryStorageSize = PositionHistoryWindowTimeMilliseconds/CommandSchedulerPeriodMilliseconds;
  private static final double RadiansPerRevolution = Math.PI * 2.0;

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
  private ArrayDeque<Pose2d> historicPositions = new ArrayDeque<Pose2d>(PositionHistoryStorageSize + 1);

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
  }

  /**
   * A method to get the current position of the robot
   * @return the current Pose2d position of the robot
   */
  public Pose2d getRobotPosition()
  {
    return currentPosition;
  }

  /**
   * A method to obtain the distance along the segments recently traveled
   * @param historicDurationMilliseconds - the total milliseconds to look back in time
   * @return A double of the distance traveled by the robot in meters over the time window requested
   */
  public double getRecentTotalDistanceInMeters(int historicDurationMilliseconds)
  {
    ArrayList<Double> distances = this.getRecentDistanceTraveled(historicDurationMilliseconds);
    double resultDistance = 0.0;
    for(int inx = 0; inx < distances.size(); ++inx)
    {
      resultDistance += distances.get(inx);
    }
    return resultDistance;
  }

  /**
   * A method to obtain the average velocity of the robot recently traveled
   * @param historicDurationMilliseconds - the total milliseconds to look back in time
   * @return A double of the velocity of the robot in meters per second over the time window requested
   */
  public double getRecentAverageVelocityInMetersPerSecond(int historicDurationMilliseconds)
  {
    ArrayList<Double> recentVelocities = this.getRecentVelocities(historicDurationMilliseconds);
    double sumOfVelocities = 0.0;
    int countOfDeltas = 0;
    for(; countOfDeltas < recentVelocities.size(); ++countOfDeltas)
    {
      sumOfVelocities += recentVelocities.get(countOfDeltas);
    }
    return sumOfVelocities/countOfDeltas;
  }

  /**
   * A method to obtain the average angular velocity of the robot recently experienced
   * @param historicDurationMilliseconds - the total milliseconds to look back in time
   * @return A double of the angular velocity of the robot in radians per second over the time window requested
   */
  public double getRecentAverageAngularVelocityInRadiansPerSecond(int historicDurationMilliseconds)
  {
    ArrayList<Double> recentAngularVelocities = this.getRecentAngularVelocities(historicDurationMilliseconds);

    // TODO - remove this next line when done testing
    // test the array to look for a measurement outlier
    MotorUtils.hasMeasurementDiscontinuity(recentAngularVelocities, true);

    double sumOfAngularVelocities = 0.0;
    int countOfDeltas = 0;
    for(; countOfDeltas < recentAngularVelocities.size(); ++countOfDeltas)
    {
      sumOfAngularVelocities += recentAngularVelocities.get(countOfDeltas);
    }
    return sumOfAngularVelocities/countOfDeltas;
  }

  /**
   * A method to obtain the acceleration experienced by the robot over the 
   * @param historicDurationMilliseconds - the total milliseconds to look back in time
   * @return A double of the average acceleration experienced by the robot in meters/second^2 for the time window requested
   */
  public double getRecentAverageAccelerationInMetersPerSecondSquared(int historicDurationMilliseconds)
  {
    ArrayList<Double> recentAccelerations = this.getRecentAccelerations(historicDurationMilliseconds);
    double sumOfAccelerations = 0.0;
    int countOfDeltas = 0;
    for(; countOfDeltas < recentAccelerations.size(); ++countOfDeltas)
    {
      sumOfAccelerations += recentAccelerations.get(countOfDeltas);
    }
    return sumOfAccelerations/countOfDeltas;
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
    // store the recalculated position
    this.storeUpdatedPosition();

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
        // TODO - we may want to throw in the future here as this is very important to our robot now
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

  /**
   * A method devoted to making sure the current position is added to the historic position listing
   * as well as maintaining the size of the list at a predefined maximum
   */
  private void storeUpdatedPosition()
  {
    this.historicPositions.add(currentPosition);
    while(this.historicPositions.size() > PositionHistoryStorageSize)
    {
      this.historicPositions.remove();
    }
  }

  /**
   * Method to get the recent translations
   * @param historicDurationMilliseconds - the historic time window in milliseconds of points to establish translations
   * @return the array of translations associated with the points at hand
   */
  private ArrayList<Translation2d> getRecentTranslations(int historicDurationMilliseconds)
  {
    // first get the list of translations between the Pos2d's stored
    int intendedCount = historicDurationMilliseconds/CommandSchedulerPeriodMilliseconds;
    int maxCount = (intendedCount>PositionHistoryStorageSize ? PositionHistoryStorageSize : intendedCount);
    int currentCount = 0;
    ArrayList<Translation2d> translations = new ArrayList<Translation2d>();
    Pose2d lastPosition = null;
    Pose2d currentPosition = null;
    for (Iterator<Pose2d> iter = this.historicPositions.iterator(); iter.hasNext() && currentCount < maxCount; ++currentCount ) {
      currentPosition = iter.next();
      // assume the first position has no translation (even though it likely actually have had one)
      if(lastPosition != null)
      {
        Transform2d nextTransform = new Transform2d(lastPosition, currentPosition);
        translations.add(nextTransform.getTranslation());
      }
      lastPosition = currentPosition;
    }
    return translations;
  }

  /**
   * Method to obtain the array of distances recently traveled
   * @param historicDurationMilliseconds - the historic time window in milliseconds of points to establish distances
   * @return array of distances traveled recently
   */
  private ArrayList<Double> getRecentDistanceTraveled(int historicDurationMilliseconds)
  {
    ArrayList<Translation2d> translations = this.getRecentTranslations(historicDurationMilliseconds);
    ArrayList<Double> resultDistances = new ArrayList<Double>();
    Translation2d previousTranslation = null;
    Translation2d currentTranslation = null;
    for(int inx = 0; inx < translations.size(); ++inx)
    {
      currentTranslation = translations.get(inx);
      if(previousTranslation != null)
      {
        resultDistances.add(previousTranslation.getDistance(currentTranslation));
      }
      previousTranslation = currentTranslation;
    }
    return resultDistances;
  }

  /**
   * Method to obtain the array of angular velocities recently traveled
   * @param historicDurationMilliseconds - the historic time window in milliseconds to obtain velocities from
   * @return array of rotation velocities traveled recently - in radians/s
   */
  private ArrayList<Double> getRecentAngularVelocities(int historicDurationMilliseconds)
  {
    ArrayList<Translation2d> translations = this.getRecentTranslations(historicDurationMilliseconds);
    ArrayList<Double> resultRotationVelocities = new ArrayList<Double>();
    Translation2d previousTranslation = null;
    Translation2d currentTranslation = null;
    for(int inx = 0; inx < translations.size(); ++inx)
    {
      currentTranslation = translations.get(inx);
      if(previousTranslation != null)
      {
        // with this conversion below we will be making an assumption:
        // that angular velocities will never exceed pi radians per 20 ms clock cycle
        // (e.g., that the robot is unable to sweep > 180 degrees in 20 ms - or said another way the robot can't physically spin > 25 spins per second)

        // to cleanly produce a result, we need to handle a couple of cases:
        // pose2d that exceed a full rotation - we will remove full rotations above 1 or below -1 -> to do this we will use the MathUtil.angleModulus() static method
        // pose2d that are negative - we will convert negative angle to its positive equivalent (2*pi + negative radians)
        double currentRadians = MathUtil.angleModulus(currentTranslation.getAngle().getRadians());
        double previousRadians = MathUtil.angleModulus(previousTranslation.getAngle().getRadians());
        double currentRadiansConverted = currentRadians;
        double previousRadiansConverted = previousRadians;

        if(currentRadians < 0.0)
        {
          currentRadiansConverted = currentRadians + RadiansPerRevolution;
        }

        if(previousRadians < 0.0)
        {
          previousRadiansConverted = previousRadians + RadiansPerRevolution;
        }

        // perform the assumption discussed above that delta measurements exceeding PI at 20 ms are physically not possible
        double minimizedAngularDistance = currentRadiansConverted - previousRadiansConverted;
        double absoluteMinimizedAngularDistance = Math.abs(minimizedAngularDistance);
        if(absoluteMinimizedAngularDistance > Math.PI)
        {
          if(minimizedAngularDistance >= 0.0)
          {
            minimizedAngularDistance = RadiansPerRevolution - absoluteMinimizedAngularDistance;
          }
          else
          {
            minimizedAngularDistance = (RadiansPerRevolution - absoluteMinimizedAngularDistance) * -1.0;
          }
        }

        // assumed cycle time and that the current distance is for a 20 ms movement
        resultRotationVelocities.add(minimizedAngularDistance * CommandSchedulerCyclesPerSecond); 
      }
      previousTranslation = currentTranslation;
    }
    return resultRotationVelocities;
  }

  /**
   * Method to obtain the array of velocities recently traveled
   * @param historicDurationMilliseconds - the historic time window in milliseconds to obtain velocities from
   * @return array of velocities traveled recently
   */
  private ArrayList<Double> getRecentVelocities(int historicDurationMilliseconds)
  {
    ArrayList<Double> recentDistances = this.getRecentDistanceTraveled(historicDurationMilliseconds);
    ArrayList<Double> resultVelocities = new ArrayList<Double>();
    for(int inx = 0; inx < recentDistances.size(); ++inx)
    {
      double currentDistance = recentDistances.get(inx);
      double currentVelocity = currentDistance * CommandSchedulerCyclesPerSecond; // assumed cycle time and that the current distance is for a 20 ms movement
      resultVelocities.add(currentVelocity);
    }
    return resultVelocities;
  }

  /**
   * Method to obtain the array of accelerations recently having occured
   * @param historicDurationMilliseconds - the historic time window in milliseconds to obtain accelerations from
   * @return array of accelerations traveled recently
   */
  private ArrayList<Double> getRecentAccelerations(int historicDurationMilliseconds)
  {
    ArrayList<Double> recentVelocities = this.getRecentVelocities(historicDurationMilliseconds);
    ArrayList<Double> resultAccelerations = new ArrayList<Double>();
    Double currentVelocity = null;
    Double previousVelocity = null;
    for(int inx = 0; inx < recentVelocities.size(); ++inx)
    {
      currentVelocity = recentVelocities.get(inx);
      if(previousVelocity != null)
      {
        double currentAcceleration = (currentVelocity - previousVelocity) * CommandSchedulerCyclesPerSecond; // assumed cycle time and that the current distance is for a 20 ms movement
        resultAccelerations.add(currentAcceleration);
      }
      previousVelocity = currentVelocity;
    }
    return resultAccelerations;
  }

}