// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rapid React - 2022
// File: DriveTrain.java
// Intent: Forms model for the DriveTrain subsystem.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import java.io.Console;

import javax.lang.model.util.ElementScanner6;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.drive.*;
import edu.wpi.first.wpilibj.motorcontrol.*;

import frc.robot.Constants;
import frc.robot.common.MotorUtils;

public class DriveTrain extends SubsystemBase implements Sendable
{
  // update this when folks are ready for it
  private static final double neoMotorSpeedReductionFactor = 0.70;

  // four matched motors - two for each tank drive side
  private CANSparkMax leftFront = new CANSparkMax(Constants.driveMotorLeftFrontCanId, MotorType.kBrushless);
  private CANSparkMax leftRear = new CANSparkMax(Constants.driveMotorLeftRearCanId, MotorType.kBrushless);
  private CANSparkMax rightFront = new CANSparkMax(Constants.driveMotorRightFrontCanId, MotorType.kBrushless);
  private CANSparkMax rightRear = new CANSparkMax(Constants.driveMotorRightRearCanId, MotorType.kBrushless);

  private SparkMaxPIDController leftFrontPidController;
  private SparkMaxPIDController leftRearPidController;
  private SparkMaxPIDController rightFrontPidController;
  private SparkMaxPIDController rightRearPidController;

  private RelativeEncoder leftFrontEncoder;
  private RelativeEncoder leftRearEncoder;
  private RelativeEncoder rightFrontEncoder;
  private RelativeEncoder rightRearEncoder;
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

  private MotorControllerGroup left = new MotorControllerGroup(leftFront, leftRear);
  private MotorControllerGroup right = new MotorControllerGroup(rightFront, rightRear);
  private DifferentialDrive forwardDrive = new DifferentialDrive(left, right);
  private DifferentialDrive reverseDrive = new DifferentialDrive(right, left);

  private DifferentialDrive currentDrive = forwardDrive;
  private boolean isCurrentDriveForward = true;

  // TODO - get this info from Greyson / John
  private static final double robotTrackWidthInches = 21.7;
  private static final double halfRobotTrackWidthInches = robotTrackWidthInches / 2;

  private static final double wheelDiameterInches = 6.0;
  private static final double effectiveWheelMotorGearBoxRatio = (40.0 / 12.0) * (40.0 / 14.0);

  private static final int motorSettingTimeout = 0; //Constants.kTimeoutMs;

  private boolean smartMotionRunning = false;
  private boolean motorsInitalizedForSmartMotion = true;

  private double leftTargetEncoderTicks = 0;
  private double rightTargetEncoderTicks = 0;
  private double leftTargetEncoderTicksError = 0;
  private double rightTargetEncoderTicksError = 0;

  private double arcadeMotorPowerReductionFactor = 0.7;

  /**
  * No argument constructor for the DriveTrain subsystem.
  */
  public DriveTrain()
  {
    this.forceSensorReset();
    CommandScheduler.getInstance().registerSubsystem(this);
  }

  /**arcade
  * A method to take in x and y stick inputs and turn them into right and left motor speeds
  * considering arcade style driving
  *
  * @param  powerValue - power forward backward speed, range -1.0 to 1.0 where positive values are forward
  * @param  spinValue - spin componet, range -1.0 to 1.0 where positive values are forward
  */
  public void arcadeDrive(double powerValue, double spinValue)
  {
    currentDrive.arcadeDrive(powerValue * this.arcadeMotorPowerReductionFactor, spinValue);
  }

  /**
   * Set the manual power fraction that will be used in arcade
   * @param arcadePowerFraction
   */
  public void setArcadePowerFactor(double arcadePowerFraction)
  {
    this.arcadeMotorPowerReductionFactor = MotorUtils.truncateValue(arcadePowerFraction, 0.1, 1.0);
  }
  /**
   * Get the manual power fraction
   * @return
   */
  public double getArcadePowerFactor()
  {
    return this.arcadeMotorPowerReductionFactor;
  }

  /**
   * Get the manual power fraction
   * @return
   */
  public void toggleDriveDirection()
  {
    if(this.isCurrentDriveForward == true)
    {
      this.currentDrive = this.reverseDrive;
      this.isCurrentDriveForward = false;
    }
    else
    {
      this.currentDrive = this.forwardDrive;
      this.isCurrentDriveForward = true;
    }
  }

  /**
   * A method to update the sensors on this device
   */
  public void forceSensorReset()
  {
    this.initializeMotorsManual();
    leftFrontEncoder.setPosition(0.0);
    leftRearEncoder.setPosition(0.0);
    rightFrontEncoder.setPosition(0.0);
    rightRearEncoder.setPosition(0.0);
  }

  @Override
  public void initSendable(SendableBuilder builder)
  {
    builder.addDoubleProperty("DriveTrainAverageLeftMotorOutput", this::getAverageLeftMotorOutputSpeed, null);
    builder.addDoubleProperty("DriveTrainAverageRightMotorOutput", this::getAverageRightMotorOutputSpeed, null);
    builder.addDoubleProperty("DriveTrainAverageLeftMotorEncoderPosition", this::getAverageLeftMotorEncoderPosition, null);
    builder.addDoubleProperty("DriveTrainAverageRightMotorEncoderPosition", this::getAverageRightMotorEncoderPosition, null);
    builder.addDoubleProperty("DriveTrainApproximateLeftTravelDistanceInches", this::getApproximateLeftWheelDistance, null);
    builder.addDoubleProperty("DriveTrainApproximateRightTravelDistanceInches", this::getApproximateRightWheelDistance, null);
    builder.addStringProperty("DriveTrainMovementDescription", this::describeDriveTrainMovement, null);
  }
  
  /**
   * Determines if the motors are performing drive movement
   * @return true if an automated drive movement is currently happening
   */
  public boolean isCurrentlyPerformingDriveMovement()
  {
    if(smartMotionRunning)
    {
      // TODO - mike questions whether this logic is right for NEOs especially the 'error' componet
      System.out.println("Smart motion running.");
      leftFrontPidController.setReference(leftTargetEncoderTicks, ControlType.kSmartMotion);
      rightFrontPidController.setReference(rightTargetEncoderTicks, ControlType.kSmartMotion);
      double leftTicks = leftFrontEncoder.getPosition();
      double rightTicks = rightFrontEncoder.getPosition();
      double leftError = Math.abs(leftTargetEncoderTicks - leftTicks);
      double rightError = Math.abs(rightTargetEncoderTicks - rightTicks);
      if(leftError < leftTargetEncoderTicksError &&
         rightError < rightTargetEncoderTicksError && 
         leftTicks >= leftTargetEncoderTicks - leftTargetEncoderTicksError && leftTicks <= leftTargetEncoderTicks + leftTargetEncoderTicksError &&
         rightTicks >= rightTargetEncoderTicks - rightTargetEncoderTicksError && rightTicks <= rightTargetEncoderTicks + rightTargetEncoderTicksError)
      {
        leftFront.set(0.0);
        rightFront.set(0.0);
        smartMotionRunning = false;
        System.out.println("Stopping smart motion.");
      }
    }
    else
    {
      smartMotionRunning = false;
      System.out.println("forcing smart motion to false.");
    }
    return smartMotionRunning;
  }

  /**
   * stops performing drive movement if the motion magic is running
   */
  public void stopPerformingDriveMovement()
  {
    if(smartMotionRunning)
    {
      System.out.println("stopPerformingDriveMovement being called ... ");
      leftFront.set(0.0);
      rightFront.set(0.0);
      smartMotionRunning = false;
    }
  }

  /**
  * Move wheels such that centroid of robot follows a circular arc with a defined distance 
  *
  * @param  leftDistanceInches - robot centroid movement distance, range includes negative
  * @param  rotationDegrees - relative rotation from current position with forward robot heading is at 0.0 (must be between -360.0 and 360.0), positive values are clockwise rotation
  * @param  targetTimeSeconds - the target time to run this operation in seconds
  */
  public void performCircleArcDriveInches(double distanceInches, double rotationDegrees, double targetTimeSeconds)
  {
    this.initializeMotorsSmartMotion();
    double leftDistanceInches = distanceInches;
    double rightDistanceInches = distanceInches; 

    double radiusInches = this.radiusFromArcDistance(distanceInches, rotationDegrees);
    if(rotationDegrees > 0.5)
    {
      leftDistanceInches = this.distanceFromArcRadius(radiusInches + DriveTrain.halfRobotTrackWidthInches, rotationDegrees);
      rightDistanceInches = this.distanceFromArcRadius(radiusInches - DriveTrain.halfRobotTrackWidthInches, rotationDegrees);
    }
    else if (rotationDegrees < -0.5)
    {
      leftDistanceInches = this.distanceFromArcRadius(radiusInches - DriveTrain.halfRobotTrackWidthInches, rotationDegrees);
      rightDistanceInches = this.distanceFromArcRadius(radiusInches + DriveTrain.halfRobotTrackWidthInches, rotationDegrees);
    }
    else
    {
      leftDistanceInches = distanceInches;
      rightDistanceInches = distanceInches;         
    }

    // sets motors to input 
    this.moveWheelsDistance(leftDistanceInches, rightDistanceInches, targetTimeSeconds);
  }

  @Override
  public void periodic()
  {
  }

  @Override
  public void setDefaultCommand(Command myCommand)
  {
      // TODO Auto-generated method stub
      super.setDefaultCommand(myCommand);
  }

  private String describeDriveTrainMovement()
  {
    double leftMovement = this.getAverageLeftMotorOutputSpeed();
    double rightMovement = this.getAverageRightMotorOutputSpeed();
    if(leftMovement == 0.0 && rightMovement == 0.0)
    {
      return "Stopped";
    }
    else if(leftMovement > 0.0 && leftMovement == rightMovement)
    {
      return "Forward";
    }
    else if(leftMovement < 0.0 && leftMovement == rightMovement)
    {
      return "Reverse";
    }
    else if(leftMovement > 0.0 && rightMovement > 0.0)
    {
      if(leftMovement > rightMovement)
      {
        return "Forward Right Arc";
      }
      else
      {
        return "Forward Left Arc";
      }
    }
    else if(leftMovement < 0.0 && rightMovement < 0.0)
    {
      if(leftMovement < rightMovement)
      {
        return "Reverse Right Arc";
      }
      else
      {
        return "Reverse Left Arc";
      }
    }
    else if(leftMovement > 0.0 && leftMovement > rightMovement)
    {
      return "Right Spin";
    }
    else if(rightMovement > 0.0 && rightMovement > leftMovement)
    {
      return "Left Spin";
    }
    else
    {
      return "Undefined";
    }
  }

  private double radiusFromArcDistance(double distance, double rotationDegrees)
  {
    double absDistance = Math.abs(distance);
    double absDegrees = Math.abs(rotationDegrees);
    return (180.0 * absDistance) / (Math.PI * absDegrees);
  }

  private double distanceFromArcRadius(double radius, double rotationDegrees)
  {
    double radDistance = Math.abs(radius);
    double absDegrees = Math.abs(rotationDegrees);
    return (radDistance * Math.PI * absDegrees) / 180.0;
  }

  private void moveWheelsDistance(double leftWheelDistanceInches, double rightWheelDistanceInches, double targetTimeInSeconds)
  {
    double leftDeltaEncoderTicks = this.getEncoderUnitsFromTrackDistanceInInches(leftWheelDistanceInches);
    double rightDeltaEncoderTicks = this.getEncoderUnitsFromTrackDistanceInInches(rightWheelDistanceInches);   

    // set the targets
    leftTargetEncoderTicks = leftDeltaEncoderTicks + this.getAverageLeftMotorEncoderPosition();
    leftTargetEncoderTicksError = Math.abs(leftDeltaEncoderTicks) * 0.01;
    rightTargetEncoderTicks = rightDeltaEncoderTicks + this.getAverageRightMotorEncoderPosition();
    rightTargetEncoderTicksError = Math.abs(rightDeltaEncoderTicks) * 0.01;

    // move it with smart motion
    leftFrontPidController.setReference(leftTargetEncoderTicks, ControlType.kSmartMotion);
    rightFrontPidController.setReference(rightTargetEncoderTicks, ControlType.kSmartMotion);

    smartMotionRunning = true;
    System.out.println("END moveWheelsDistance!");
  }

  private double getEncoderUnitsFromTrackDistanceInInches(double wheelTrackDistanceInches)
  {
    return (wheelTrackDistanceInches / (Math.PI * DriveTrain.wheelDiameterInches)) * Constants.CtreTalonFx500EncoderTicksPerRevolution * DriveTrain.effectiveWheelMotorGearBoxRatio;
  }

  private double getTrackDistanceInInchesFromEncoderUnits(double encoderUnits)
  {
    return (encoderUnits / Constants.CtreTalonFx500EncoderTicksPerRevolution / DriveTrain.effectiveWheelMotorGearBoxRatio) * (Math.PI * DriveTrain.wheelDiameterInches);
  }

  private double getApproximateLeftWheelDistance()
  {
    return this.getTrackDistanceInInchesFromEncoderUnits(this.getAverageLeftMotorEncoderPosition());
  }

  private double getApproximateRightWheelDistance()
  {
    return this.getTrackDistanceInInchesFromEncoderUnits(this.getAverageRightMotorEncoderPosition());
  }

  private double getAverageLeftMotorEncoderPosition()
  {
    return leftFrontEncoder.getPosition();
  }

  private double getAverageLeftMotorOutputSpeed()
  {
    return leftFrontEncoder.getVelocity() / Constants.neoMaximumRevolutionsPerMinute;
  }

  private double getAverageRightMotorEncoderPosition()
  {
    return rightFrontEncoder.getPosition();
  }

  private double getAverageRightMotorOutputSpeed()
  {
    return rightFrontEncoder.getVelocity() / Constants.neoMaximumRevolutionsPerMinute;
  }

  private void initializeMotorsManual()
  {
    if(motorsInitalizedForSmartMotion == true)
    {
      this.initalizePidsAndEncoders(false);
      this.motorsInitalizedForSmartMotion = false;
    }
  }

  private void initializeMotorsSmartMotion()
  {
    if(motorsInitalizedForSmartMotion == false)
    {
      // PID coefficients
      kP = 5e-5; 
      kI = 1e-6;
      kD = 0; 
      kIz = 0; 
      kFF = 0.000156; 
      kMaxOutput = 1; 
      kMinOutput = -1;
      maxRPM = Constants.neoMaximumRevolutionsPerMinute;   
  
      // Smart Motion Coefficients
      maxVel = maxRPM * neoMotorSpeedReductionFactor; // rpm
      maxAcc = maxVel * 2; // 1/2 second to get up to full speed

      this.initalizePidsAndEncoders(true);
      this.motorsInitalizedForSmartMotion = true;
    }
  }

  private void initalizePidsAndEncoders(boolean doSmartMotionSetup)
  {
    leftFront.restoreFactoryDefaults();
    leftRear.restoreFactoryDefaults();
    rightFront.restoreFactoryDefaults();
    rightRear.restoreFactoryDefaults();

    leftFront.setIdleMode(IdleMode.kBrake);
    leftFrontPidController = leftFront.getPIDController();
    leftFrontEncoder = leftFront.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, Constants.countPerRevHallSensor);

    leftRear.setIdleMode(IdleMode.kBrake);
    leftRearPidController = leftFront.getPIDController();
    leftRearEncoder = leftFront.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, Constants.countPerRevHallSensor);

    rightFront.setIdleMode(IdleMode.kBrake);
    rightFrontPidController = leftFront.getPIDController();
    rightFrontEncoder = leftFront.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, Constants.countPerRevHallSensor);

    rightRear.setIdleMode(IdleMode.kBrake);
    rightRearPidController = leftFront.getPIDController();
    rightRearEncoder = leftFront.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, Constants.countPerRevHallSensor);

    if(doSmartMotionSetup)
    {
      this.initializeSinglePidEncoder(leftFrontPidController, leftFrontEncoder);
      this.initializeSinglePidEncoder(leftRearPidController, leftRearEncoder);
      this.initializeSinglePidEncoder(rightFrontPidController, rightFrontEncoder);
      this.initializeSinglePidEncoder(rightRearPidController, rightRearEncoder);
    }

    leftRear.follow(leftFront);
    rightRear.follow(rightFront);
  }

  private void initializeSinglePidEncoder(SparkMaxPIDController pid, RelativeEncoder encoder)
  {
    int smartMotionSlot = 0;
    encoder.setPositionConversionFactor((double)Constants.RevNeoEncoderTicksPerRevolution);

    // set PID coefficients
    pid.setP(kP);
    pid.setI(kI);
    pid.setD(kD);
    pid.setIZone(kIz);
    pid.setFF(kFF);
    pid.setOutputRange(kMinOutput, kMaxOutput);

    pid.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    pid.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    pid.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    pid.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
  }
}
 