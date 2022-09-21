// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rapid React - 2022
// File: Shooter.java
// Intent: Forms a subsystem that controls Shooter operations.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.common.MotorUtils;

public class Shooter extends SubsystemBase implements Sendable
{

  // Shooter gearing - currently 1:1
  private static final double topShooterGearRatio = 1.0;
  private static final double bottomShooterGearRatio = 1.0;
  
  private CANSparkMax topMotor = new CANSparkMax(Constants.shooterMotorTopCanId, MotorType.kBrushless);
  private CANSparkMax bottomMotor = new CANSparkMax(Constants.shooterMotorBottomCanId, MotorType.kBrushless);
  private SparkMaxPIDController topPidController = topMotor.getPIDController();
  private SparkMaxPIDController bottomPidController = bottomMotor.getPIDController();
  private RelativeEncoder topEncoder = topMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, Constants.countPerRevHallSensor);
  private RelativeEncoder bottomEncoder = topMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, Constants.countPerRevHallSensor);
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  /**
   * Constructor
   */
  public Shooter()
  {
      // TODO - tune the 'gain' values below for the shooter setup on the actual 'yogi' robot 
      // ideally the values will differ for the top shooter motor and the bottom shooter motor (yet another change)
      // this mostly because the top shooter has more mass and angular momentum than the bottom one

      // the magic numbers here come from
      // https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Velocity%20Closed%20Loop%20Control/src/main/java/frc/robot/Robot.java 
      // PID coefficients
      kP = 6e-5; 
      kI = 0;
      kD = 0; 
      kIz = 0; 
      kFF = 0.000015; 
      kMaxOutput = 1.0; 
      kMinOutput = -1.0;
  
      // update the motor
      topMotor.restoreFactoryDefaults();
      topMotor.setIdleMode(IdleMode.kCoast);
      topMotor.setInverted(Constants.shooterTopMotorDefaultDirection);

      // set PID coefficients
      topPidController.setP(kP);
      topPidController.setI(kI);
      topPidController.setD(kD);
      topPidController.setIZone(kIz);
      topPidController.setFF(kFF);
      topPidController.setOutputRange(kMinOutput, kMaxOutput);

      // update the motor
      bottomMotor.restoreFactoryDefaults();
      bottomMotor.setIdleMode(IdleMode.kCoast);
      bottomMotor.setInverted(Constants.shooterBottomMotorDefaultDirection);
      
      // set PID coefficients
      bottomPidController.setP(kP);
      bottomPidController.setI(kI);
      bottomPidController.setD(kD);
      bottomPidController.setIZone(kIz);
      bottomPidController.setFF(kFF);
      bottomPidController.setOutputRange(kMinOutput, kMaxOutput);

      CommandScheduler.getInstance().registerSubsystem(this);
  }
  
  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
  }

  @Override
  public void initSendable(SendableBuilder builder)
  {
    builder.addDoubleProperty("ShooterBottomMotorSpeed", this::getBottomMotorSpeed, null);
    builder.addDoubleProperty("ShooterBottomRevolutionsPerMinute", this::getBottomShooterRevolutionsPerMinute, null);
    builder.addDoubleProperty("ShooterTopMotorSpeed", this::getTopMotorSpeed, null);
    builder.addDoubleProperty("ShooterTopRevolutionsPerMinute", this::getTopShooterRevolutionsPerMinute, null);
    builder.addStringProperty("ShooterIntakeDescription", this::getShooterIntakeDescription, null);
  }

  /**
   * A non-blocking call to spin up the shooter motors to a preset value for the intake
   * @return True when the motor is sufficiently up to speed, else false
   */
  public boolean intake()
  {
    this.setShooterVelocityTop(Constants.topMotorIntakeSpeedRpm);
    this.setShooterVelocityBottom(Constants.bottomMotorIntakeSpeedRpm);
    return this.isShooterVelocityUpToSpeedTop(Constants.topMotorIntakeSpeedRpm, Constants.defaultMotorSpeedToleranceRpm) &&
     this.isShooterVelocityUpToSpeedBottom(Constants.bottomMotorIntakeSpeedRpm, Constants.defaultMotorSpeedToleranceRpm);
  }

  /**
   * Determine if the the bottom shooter motor velocity 
   * @param targetRpm - the target in rpm
   * @param targetToleranceRpm - the tolerance in rpm 
   * @return true if the error in RPM is within the tolerance else false
   */
  public boolean isShooterVelocityUpToSpeedBottom(double targetRpm, double targetToleranceRpm)
  {
    return this.isMotorErrorWithinToleranceUsingVelocity(
      bottomEncoder.getVelocity(),
      targetRpm,
      targetToleranceRpm,
      Shooter.bottomShooterGearRatio);
  }

  /**
   * Determine if the the top shooter motor velocity 
   * @param targetRpm - the target in rpm
   * @param targetToleranceRpm - the tolerance in rpm 
   * @return true if the error in RPM is within the tolerance else false
   */
  public boolean isShooterVelocityUpToSpeedTop(double targetRpm, double targetToleranceRpm)
  {
    return this.isMotorErrorWithinToleranceUsingVelocity(
      topEncoder.getVelocity(),
      targetRpm,
      targetToleranceRpm,
      Shooter.topShooterGearRatio);
  }

  @Override
  public void setDefaultCommand(Command myCommand)
  {
      // TODO Auto-generated method stub
      super.setDefaultCommand(myCommand);
  }
  
  /**
   * A non-blocking call to spin up the shooter motors to a preset value for the shoot low
   * @return True when the motor is sufficiently up to speed, else false
   */
  public boolean shootLow()
  {
    this.setShooterVelocityTop(Constants.topMotorForwardLowGoalSpeedRpm);
    this.setShooterVelocityBottom(Constants.bottomMotorForwardLowGoalSpeedRpm);
    return this.isShooterVelocityUpToSpeedTop(Constants.topMotorForwardLowGoalSpeedRpm, Constants.defaultMotorSpeedToleranceRpm) &&
     this.isShooterVelocityUpToSpeedBottom(Constants.bottomMotorForwardLowGoalSpeedRpm, Constants.defaultMotorSpeedToleranceRpm);
  }

  /**
   * A non-blocking call to spin up the shooter motors to a preset value for the shoot high
   * @return True when the motor is sufficiently up to speed, else false
   */
  public boolean shootHigh()
  {
    this.setShooterVelocityTop(Constants.topMotorForwardHighGoalSpeedRpm);
    this.setShooterVelocityBottom(Constants.bottomMotorForwardHighGoalSpeedRpm);
    return this.isShooterVelocityUpToSpeedTop(Constants.topMotorForwardHighGoalSpeedRpm, Constants.defaultMotorSpeedToleranceRpm) &&
     this.isShooterVelocityUpToSpeedBottom(Constants.bottomMotorForwardHighGoalSpeedRpm, Constants.defaultMotorSpeedToleranceRpm);
  }

  /**
   * A non-blocking call to spin up the shooter motors to a preset value for the shoot high reverse
   * @return True when the motor is sufficiently up to speed, else false
   */
  public boolean shootHighReverse()
  {
    this.setShooterVelocityTop(Constants.topMotorReverseHighGoalSpeedRpm);
    this.setShooterVelocityBottom(Constants.bottomMotorReverseHighGoalSpeedRpm);
    return this.isShooterVelocityUpToSpeedTop(Constants.topMotorReverseHighGoalSpeedRpm, Constants.defaultMotorSpeedToleranceRpm) &&
     this.isShooterVelocityUpToSpeedBottom(Constants.bottomMotorReverseHighGoalSpeedRpm, Constants.defaultMotorSpeedToleranceRpm);
  }

  /**
   * Set both motor speeds to to the same value 
   * @param speed - Range -1.0 to 1.0 where negative values imply intake and positive imply shooting
   */
  public void setShooterManual(double speed)
  {
    double cleanSpeed = MotorUtils.truncateValue(speed, -1.0, 1.0);
    this.setShooterManualBottom(cleanSpeed);
    this.setShooterManualTop(cleanSpeed);
  }

  /**
   * Set the bottom shooter motor to a specific speed 
   * @param speed - Set the bottom motor speed, -1.0 to 1.0 where negative values imply intake and positive imply shooting
   */
  public void setShooterManualBottom(double speed)
  {
    bottomMotor.set(MotorUtils.truncateValue(speed, -1.0, 1.0));
  }

  /**
   * Set the top shooter motor to a specific speed 
   * @param speed - Set the top motor speed, -1.0 to 1.0 where negative values imply intake and positive imply shooting
   */
  public void setShooterManualTop(double speed)
  {
    topMotor.set(MotorUtils.truncateValue(speed, -1.0, 1.0));
  }

  /**
   * Set the bottom shooter motor to a specific velocity using the in-built PID controller
   * @param revolutionsPerMinute - the RPM that the bottom motor should spin
   */
  public void setShooterVelocityBottom(double revolutionsPerMinute)
  {
    double truncatedRpm = MotorUtils.truncateValue(
      revolutionsPerMinute,
      -1.0 * Constants.neoMaximumRevolutionsPerMinute,
      Constants.neoMaximumRevolutionsPerMinute);
    bottomPidController.setReference(truncatedRpm, CANSparkMax.ControlType.kVelocity);
  }

  /**
   * Set the top shooter motor to a specific velocity using the in-built PID controller
   * @param revolutionsPerMinute - the RPM that the top motor should spin
   */
  public void setShooterVelocityTop(double revolutionsPerMinute)
  {
    double truncatedRpm = MotorUtils.truncateValue(
      revolutionsPerMinute,
      -1.0 * Constants.neoMaximumRevolutionsPerMinute,
      Constants.neoMaximumRevolutionsPerMinute);
    topPidController.setReference(truncatedRpm, CANSparkMax.ControlType.kVelocity);
  }

  /**
   * Method to stop the shooter motors
   */
  public void stopShooter()
  {
    topMotor.set(0.0);
    bottomMotor.set(0.0);
  }

  private double convertShooterRpmToMotorRpm(double targetRpm, double targetGearRatio)
  {
    double convertedTargetRpm = 
      MotorUtils.truncateValue(
        targetRpm,
        Constants.neoMaximumRevolutionsPerMinute * -1.0,
        Constants.neoMaximumRevolutionsPerMinute) * targetGearRatio;
    return convertedTargetRpm;
  }

  /**
   * Gets the most recent bottom shooter RPM
   * @return the bottom shooter RPM based on the past 100 ms
   */
  private double getBottomShooterRevolutionsPerMinute()
  {
    return bottomEncoder.getVelocity() * Shooter.bottomShooterGearRatio;
  }

  /**
   * Gets the most recent top shooter RPM
   * @return the top shooter RPM based on the past 100 ms
   */
  private double getTopShooterRevolutionsPerMinute()
  {
    return topEncoder.getVelocity() * Shooter.topShooterGearRatio;
  }

  /**
   * Gets the bottom motor speed setting
   * @return the bottom motor controller output as decmil fraction
   */
  private double getBottomMotorSpeed()
  {
    return bottomMotor.getAppliedOutput();
  }

  /**
   * Gets the top motor speed setting
   * @return the top motor controller output as decmil fraction
   */
  private double getTopMotorSpeed()
  {
    return topMotor.getAppliedOutput();
  }

  private String getShooterIntakeDescription()
  {
    double topMotorSpeed = this.getTopMotorSpeed();
    double bottomMotorSpeed = this.getBottomMotorSpeed();
    if(topMotorSpeed == 0.0 && bottomMotorSpeed == 0.0)
    {
      return "Stopped";
    }
    else if (topMotorSpeed > 0.0 && bottomMotorSpeed > 0.0)
    {
      return "Shooting";
    }
    else if (topMotorSpeed < 0.0 && bottomMotorSpeed < 0.0)
    {
      return "Intaking";
    }
    else
    {
      return "Undefined";
    }
  }

  private boolean isMotorErrorWithinToleranceUsingVelocity(
    double actualMotorVelocityInRpm,
    double targetVelocityRpm,
    double targetToleranceRpm,
    double gearRatio)
  {
    double targetVelocityInMotorRpm = this.convertShooterRpmToMotorRpm(targetVelocityRpm, gearRatio);
    double targetVelocityToleranceInMotorRpm = this.convertShooterRpmToMotorRpm(Math.abs(targetToleranceRpm), gearRatio);

    boolean withinVelocityBounds = actualMotorVelocityInRpm > targetVelocityInMotorRpm - targetVelocityToleranceInMotorRpm &&
    actualMotorVelocityInRpm < targetVelocityInMotorRpm + targetVelocityToleranceInMotorRpm;
    /*
    System.out.println(
      "actualMotorVelocityInRpm=" + actualMotorVelocityInRpm +
      " targetVelocityInMotorRpm=" + targetVelocityInMotorRpm +
      " targetVelocityToleranceInMotorRpm=" + targetVelocityToleranceInMotorRpm +
      " withinVelocityBounds" + (withinVelocityBounds ? " YES" : " NO")
      );
    */
    return withinVelocityBounds;
  }
}