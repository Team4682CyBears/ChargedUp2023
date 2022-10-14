// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rapid React - 2022
// File: Jaws.java
// Intent: Forms a subsystem that controls movements by the Jaws.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import frc.robot.Constants;
import frc.robot.common.MotorUtils;

public class Jaws extends SubsystemBase implements Sendable
{
    /* *********************************************************************
    CONSTANTS
    ************************************************************************/
    private static final double jawsMotorEncoderTicksPerDegree = Constants.CtreTalonFx500EncoderTicksPerRevolution / Constants.DegreesPerRevolution;
    private static final double jawsMotorToArmEffectiveGearRatio = 212; // according to nathan on 02/08/2022
    private static final double minmumTargetAngle = 0.0;
    private static final double maximumTargetAngle = 150.1;
    private static final double smartMotionAccelerationMultiplier = 0.4;

    // update this when folks are ready for it
    private static final double talonFxMotorSpeedReductionFactor = 0.25;

    /* *********************************************************************
    MEMBERS
    ************************************************************************/
    private final WPI_TalonFX rightMotor = new WPI_TalonFX(Constants.jawsMotorRightCanId);
    private final WPI_TalonFX leftMotor = new WPI_TalonFX(Constants.jawsMotorLeftCanId);

    private double motorReferencePosition = 0.0;
    private boolean motorsNeedInit = true;

    /* *********************************************************************
    CONSTRUCTORS
    ************************************************************************/

    /**
    * constructor for Jaws subsystem
    */
    public Jaws()
    {
      this.forceSensorReset();
      CommandScheduler.getInstance().registerSubsystem(this);
    }

    /* *********************************************************************
    PUBLIC METHODS
    ************************************************************************/
    /**
     * A method to update the sensors on this device
     */
    public void forceSensorReset()
    {
      this.initializeMotors();
      rightMotor.setSelectedSensorPosition(Constants.jawsReferencePositionMotorEncoderUnits);
      leftMotor.setSelectedSensorPosition(Constants.jawsReferencePositionMotorEncoderUnits);
    }

    /**
    * A method to obtain the Jaws current angle
    *
    * @return the current jaws angle
    */
    public double getJawsAngle()
    {
      return this.convertMotorEncoderPositionToJawsAngle(this.getAverageMotorEncoderPosition());
    }

    /**
     * A method to return the jaws subsystem to the reference position
     */
    public double getJawsReferencePositionAngle()
    {
      return this.convertMotorEncoderPositionToJawsAngle(Constants.jawsReferencePositionMotorEncoderUnits);
    }

    @Override
    public void initSendable(SendableBuilder builder)
    {
      builder.addDoubleProperty("JawsAverageMotorOutput", this::getAverageMotorOutput, null);
      builder.addDoubleProperty("JawsAverageMotorEncoderPosition", this::getAverageMotorEncoderPosition, null);
      builder.addDoubleProperty("JawsApproximageJawsAngle", this::getApproximateJawsAnglePosition, null);
    }

    /**
    * This method will be called once per scheduler run
    */
    @Override
    public void periodic()
    {
      // TODO - do we need anything here?
    }

    /**
    * This method helps decide the default command
    *
    * @param  value - The default command
    */
    @Override
    public void setDefaultCommand(Command myCommand)
    {
        super.setDefaultCommand(myCommand);
    }

    /**
    * a method exposed to callers to set the jaws angle
    *
    * @param  targetAngle - target angle of the jaws measured from front limit switch position
    * @param  toleranceInDegrees - the tolerance bounds in degrees that determine if the jaws have reached the proper setting
    * @return if the jaws have attained the target angle setpoint and are within the tolerance threashold
    */
    public boolean setJawsAngle(double targetAngleInDegrees, double toleranceInDegrees)
    {
      double trimmedAngle = MotorUtils.truncateValue(targetAngleInDegrees, Jaws.minmumTargetAngle, Jaws.maximumTargetAngle);

      // because of follower this will set both motors
      rightMotor.set(TalonFXControlMode.MotionMagic, convertJawsAngleToMotorEncoderPosition(trimmedAngle));
      double currentAngle = this.getJawsAngle();

      boolean result = (currentAngle >= targetAngleInDegrees - toleranceInDegrees && currentAngle <= targetAngleInDegrees + toleranceInDegrees); 
//      System.out.println("target angle = " + targetAngleInDegrees + " current angle = " + currentAngle + " tolerance degrees = " + toleranceInDegrees + " result = " + result);
      return result;
    }

    /**
    * a method to drive the jaws motors manually
    *
    * @param  jawsSpeed - the target jaws speed
    */
    public void setJawsSpeedManual(double jawsSpeed)
    {
      rightMotor.set(TalonFXControlMode.PercentOutput, MotorUtils.truncateValue(jawsSpeed, -1.0, 1.0));
    }

    /**
    * a method exposed to callers to hold/lock the current jaws angle
    */
    public void suspendJawMovement()
    {
      rightMotor.set(TalonFXControlMode.PercentOutput, 0.0);
    }
    
    /* *********************************************************************
    PRIVATE METHODS
    ************************************************************************/
    // a method to convert a jaws angle into the motor encoder position for the existing setup
    // note this includes adding the originating motor reference position (which is hopefully 0.0)
    private double convertJawsAngleToMotorEncoderPosition(double jawsAngle)
    {
      return jawsAngle * Jaws.jawsMotorEncoderTicksPerDegree * Jaws.jawsMotorToArmEffectiveGearRatio + motorReferencePosition;
    }

    // a method to convert the current motor encoder position for the existing setup into jaws angle position
    // note this includes subtracting the originating motor reference position (which is hopefully 0.0)
    private double convertMotorEncoderPositionToJawsAngle(double jawsMotorEncoderPosition)
    {
      return (jawsMotorEncoderPosition - motorReferencePosition) / Jaws.jawsMotorEncoderTicksPerDegree / Jaws.jawsMotorToArmEffectiveGearRatio;
    }

    private double getApproximateJawsAnglePosition()
    {
      return this.convertMotorEncoderPositionToJawsAngle(this.getAverageMotorEncoderPosition());
    }

    private double getAverageMotorEncoderPosition()
    {
      // since left is follower we will always uses right
      return rightMotor.getSelectedSensorPosition();
    }

    private double getAverageMotorOutput()
    {
      // since left is follower we will always uses right
      return rightMotor.getMotorOutputPercent();
    }

    // a method devoted to establishing proper startup of the jaws motors
    // this method sets all of the key settings that will help in motion magic
    private void initializeMotors()
    {
      if(motorsNeedInit)
      {
        double maxVelocity = Constants.talonMaximumRevolutionsPerMinute * Constants.CtreTalonFx500EncoderTicksPerRevolution / 10.0 * Jaws.talonFxMotorSpeedReductionFactor;
        double maxAcceleration = maxVelocity * smartMotionAccelerationMultiplier;

        // RIGHT MOTOR
        rightMotor.configFactoryDefault();
        rightMotor.setNeutralMode(NeutralMode.Brake);
        rightMotor.setInverted(Constants.jawsRightMotorDefaultDirection);
        rightMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
        rightMotor.setSensorPhase(false);
        rightMotor.configNeutralDeadband(0.001, Constants.kTimeoutMs);
        rightMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
        rightMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);
  
        /* Set the peak and nominal outputs */
        rightMotor.configNominalOutputForward(0.0, Constants.kTimeoutMs);
        rightMotor.configNominalOutputReverse(-0.0, Constants.kTimeoutMs);
        rightMotor.configPeakOutputForward(1.0, Constants.kTimeoutMs);
        rightMotor.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);
  
        rightMotor.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
        rightMotor.config_kF(Constants.kSlotIdx, Constants.kGains.kF, Constants.kTimeoutMs);
        rightMotor.config_kP(Constants.kSlotIdx, Constants.kGains.kP, Constants.kTimeoutMs);
        rightMotor.config_kI(Constants.kSlotIdx, Constants.kGains.kI, Constants.kTimeoutMs);
        rightMotor.config_kD(Constants.kSlotIdx, Constants.kGains.kD, Constants.kTimeoutMs);
        rightMotor.configMotionCruiseVelocity(maxVelocity, Constants.kTimeoutMs);
        rightMotor.configMotionAcceleration(maxAcceleration, Constants.kTimeoutMs);
  
        // current limit enabled | Limit(amp) | Trigger Threshold(amp) | Trigger
        rightMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 20, 25, 1.0));

        // LEFT MOTOR
        leftMotor.configFactoryDefault();
        leftMotor.setNeutralMode(NeutralMode.Brake);
        leftMotor.setInverted(Constants.jawsLeftMotorDefaultDirection);
        leftMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
        leftMotor.setSensorPhase(false);
        leftMotor.configNeutralDeadband(0.001, Constants.kTimeoutMs);
        leftMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
        leftMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);
  
        /* Set the peak and nominal outputs */
        leftMotor.configNominalOutputForward(0.0, Constants.kTimeoutMs);
        leftMotor.configNominalOutputReverse(-0.0, Constants.kTimeoutMs);
        leftMotor.configPeakOutputForward(1.0, Constants.kTimeoutMs);
        leftMotor.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);
  
        leftMotor.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
        leftMotor.config_kF(Constants.kSlotIdx, Constants.kGains.kF, Constants.kTimeoutMs);
        leftMotor.config_kP(Constants.kSlotIdx, Constants.kGains.kP, Constants.kTimeoutMs);
        leftMotor.config_kI(Constants.kSlotIdx, Constants.kGains.kI, Constants.kTimeoutMs);
        leftMotor.config_kD(Constants.kSlotIdx, Constants.kGains.kD, Constants.kTimeoutMs);
        leftMotor.configMotionCruiseVelocity(maxVelocity, Constants.kTimeoutMs);
        leftMotor.configMotionAcceleration(maxAcceleration, Constants.kTimeoutMs);
  
        // current limit enabled | Limit(amp) | Trigger Threshold(amp) | Trigger
        leftMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 20, 25, 1.0));

        // make left a follower of right
        leftMotor.follow(rightMotor);
        motorsNeedInit = false;
      }
   }
}