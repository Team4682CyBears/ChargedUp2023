// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: TelescopingArm.java
// Intent: Forms a subsystem that controls TelescopingArm operations.
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
import frc.robot.common.*;

public class TelescopingArms extends SubsystemBase implements Sendable
{
    /* *********************************************************************
    CONSTANTS
    ************************************************************************/
    // expected to be < 1.0 due to encoder granularity being lower for Rev/Neo
    private static final double telescopingArmsMotorEncoderTicksPerDegree = Constants.RevNeoEncoderTicksPerRevolution / Constants.DegreesPerRevolution; 
    // Based on discussion with Simeon on 02/18/2022 - ~20:1
    private static final double telescopingArmsMotorToArmEffectiveGearRatio = (60.0/11.0) * (64.0/18.0);

    // important - this should be the maximum extension of the arms and it must also be the length of the wire on the spool - in inches!
    // TODO - must get this from Simeon/Carter soon-ish
    private static final double minimumOverageArmHeightInches = -0.1;
    private static final double minimumArmHeightInches = 0.0;
    private static final double maximumArmHeightInches = 31.0;
    private static final double maximumOverageArmHeightInches = 31.1;
    private static final double maximumHeightFromStoredPositionInches = maximumArmHeightInches - minimumArmHeightInches;
    // measurements of spool diameter in 4 discrete ranges
    // intended to be an average measurement of wire/chord on the spool when the spool is 'fractionaly wound'
    // for example when 0-25% of the wire is wound on the spool we need the diameter of the average winding to be placed in telescopingArmsSpoolDiameterInches0to25
    // TODO - must get these from Simeon/Carter soon-ish
    private static final double telescopingArmsSpoolDiameterInches0to25 = 1.50; 
    private static final double telescopingArmsSpoolDiameterInches26to50 = 1.51; 
    private static final double telescopingArmsSpoolDiameterInches51to75 = 1.52; 
    private static final double telescopingArmsSpoolDiameterInches76to100 = 1.53;
    
    // Based on discussion with Simeon, this is true
    private static final boolean spoolWindingIsPositiveSparkMaxNeoMotorOutput = true;

    // TODO change this to final speed when everyone is ready for it
    private static final double neoMotorSpeedReductionFactor = 1.0;

    /* *********************************************************************
    MEMBERS
    ************************************************************************/
    // two matched motors - one for each climber side
    private CANSparkMax leftMotor = new CANSparkMax(Constants.telescopingArmsMotorLeftCanId, MotorType.kBrushless);
    private SparkMaxPIDController leftPidController;
    private RelativeEncoder leftEncoder;
    private CANSparkMax rightMotor = new CANSparkMax(Constants.telescopingArmsMotorRightCanId, MotorType.kBrushless);
    private SparkMaxPIDController rightPidController;
    private RelativeEncoder rightEncoder;
    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

    private double motorReferencePosition = 0.0;

    private double motorEncoderTicksAt100 = this.convertTelescopingArmsHeightToMotorEncoderPosition(maximumHeightFromStoredPositionInches * 1.00);
    private double motorEncoderTicksAt75 = this.convertTelescopingArmsHeightToMotorEncoderPosition(maximumHeightFromStoredPositionInches * 0.75);
    private double motorEncoderTicksAt50 = this.convertTelescopingArmsHeightToMotorEncoderPosition(maximumHeightFromStoredPositionInches * 0.50);
    private double motorEncoderTicksAt25 = this.convertTelescopingArmsHeightToMotorEncoderPosition(maximumHeightFromStoredPositionInches * 0.25);

    private boolean motorsInitalizedForSmartMotion = false;
    /* *********************************************************************
    CONSTRUCTORS
    ************************************************************************/

    /**
    * constructor for TelescopingArms subsystem
    */
    public TelescopingArms()
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
      this.initializeMotorsSmartMotion();
      leftEncoder.setPosition(0.0);
      rightEncoder.setPosition(0.0);
    }

    @Override
    public void initSendable(SendableBuilder builder)
    {
      builder.addDoubleProperty("TelescopingArmsLeftMotorSpeed", this::getLeftMotorOutputSpeed, null);
      builder.addStringProperty("TelescopingArmsLeftArmMotionDescription", this::getLeftArmMotionDescription, null);
      builder.addDoubleProperty("TelescopingArmsLeftClimberHeightInInches", this::getLeftClimberHeightInInches, null);
      builder.addDoubleProperty("TelescopingArmsLeftEncoderPosition", this::getLeftMotorEncoderPosition, null);
      builder.addDoubleProperty("TelescopingArmsRightMotorSpeed", this::getRightMotorOutputSpeed, null);
      builder.addStringProperty("TelescopingArmsRightArmMotionDescription", this::getRightArmMotionDescription, null);     
      builder.addDoubleProperty("TelescopingArmsRightClimberHeightInInches", this::getRightClimberHeightInInches, null);
      builder.addDoubleProperty("TelescopingArmsRightEncoderPosition", this::getRightMotorEncoderPosition, null);
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
    * @param  myCommand - The default command
    */
    @Override
    public void setDefaultCommand(Command myCommand)
    {
        // TODO Auto-generated method stub
        super.setDefaultCommand(myCommand);
    }
  
    /**
    * a method exposed to callers to set the telescopingArms height
    *
    * @param  telescopingArmsHeightInInches - target telescopingArms height measured from stored position
    * @param  toleranceInInches - the tolerance bounds in inches that determine if the telescoping arms have reached the proper setting
    * @return if the Telescoping Arms have attained the target hight setpoint and are within the tolerance threashold
    */
    public boolean setTelescopingArmsHeight(double telescopingArmsHeightInInches, double toleranceInInches)
    {
      double trimmedHeight = MotorUtils.truncateValue(
        telescopingArmsHeightInInches,
        0.0, // 
        TelescopingArms.maximumHeightFromStoredPositionInches);

      // left
      double leftEncoderTicksTarget = this.convertTelescopingArmsHeightToMotorEncoderPosition(trimmedHeight);
      leftPidController.setReference(
        leftEncoderTicksTarget,
        ControlType.kSmartMotion);
      double leftHeight = this.getLeftClimberHeightInInches();
      boolean isLeftDone =  (leftHeight >= telescopingArmsHeightInInches - toleranceInInches  && leftHeight <= telescopingArmsHeightInInches + toleranceInInches) ||
        leftHeight >= TelescopingArms.minimumOverageArmHeightInches || leftHeight <= TelescopingArms.maximumOverageArmHeightInches;

      // right
      double rightEncoderTicksTarget = this.convertTelescopingArmsHeightToMotorEncoderPosition(trimmedHeight);
      rightPidController.setReference(
        rightEncoderTicksTarget,
        ControlType.kSmartMotion);
      double rightHeight = this.getRightClimberHeightInInches();
      boolean isRightDone =  (rightHeight >= telescopingArmsHeightInInches - toleranceInInches  && rightHeight <= telescopingArmsHeightInInches + toleranceInInches) ||
        rightHeight >= TelescopingArms.maximumOverageArmHeightInches || rightHeight <= TelescopingArms.minimumOverageArmHeightInches;

//      System.out.println("Target height = " + telescopingArmsHeightInInches + " Target Ticks = " + leftEncoderTicksTarget);

      return isLeftDone && isRightDone;
    }

    /**
    * a method to drive the telescopingArms motors manually
    *
    * @param  telescopingArmsSpeed - the target jaws speed
    */
    public void setTelescopingArmsSpeedManual(double telescopingArmsSpeed)
    {
      leftMotor.set(MotorUtils.truncateValue(telescopingArmsSpeed, -1.0, 1.0));
      rightMotor.set(MotorUtils.truncateValue(telescopingArmsSpeed, -1.0, 1.0));
    }

    /**
    * a method to drive the telescopingArms motors manually
    *
    * @param  telescopingArmsSpeed - the target jaws speed
    */
    public void setTelescopingArmLeftSpeedManual(double telescopingArmsSpeed)
    {
      leftMotor.set(MotorUtils.truncateValue(telescopingArmsSpeed, -1.0, 1.0));
    }

    /**
    * a method to drive the telescopingArms motors manually
    *
    * @param  telescopingArmsSpeed - the target jaws speed
    */
    public void setTelescopingArmRightSpeedManual(double telescopingArmsSpeed)
    {
      rightMotor.set(MotorUtils.truncateValue(telescopingArmsSpeed, -1.0, 1.0));
    }

    /* *********************************************************************
    PRIVATE METHODS
    ************************************************************************/
    // a method to convert a telescoping arms height into the motor encoder position for the existing setup
    // note this includes adding the originating motor reference position (which is hopefully 0.0)
    private double convertTelescopingArmsHeightToMotorEncoderPosition(double telescopingArmsHeightInInches)
    {
        double remainingFractionUnwound = telescopingArmsHeightInInches / TelescopingArms.maximumHeightFromStoredPositionInches;
        double encoderTargetUnits = this.motorReferencePosition;
        double travelingContributionFraction = 0.0;
        double travelingContributionHeight = 0.0;
        double travelingContributionRevolutions = 0.0;
        double travelingSpoolDiameter = 0.0;

        // build encoder units by looping through 4 different spool diameters
        for (int inx = 0; inx < 4; ++inx)
        {
            // determine the spool diameter based on each quarter of the spool
            switch (inx)
            {
                case (0):
                    travelingSpoolDiameter = TelescopingArms.telescopingArmsSpoolDiameterInches76to100;
                    break;
                case (1):
                    travelingSpoolDiameter = TelescopingArms.telescopingArmsSpoolDiameterInches51to75;
                    break;
                case (2):
                    travelingSpoolDiameter = TelescopingArms.telescopingArmsSpoolDiameterInches26to50;
                    break;
                case (3):
                    travelingSpoolDiameter = TelescopingArms.telescopingArmsSpoolDiameterInches0to25;
                    break;
                default:
                    // should we throw ... using max value will drive revolutions toward 0, so its ok for now
                    travelingSpoolDiameter = Double.MAX_VALUE;
                    break;
            }

            // build up each fractional portion of the spool using its differing diameters due to wire wind
            travelingContributionFraction = remainingFractionUnwound > 0.25 ? 0.25 : remainingFractionUnwound;
            remainingFractionUnwound -= travelingContributionFraction;
            if (travelingContributionFraction > 0.0)
            {
                travelingContributionHeight = travelingContributionFraction * TelescopingArms.maximumHeightFromStoredPositionInches;
                travelingContributionRevolutions = travelingContributionHeight / (Math.PI * travelingSpoolDiameter);
                encoderTargetUnits += travelingContributionRevolutions * Constants.DegreesPerRevolution * TelescopingArms.telescopingArmsMotorEncoderTicksPerDegree * TelescopingArms.telescopingArmsMotorToArmEffectiveGearRatio;
            }
        }

        return encoderTargetUnits;
    }

    // a method to convert the current motor encoder position for the existing setup into telescoping arms height 
    // note this includes subtracting the originating motor reference position (which is hopefully 0.0)
    private double convertMotorEncoderPositionToTelescopingArmsHeight(double telescopingArmsMotorEncoderPosition)
    {
        double remainingEncoderTicksUnwound = telescopingArmsMotorEncoderPosition - this.motorReferencePosition;
        double travelingEncoderMaximumContribution = 0.0;
        double travelingEncoderTicksUnwound = 0.0;
        double travelingSpoolDiameter = 0.0;
        double targetHeightInInches = 0.0;

        // build encoder units by looping through 4 different spool diameters
        for (int inx = 0; inx < 4; ++inx)
        {
            // determine the spool diameter based on each quarter of the spool
            switch (inx)
            {
                case (0):
                    travelingEncoderMaximumContribution = this.motorEncoderTicksAt100 - this.motorEncoderTicksAt75;
                    travelingSpoolDiameter = TelescopingArms.telescopingArmsSpoolDiameterInches76to100;
                    break;
                case (1):
                    travelingEncoderMaximumContribution = this.motorEncoderTicksAt75 - this.motorEncoderTicksAt50;
                    travelingSpoolDiameter = TelescopingArms.telescopingArmsSpoolDiameterInches51to75;
                    break;
                case (2):
                    travelingEncoderMaximumContribution = this.motorEncoderTicksAt50 - this.motorEncoderTicksAt25;
                    travelingSpoolDiameter = TelescopingArms.telescopingArmsSpoolDiameterInches26to50;
                    break;
                case (3):
                    travelingEncoderMaximumContribution = this.motorEncoderTicksAt25 - this.motorReferencePosition;
                    travelingSpoolDiameter = TelescopingArms.telescopingArmsSpoolDiameterInches0to25;
                    break;
                default:
                    // should we throw ... 
                    travelingEncoderMaximumContribution = 0;
                    travelingSpoolDiameter = 0;
                    break;
            }

            // build up each portion of the spool that was unwound
            travelingEncoderTicksUnwound = remainingEncoderTicksUnwound > travelingEncoderMaximumContribution ? travelingEncoderMaximumContribution : remainingEncoderTicksUnwound;
            remainingEncoderTicksUnwound -= travelingEncoderTicksUnwound;
            if (travelingEncoderTicksUnwound > 0.0)
            {
                targetHeightInInches +=
                  (travelingEncoderTicksUnwound / Constants.RevNeoEncoderTicksPerRevolution / TelescopingArms.telescopingArmsMotorToArmEffectiveGearRatio) *
                  (Math.PI * travelingSpoolDiameter);
            }
        }

        return targetHeightInInches;
    }

    private double getLeftClimberHeightInInches()
    {
      return this.convertMotorEncoderPositionToTelescopingArmsHeight(this.getLeftMotorEncoderPosition());
    }

    private double getLeftMotorEncoderPosition()
    {
      return leftEncoder.getPosition();
    }

    private double getLeftMotorOutputSpeed()
    {
      return leftMotor.getAppliedOutput();
    }

    private String getLeftArmMotionDescription()
    {
      return this.getArmMotionDescription(this.getLeftMotorOutputSpeed());
    }

    private double getRightClimberHeightInInches()
    {
      return this.convertMotorEncoderPositionToTelescopingArmsHeight(this.getRightMotorEncoderPosition());
    }

    private double getRightMotorEncoderPosition()
    {
      return rightEncoder.getPosition();   
    }

    private double getRightMotorOutputSpeed()
    {
      return rightMotor.getAppliedOutput();
    }

    private String getRightArmMotionDescription()
    {
      return this.getArmMotionDescription(this.getRightMotorOutputSpeed());
    }

    private String getArmMotionDescription(double motorAppliedOutput)
    {
      double actualMotorOutput = spoolWindingIsPositiveSparkMaxNeoMotorOutput ? motorAppliedOutput : -1.0 * motorAppliedOutput;
      if(actualMotorOutput == 0.0)
      {
        return "Stopped";
      }
      else if(actualMotorOutput > 0.0)
      {
        return "Retracting";
      }
      else if(actualMotorOutput < 0.0)
      {
        return "Extending";
      }
      else
      {
        return "Undefined";
      }
    }

    // a method devoted to establishing proper startup of the jaws motors
    // this method sets all of the key settings that will help in motion magic
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
        int smartMotionSlot = 0;
    
        // Smart Motion Coefficients
        maxVel = maxRPM * neoMotorSpeedReductionFactor; // rpm
        maxAcc = maxVel * 2; // 1/2 second to get up to full speed

        leftMotor.restoreFactoryDefaults();
        leftMotor.setIdleMode(IdleMode.kBrake);
        leftPidController = leftMotor.getPIDController();
        leftEncoder = leftMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, Constants.countPerRevHallSensor);
        leftEncoder.setPositionConversionFactor((double)Constants.RevNeoEncoderTicksPerRevolution);
   
        // set PID coefficients
        leftPidController.setP(kP);
        leftPidController.setI(kI);
        leftPidController.setD(kD);
        leftPidController.setIZone(kIz);
        leftPidController.setFF(kFF);
        leftPidController.setOutputRange(kMinOutput, kMaxOutput);
    
        leftPidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        leftPidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        leftPidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        leftPidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

        rightMotor.restoreFactoryDefaults();
        rightMotor.setIdleMode(IdleMode.kBrake);
        rightPidController = rightMotor.getPIDController();
        rightEncoder = rightMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, Constants.countPerRevHallSensor);
        rightEncoder.setPositionConversionFactor((double)Constants.RevNeoEncoderTicksPerRevolution);
   
        // set PID coefficients
        rightPidController.setP(kP);
        rightPidController.setI(kI);
        rightPidController.setD(kD);
        rightPidController.setIZone(kIz);
        rightPidController.setFF(kFF);
        rightPidController.setOutputRange(kMinOutput, kMaxOutput);
    
        rightPidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        rightPidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        rightPidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        rightPidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

        this.motorsInitalizedForSmartMotion = true;
      }
    }
}
