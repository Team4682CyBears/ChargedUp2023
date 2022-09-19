// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rapid React - 2022
// File: AngleArm.java
// Intent: Forms a subsystem that controls the AngleArm operations.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants;
import frc.robot.InstalledHardware;
import frc.robot.common.MotorUtils;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class AngleArms extends SubsystemBase implements Sendable
{
    // TODO change this to final speed when everyone is ready for it
    private static final double neoMotorSpeedReductionFactor = 0.5;

    private CANSparkMax rightMotor = new CANSparkMax(Constants.angleArmsMotorCanId, MotorType.kBrushless);
    private SparkMaxPIDController rightPidController;
    private RelativeEncoder rightEncoder;
    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
    private boolean motorsInitalizedForSmartMotion = false;

    /**
     * The constructor for AngleArms
     */
    public AngleArms()
    {
        this.forceSensorReset();
        CommandScheduler.getInstance().registerSubsystem(this);
    }

    /**
     * A method to update the sensors on this device
     */
    public void forceSensorReset()
    {
      this.initializeMotorsSmartMotion();
      rightEncoder.setPosition(0.0);
    }

    /**
    * A method to obtain the Jaws current angle
    *
    * @return the current jaws angle
    */
    public double getCurrentAngleArmsAngle()
    {
        return this.convertMotorEncoderPositionToAngleArmsAngle(this.getAverageMotorEncoderPosition());
    }

    /**
     * A method to return the jaws subsystem to the reference position
     */
    public double getAngleArmsReferencePositionAngle()
    {
        return Constants.angleArmsReferencePositionAngle;
    }

    @Override
    public void initSendable(SendableBuilder builder)
    {
      builder.addDoubleProperty("AngleArmsAverageMotorOutput", this::getAverageMotorOutput, null);
      builder.addDoubleProperty("AngleArmsAverageMotorEncoderPosition", this::getAverageMotorEncoderPosition, null);
      builder.addDoubleProperty("AngleArmsCurrentAngleArmsAngle", this::getCurrentAngleArmsAngle, null);
    }

    @Override
    public void setDefaultCommand(Command myCommand)
    {
        super.setDefaultCommand(myCommand);
    }

    /**
    * a method exposed to callers to set the angle arms angle
    *
    * @param  targetAngle - target angle of the angle arms measured from front limit switch position
    * @param  toleranceInDegrees - the tolerance bounds in degrees that determine if the jaws have reached the proper setting
    * @return if the angle arms have attained the target angle setpoint and are within the tolerance threashold
    */
    public boolean setAngleArmsAngle(double targetAngleInDegrees, double toleranceInDegrees)
    {
        double trimmedAngle = MotorUtils.truncateValue(targetAngleInDegrees, Constants.angleArmsMinimumPositionAngle, Constants.angleArmsMaximumPositionAngle);

        // because of follower this will set both motors
        rightPidController.setReference(
          this.convertAngleArmsAngleToMotorEncoderPosition(trimmedAngle),
          ControlType.kSmartMotion);
        double currentAngle = this.getCurrentAngleArmsAngle();

        boolean result = (currentAngle >= targetAngleInDegrees - toleranceInDegrees && currentAngle <= targetAngleInDegrees + toleranceInDegrees); 
//      System.out.println("target angle = " + targetAngleInDegrees + " current angle = " + currentAngle + " tolerance degrees = " + toleranceInDegrees + " result = " + result);
        return result;
    }

    /**
    * a method to drive the jaws motors manually
    *
    * @param  angleArmsSpeed - the target jaws speed
    */
    public void setAngleArmsSpeedManual(double angleArmsSpeed)
    {
        rightMotor.set(MotorUtils.truncateValue(angleArmsSpeed, -1.0, 1.0));
    }

    /* *********************************************************************
    PRIVATE METHODS
    ************************************************************************/
    // a method to convert a angle arms angle into the motor encoder position for the existing setup
    private double convertAngleArmsAngleToMotorEncoderPosition(double angleArmsAngle)
    {
      // TODO - fix this to properly approximate what the mechanism that will be used on the robot
      // for instance if there is a wire and a spool this might be more of a trig type function to approximate
      // straight line movement vs on an arc - not sure what mechanical will look like
      return angleArmsAngle * Constants.CtreTalonFx500EncoderTicksPerRevolution / Constants.DegreesPerRevolution * Constants.angleArmsMotorEffectiveGearRatio;
    }

    // a method to convert the current motor encoder position for the existing setup into angle arms angle position
    private double convertMotorEncoderPositionToAngleArmsAngle(double angleArmsMotorEncoderPosition)
    {
      // TODO - fix this to properly approximate what the mechanism that will be used on the robot
      // for instance if there is a wire and a spool this might be more of a trig type function to approximate
      // straight line movement vs on an arc - not sure what mechanical will look like
      return angleArmsMotorEncoderPosition / Constants.CtreTalonFx500EncoderTicksPerRevolution * Constants.DegreesPerRevolution / Constants.angleArmsMotorEffectiveGearRatio;
    }

    private double getAverageMotorEncoderPosition()
    {
      return rightEncoder.getPosition();
    }

    private double getAverageMotorOutput()
    {
      return rightMotor.getAppliedOutput();
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
        maxAcc = maxVel; // 1 second to get up to full speed

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
