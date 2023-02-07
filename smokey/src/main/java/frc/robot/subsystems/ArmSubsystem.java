// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: ArmSubsystem.java
// Intent: Forms a stub for the prelminary named subsystem above.
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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.common.*;
import java.util.*;

public class ArmSubsystem extends SubsystemBase
{
    /* *********************************************************************
    CONSTANTS
    ************************************************************************/
    // expected to be < 1.0 due to encoder granularity being lower for Rev/Neo
    private static final double telescopingArmsMotorEncoderTicksPerDegree = Constants.RevNeoEncoderTicksPerRevolution / Constants.DegreesPerRevolution;
    // TODO - find out gear ratio from build side / Owen
    private static final double verticalArmMovementInMetersPerMotorRotation = (0.2/12.0) * (1.0/50.0); // (linear distance meters / number of teeth moved in one rotation) * (gear ratio - number of final gear rotations / number of motor rotations)
    private static final double horizontalArmMovementInMetersPerMotorRotation = verticalArmMovementInMetersPerMotorRotation;

    // important - this should be the maximum extension of the arms - in meters
    // TODO - must get this from build side / Owen
    private static final double minimumVerticalArmExtensionMeters = 0.0;
    private static final double maximumVerticalArmExtensionMeters = 0.25;
    private static final double minimumHorizontalArmExtensionMeters = 0.0;
    private static final double maximumHorizontalArmExtensionMeters = 0.40;

    // TODO - use something less than 1.0 for testing
    private static final double neoMotorSpeedReductionFactor = 0.3;
    private static final double neoMotorFindZeroSpeed = 0.3;

    /* *********************************************************************
    MEMBERS
    ************************************************************************/
    // two matched motors - one for each climber side
    private CANSparkMax verticalMotor = new CANSparkMax(Constants.VerticalArmDriveMotorCanId, MotorType.kBrushless);
    private SparkMaxPIDController verticalPidController;
    private RelativeEncoder verticalEncoder;
    private CANSparkMax horizontalMotor = new CANSparkMax(Constants.HorizontalArmDriveMotorCanId, MotorType.kBrushless);
    private SparkMaxPIDController horizontalPidController;
    private RelativeEncoder horizontalEncoder;
    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
    private double motorReferencePosition = 0.0;
    private boolean motorsInitalizedForSmartMotion = false;

    private DigitalInput verticalArmMageneticSensor = new DigitalInput(Constants.VirticalArmMagneticSensor);
    private DigitalInput horizontalArmMageneticSensor = new DigitalInput(Constants.HorizontalArmMagneticSensor);

    private boolean isHorizontalMotorRetractDirectionPositive = false;
    private boolean isVerticalMotorRetractDirectionPositive = false;

    private boolean inSpeedMode = true;
    private double requestedHorizontalMotorSpeed = 0.0;
    private double requestedVerticalMotorSpeed = 0.0;
    private double requestedHorizontalArmExtension = 0.0;
    private double requestedVerticalArmExtension = 0.0;

    /* *********************************************************************
    CONSTRUCTORS
    ************************************************************************/

    /**
    * constructor for TelescopingArms subsystem
    */
    public ArmSubsystem() {
      CommandScheduler.getInstance().registerSubsystem(this);
    }

    /* *********************************************************************
    PUBLIC METHODS
    ************************************************************************/
    /**
     * A method to set requested the arms motor speeds
     * @param horizontalArmSpeed the speed to run the horizontal arm motor at
     * @param verticalArmSpeed the speed to run the vertical arm motor at
     */
    public void setArmSpeeds(double horizontalArmSpeed, double verticalArmSpeed){
      this.inSpeedMode = true;
      this.requestedHorizontalMotorSpeed = MotorUtils.truncateValue(horizontalArmSpeed, -1.0, 1.0);
      this.requestedVerticalMotorSpeed = MotorUtils.truncateValue(verticalArmSpeed, -1.0, 1.0);
    }

    /**
     * A method to set requested the arms motor extension distance
     * @param horizontalArmExtension the distance to extend the vertical arm to
     * @param verticalArmExtension the distance to extend the vertical arm to
     */
    public void setArmExtensions(double horizontalArmExtension, double verticalArmExtension){
      this.inSpeedMode = false;
      this.requestedHorizontalArmExtension = MotorUtils.truncateValue(horizontalArmExtension, minimumHorizontalArmExtensionMeters, maximumHorizontalArmExtensionMeters);
      this.requestedVerticalArmExtension = MotorUtils.truncateValue(verticalArmExtension, minimumVerticalArmExtensionMeters, maximumVerticalArmExtensionMeters);
    }
    
    @Override
    public void periodic() {

      // confirm that the smart motion is setup - no-op after it is setup first time
      this.initializeMotorsSmartMotion();
      this.refreshArmPosition();

      boolean allowHorizontalMovementDespiteStop = false;
      if(this.isHorizontalMotorRetractDirectionPositive){
        allowHorizontalMovementDespiteStop = 
          (this.inSpeedMode == true) ? 
          (this.requestedHorizontalMotorSpeed < 0.0) :
          (this.requestedHorizontalArmExtension > minimumHorizontalArmExtensionMeters && this.requestedVerticalArmExtension < maximumHorizontalArmExtensionMeters);
      }            
      else{
        allowHorizontalMovementDespiteStop =
          (this.inSpeedMode == true) ?
          (this.requestedHorizontalMotorSpeed > 0.0) :
          (this.requestedHorizontalArmExtension > minimumHorizontalArmExtensionMeters && this.requestedVerticalArmExtension < maximumHorizontalArmExtensionMeters);
      }

      boolean allowVerticalMovementDespiteStop = false;
      if(this.isVerticalMotorRetractDirectionPositive){
        allowVerticalMovementDespiteStop =
          (this.inSpeedMode == true) ?
          (this.requestedVerticalMotorSpeed < 0.0) :
          (this.requestedVerticalArmExtension > minimumVerticalArmExtensionMeters && this.requestedVerticalArmExtension < maximumHorizontalArmExtensionMeters);
      }            
      else{
        allowVerticalMovementDespiteStop =
          (this.inSpeedMode == true) ?
          (this.requestedVerticalMotorSpeed > 0.0) :
          (this.requestedVerticalArmExtension > minimumVerticalArmExtensionMeters && this.requestedVerticalArmExtension < maximumHorizontalArmExtensionMeters);
      }

      // move motors in speed mode - where the input is in speed units
      if(this.inSpeedMode) {
        // horizontal motor
        if(this.horizontalArmMageneticSensor.get() && !allowHorizontalMovementDespiteStop){
          this.horizontalMotor.set(0.0);
          this.horizontalEncoder.setPosition(0.0);
        }
        else{
          this.horizontalMotor.set(this.requestedHorizontalMotorSpeed);
        }
        // vertical motor
        if(this.verticalArmMageneticSensor.get() && !allowVerticalMovementDespiteStop){
          this.verticalMotor.set(0.0);
          this.verticalEncoder.setPosition(0.0);
        }
        else{
          this.verticalMotor.set(this.requestedVerticalMotorSpeed);
        }
      }
      // use smart motion to control the movement
      else {
        // horizontal motor
        if(this.horizontalArmMageneticSensor.get() && !allowHorizontalMovementDespiteStop){
          this.horizontalMotor.set(0.0);
          this.horizontalEncoder.setPosition(0.0);
        }
        else{
          horizontalPidController.setReference(
            this.convertHorizontalArmExtensionFromMetersToTicks(this.requestedHorizontalArmExtension),
            ControlType.kSmartMotion);
        }
        // vertical motor
        if(this.verticalArmMageneticSensor.get() && !allowVerticalMovementDespiteStop){
          this.verticalMotor.set(0.0);
          this.verticalEncoder.setPosition(0.0);
        }
        else{
          verticalPidController.setReference(
            this.convertHorizontalArmExtensionFromMetersToTicks(this.requestedVerticalArmExtension),
            ControlType.kSmartMotion);
        }
      }


    }

    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }

    /* *********************************************************************
    PRIVATE METHODS
    ************************************************************************/
    /**
     * Convert the horizontal arm extension from ticks to meters
     * @param targetPositionTicks - the arms extension distance in ticks
     * @return the distance in meters the arm extension is epected for coresponding ticks
     */
    private double convertHorizontalArmExtensionFromTicksToMeters(double targetPositionTicks) {
      return targetPositionTicks * horizontalArmMovementInMetersPerMotorRotation * Constants.RevNeoEncoderTicksPerRevolution;
    }

    /**
     * Convert the vertical arm extension from ticks to meters
     * @param targetPositionTicks - the arms extension distance in ticks
     * @return the distance in meters the arm extension is epected for coresponding ticks
     */
    private double convertVerticalArmExtensionFromTicksToMeters(double targetPositionTicks) {
      return targetPositionTicks * verticalArmMovementInMetersPerMotorRotation * Constants.RevNeoEncoderTicksPerRevolution;
    }

    /**
     * Convert the horizontal arm extension from meters to ticks
     * @param targetPositionMeters - the arms extension distance in meters
     * @return the distance in motor encoder ticks epected for the arm extension in meters
     */
    private double convertHorizontalArmExtensionFromMetersToTicks(double extensionInMeters) {
      return extensionInMeters * Constants.RevNeoEncoderTicksPerRevolution / horizontalArmMovementInMetersPerMotorRotation;
    }

    /**
     * Convert the vertical arm extension from meters to ticks
     * @param targetPositionMeters - the arms extension distance in meters
     * @return the distance in motor encoder ticks epected for the arm extension in meters
     */
    private double convertVerticalArmExtensionFromMetersToTicks(double extensionInMeters) {
      return extensionInMeters * Constants.RevNeoEncoderTicksPerRevolution / verticalArmMovementInMetersPerMotorRotation;
    }

    /**
     * A method to return the current horizontal arms extension in meters
     * @return the distance in meters the arm is expected to be deployed based on current motor encoder values
     */
    private double getCurrentHorizontalArmExtensionInMeters()
    {
      return this.convertHorizontalArmExtensionFromTicksToMeters(this.horizontalEncoder.getPosition());
    }

    /**
     * A method to return the current vertical arms extension in meters
     * @return the distance in meters the arm is expected to be deployed based on current motor encoder values
     */
    private double getCurrentVerticalArmExtensionInMeters()
    {
      return this.convertVerticalArmExtensionFromTicksToMeters(this.verticalEncoder.getPosition());
    }

    /**
     * A function intended to be called from perodic to update the robots centroid position on the field.
     */
    private void refreshArmPosition() {
      SmartDashboard.putNumber("HorizontalArmTicks", this.horizontalEncoder.getPosition());
      SmartDashboard.putNumber("HorizontalArmMeters", this.getCurrentHorizontalArmExtensionInMeters());
      SmartDashboard.putNumber("VerticalArmTicks", this.verticalEncoder.getPosition());
      SmartDashboard.putNumber("VerticalArmMeters", this.getCurrentVerticalArmExtensionInMeters());
    }

    // a method devoted to establishing proper startup of the jaws motors
    // this method sets all of the key settings that will help in motion magic
    private void initializeMotorsSmartMotion() {
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

        verticalMotor.restoreFactoryDefaults();
        verticalMotor.setIdleMode(IdleMode.kBrake);
        verticalPidController = verticalMotor.getPIDController();
        verticalEncoder = verticalMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, (int)Constants.RevNeoEncoderTicksPerRevolution);
        verticalEncoder.setPositionConversionFactor((double)Constants.RevNeoEncoderTicksPerRevolution);
   
        // set PID coefficients
        verticalPidController.setP(kP);
        verticalPidController.setI(kI);
        verticalPidController.setD(kD);
        verticalPidController.setIZone(kIz);
        verticalPidController.setFF(kFF);
        verticalPidController.setOutputRange(kMinOutput, kMaxOutput);
    
        verticalPidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        verticalPidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        verticalPidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        verticalPidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

        horizontalMotor.restoreFactoryDefaults();
        horizontalMotor.setIdleMode(IdleMode.kBrake);
        horizontalPidController = horizontalMotor.getPIDController();
        horizontalEncoder = horizontalMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, (int)Constants.RevNeoEncoderTicksPerRevolution);
        horizontalEncoder.setPositionConversionFactor((double)Constants.RevNeoEncoderTicksPerRevolution);
   
        // set PID coefficients
        horizontalPidController.setP(kP);
        horizontalPidController.setI(kI);
        horizontalPidController.setD(kD);
        horizontalPidController.setIZone(kIz);
        horizontalPidController.setFF(kFF);
        horizontalPidController.setOutputRange(kMinOutput, kMaxOutput);
    
        horizontalPidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        horizontalPidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        horizontalPidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        horizontalPidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

        this.motorsInitalizedForSmartMotion = true;
      }
    }
}
