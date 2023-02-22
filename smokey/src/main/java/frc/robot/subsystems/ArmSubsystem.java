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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.common.*;
import java.util.*;

//import javax.lang.model.util.ElementScanner14;

public class ArmSubsystem extends SubsystemBase
{
    /* *********************************************************************
    CONSTANTS
    ************************************************************************/
    // expected to be < 1.0 due to encoder granularity being lower for Rev/Neo
    private static final double telescopingArmsMotorEncoderTicksPerDegree = Constants.RevNeoEncoderTicksPerRevolution / Constants.DegreesPerRevolution;
    // Discussion with Nathan on 02/08/2023 on 'angle arm' - 0.375" per hole * 15 teeth - gearbox is aprox 1:100
    // TODO - confirm gearbox reduction
    private static final double verticalArmMovementInMetersPerMotorRotation = (0.009525 * 15) * (1.0 / 100.0); 
    // Discussion with Owen on 02/08/2023 on 'extension arm' - 5mm per tooth on belt 36 teeth - gearbox is aprox 1:10
    // TODO - confirm gearbox reduction
    private static final double horizontalArmMovementInMetersPerMotorRotation = (0.005 * 36) * (1.0 / 10.0); 
    
    // the extension distances of the arms - in meters
    private static final double minimumVerticalArmExtensionMeters = 0.0;
    private static final double maximumVerticalArmExtensionMeters = 0.254; // 10.0 inches
    private static final double toleranceVerticalArmExtensionMeters = 0.001;
    private static final double minimumHorizontalArmExtensionMeters = 0.0;
    private static final double maximumHorizontalArmExtensionMeters = 0.9439402; // 78.5" - 41.337" == 37.163 inches
    private static final double toleranceHorizontalArmExtensionMeters = 0.001;

    // the various geometry aspects of the arm setup
    private static final double lengthFloorToHorizontalArmPivotMeters = 0.0762; // 3 inches
    private static final double lengthFloorToVerticalArmPivotMeters = 0.0762; // 3 inches
    private static final double lengthBasePinDistanceBetweewnHorizontalAndVerticalArmsMeters = 0.572262; // 22.53 inches
    private static final double lengthHorizontalArmPinDistanceMeters = 0.5554472; // 21.868 inches
    private static final double lengthMinimumVerticalArmMeters = 0.5394706; //0.5140706 - 20.239 inches  // 0.5394706 - 21.239??
    private static final double lengthMaximumVerticalArmMeters = lengthMinimumVerticalArmMeters + (maximumVerticalArmExtensionMeters - minimumVerticalArmExtensionMeters);
    private static final double lengthMinimumHorizontalArmMeters = 1.1537188; // 45.422 inches
    private static final double lengthMaximumHorizontalArmMeters = lengthMinimumHorizontalArmMeters + (maximumHorizontalArmExtensionMeters - minimumHorizontalArmExtensionMeters);

    // TODO - use something less than 1.0 for testing
    private static final double neoMotorSpeedReductionFactor = 1.0;

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
    private boolean motorsInitalizedForSmartMotion = false;

    private DigitalInput verticalArmMageneticSensor = new DigitalInput(Constants.VirticalArmMagneticSensor);
//    private DigitalInput horizontalArmMageneticSensor = new DigitalInput(Constants.HorizontalArmMagneticSensor);
    private DigitalInput horizontalArmMageneticSensor = verticalArmMageneticSensor;

    private boolean isHorizontalMotorInverted = true;
    private boolean isVerticalMotorInverted = true;

    private boolean inSpeedMode = true;
    private boolean movementWithinTolerance = false;
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
    public void setArmSpeeds(double horizontalArmSpeed, double verticalArmSpeed) {
      this.inSpeedMode = true;
      this.movementWithinTolerance = false;
      this.requestedHorizontalMotorSpeed = MotorUtils.truncateValue(horizontalArmSpeed, -1.0, 1.0);
      this.requestedVerticalMotorSpeed = MotorUtils.truncateValue(verticalArmSpeed, -1.0, 1.0);
    }

    /**
     * A method to set requested the arms motor extension distance
     * @param horizontalArmExtension the distance to extend the vertical arm to
     * @param verticalArmExtension the distance to extend the vertical arm to
     */
    public void setArmExtensions(double horizontalArmExtension, double verticalArmExtension) {
      this.inSpeedMode = false;
      this.movementWithinTolerance = false;
      this.requestedHorizontalArmExtension = MotorUtils.truncateValue(horizontalArmExtension, minimumHorizontalArmExtensionMeters, maximumHorizontalArmExtensionMeters);
      this.requestedVerticalArmExtension = MotorUtils.truncateValue(verticalArmExtension, minimumVerticalArmExtensionMeters, maximumVerticalArmExtensionMeters);
    }
    
    /**
     * A method to set requested the arms motor extension distance
     * @param yPointMeters the y aspect of the arm from the primary arm pivot center in meters
     * @param zPointMeters the z aspect of the arm above the level playing floor in meters
     * @return true if the position is valid and was set, otherwise false
     */
    public boolean setArmToPointInSpace(double yPointMeters, double zPointMeters) {
      
      double requestedAngle = Math.atan(zPointMeters/yPointMeters);
      double requestedHorizontalArmLength = yPointMeters/Math.cos(requestedAngle);
      double requestedVerticalArmLength = 
        Math.sqrt(
          lengthHorizontalArmPinDistanceMeters * lengthHorizontalArmPinDistanceMeters +
          lengthBasePinDistanceBetweewnHorizontalAndVerticalArmsMeters * lengthBasePinDistanceBetweewnHorizontalAndVerticalArmsMeters -
          (2 *lengthHorizontalArmPinDistanceMeters * lengthBasePinDistanceBetweewnHorizontalAndVerticalArmsMeters * Math.cos(requestedAngle)));

      double requestedHorizontalArmExtensionMeters = requestedHorizontalArmLength - lengthMinimumHorizontalArmMeters;
      double requestedVerticalArmExtensionMeters = requestedVerticalArmLength - lengthMinimumVerticalArmMeters;

      boolean armPointInSpaceValid = false;
      if( requestedHorizontalArmExtensionMeters >= minimumHorizontalArmExtensionMeters &&
        requestedHorizontalArmExtensionMeters <= maximumHorizontalArmExtensionMeters &&
        requestedVerticalArmExtensionMeters >= minimumVerticalArmExtensionMeters &&
        requestedVerticalArmExtensionMeters <= maximumVerticalArmExtensionMeters) {
          armPointInSpaceValid = true;
          this.setArmExtensions(requestedHorizontalArmExtensionMeters, requestedVerticalArmExtensionMeters);
      }
      else {
        System.out.println("!!!INVALID POSITION REQUESTED!!!");
        System.out.println("angle radians = " + requestedAngle +
        "\nhorizontal len = " + requestedHorizontalArmLength +
        "\nvertical len = " + requestedVerticalArmLength +
        "\nhorizontal extension = " + requestedHorizontalArmExtensionMeters +
        "\nvertical extension = " + requestedVerticalArmExtensionMeters);
      }
      return armPointInSpaceValid;
    }

    /**
     * Method to help indicate when a requested movement is complete
     * @return true when the arms have arrived at their extension distances, else false
     */
    public boolean isRequestedArmMovementComplete() {
      return this.inSpeedMode == false &&  this.movementWithinTolerance;
    }

    /**
     * A method to handle periodic processing
     */
    @Override
    public void periodic() {

      // confirm that the smart motion is setup - no-op after it is setup first time
      this.initializeMotorsSmartMotion();
      this.refreshArmPosition();

      // determine if the movement is in the stop range
      // stop range implies any of the following:
      // magnetic sensor triggered 
      // arm deployed >= limit (e.g., maximumXXXArmExtensionMeters)
      double currentHorizontalExtensionInMeters = this.getCurrentHorizontalArmExtensionInMeters();
      boolean isHorizontalArmAtOrBelowLowStop = (this.horizontalArmMageneticSensor.get() == false);
      boolean isHorizontalArmAtOrAboveHighStop = currentHorizontalExtensionInMeters >= maximumHorizontalArmExtensionMeters;
      double currentVerticalExtensionInMeters = this.getCurrentVerticalArmExtensionInMeters();
      boolean isVerticalArmAtOrBelowLowStop = (this.verticalArmMageneticSensor.get() == false);
      boolean isVerticalArmAtOrAboveHighStop = currentVerticalExtensionInMeters >= maximumVerticalArmExtensionMeters;

      // if we are in speed mode always set motor speeds using motor set
      if(this.inSpeedMode) {

        // Horizontal
        if(isHorizontalArmAtOrBelowLowStop && this.requestedHorizontalMotorSpeed < 0.0) {
          this.horizontalMotor.set(0.0);
          this.horizontalEncoder.setPosition(0.0);
        }
        else if(isHorizontalArmAtOrAboveHighStop && this.requestedHorizontalMotorSpeed > 0.0) {
          this.horizontalMotor.set(0.0);
        }
        else {
          this.horizontalMotor.set(this.requestedHorizontalMotorSpeed * neoMotorSpeedReductionFactor);
        }
        
        // Vertical
        if(isVerticalArmAtOrBelowLowStop && this.requestedVerticalMotorSpeed < 0.0) {
          this.verticalMotor.set(0.0);
          this.verticalEncoder.setPosition(0.0);
        }
        else if(isVerticalArmAtOrAboveHighStop && this.requestedVerticalMotorSpeed > 0.0) {
          this.verticalMotor.set(0.0);
        }
        else {
          this.verticalMotor.set(this.requestedVerticalMotorSpeed * neoMotorSpeedReductionFactor);
        }

      }
      // if not in speed mode we assume the caller wants smart motion
      else {

        boolean isHorizontalWithinTolerance =  (Math.abs(currentHorizontalExtensionInMeters - this.requestedHorizontalArmExtension) <= toleranceHorizontalArmExtensionMeters);
        boolean isVerticalWithinTolerance =  (Math.abs(currentVerticalExtensionInMeters - this.requestedVerticalArmExtension) <= toleranceVerticalArmExtensionMeters);
        movementWithinTolerance = isHorizontalWithinTolerance && isVerticalWithinTolerance;

        // Horizontal
        if(isHorizontalArmAtOrBelowLowStop && this.requestedHorizontalArmExtension <= 0.0) {
          this.horizontalMotor.set(0.0);
          this.horizontalEncoder.setPosition(0.0);
        }
        else if(isHorizontalArmAtOrAboveHighStop && this.requestedHorizontalArmExtension >= maximumHorizontalArmExtensionMeters) {
          this.horizontalMotor.set(0.0);
        }
        else if (isHorizontalWithinTolerance) {
          this.horizontalMotor.set(0.0);
        }
        else {
          horizontalPidController.setReference(
            this.convertHorizontalArmExtensionFromMetersToTicks(this.requestedHorizontalArmExtension),
            ControlType.kSmartMotion);
        }

        // Vertical
        if(isVerticalArmAtOrBelowLowStop && this.requestedVerticalArmExtension <= 0.0) {
          this.verticalMotor.set(0.0);
          this.verticalEncoder.setPosition(0.0);
        }
        else if(isVerticalArmAtOrAboveHighStop && this.requestedVerticalArmExtension >= maximumVerticalArmExtensionMeters) {
          this.verticalMotor.set(0.0);
        }
        else if (isVerticalWithinTolerance) {
          this.verticalMotor.set(0.0);
        }
        else {
          verticalPidController.setReference(
            this.convertVerticalArmExtensionFromMetersToTicks(this.requestedVerticalArmExtension),
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
      return targetPositionTicks / Constants.RevNeoEncoderTicksPerRevolution * horizontalArmMovementInMetersPerMotorRotation;
    }

    /**
     * Convert the vertical arm extension from ticks to meters
     * @param targetPositionTicks - the arms extension distance in ticks
     * @return the distance in meters the arm extension is epected for coresponding ticks
     */
    private double convertVerticalArmExtensionFromTicksToMeters(double targetPositionTicks) {
      return targetPositionTicks / Constants.RevNeoEncoderTicksPerRevolution * verticalArmMovementInMetersPerMotorRotation;
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
    private double getCurrentHorizontalArmExtensionInMeters() {
      return this.convertHorizontalArmExtensionFromTicksToMeters(this.horizontalEncoder.getPosition());
    }

    /**
     * A method to return the current vertical arms extension in meters
     * @return the distance in meters the arm is expected to be deployed based on current motor encoder values
     */
    private double getCurrentVerticalArmExtensionInMeters() {
      return this.convertVerticalArmExtensionFromTicksToMeters(this.verticalEncoder.getPosition());
    }

    /**
     * A method to return the current horizontal arm angle measured from floor to arm centerline
     * @return the angle
     */
    private double getCurrentHorizontalArmAngleRadians() {
      // need to use arc cos equation for angle given all three sides
      double a = lengthMinimumVerticalArmMeters + this.getCurrentVerticalArmExtensionInMeters();
      double b = lengthHorizontalArmPinDistanceMeters;
      double c = lengthBasePinDistanceBetweewnHorizontalAndVerticalArmsMeters;
      return Math.acos((Math.pow(b, 2) + Math.pow(c, 2) - Math.pow(a, 2))/(2 * b * c));
    }

    /**
     * A method to return the current horizontal arm angle measured from floor to arm centerline
     * @return the angle
     */
    private double getCurrentHorizontalArmAngleDegrees() {
      return Units.radiansToDegrees(this.getCurrentHorizontalArmAngleRadians());
    }

    /**
     * A method to return the current arms Z height in meters from the floor
     * @return the height in meters from the floor to the arm tip
     */
    private double getCurrentArmsHeightInMeters() {
      return lengthFloorToHorizontalArmPivotMeters + (Math.sin(this.getCurrentHorizontalArmAngleRadians()) * (lengthMinimumHorizontalArmMeters + this.getCurrentHorizontalArmExtensionInMeters()));
    }

    /**
     * A method to return the current arms Y distance in meters from the floor
     * @return the height in meters from the floor to the arm tip
     */
    private double getCurrentArmsDistanceInMeters() {
      return Math.cos(this.getCurrentHorizontalArmAngleRadians()) * (lengthMinimumHorizontalArmMeters + this.getCurrentHorizontalArmExtensionInMeters());
    }

    /**
     * A function intended to be called from perodic to update the robots centroid position on the field.
     */
    private void refreshArmPosition() {
      SmartDashboard.putNumber("HorizontalArmMotorTicks", this.horizontalEncoder.getPosition());
      SmartDashboard.putNumber("VerticalArmMotorTicks", this.verticalEncoder.getPosition());
      SmartDashboard.putNumber("ExtensionHorizontalArmMeters", this.getCurrentHorizontalArmExtensionInMeters());
      SmartDashboard.putNumber("ExtensionVerticalArmMeters", this.getCurrentVerticalArmExtensionInMeters());
      SmartDashboard.putNumber("ArmAngleRadians", this.getCurrentHorizontalArmAngleRadians());
      SmartDashboard.putNumber("ArmAngleDegrees", this.getCurrentHorizontalArmAngleDegrees());
      SmartDashboard.putNumber("ArmHeightMetersZ", this.getCurrentArmsHeightInMeters());
      SmartDashboard.putNumber("ArmDistanceMetersY", this.getCurrentArmsDistanceInMeters());
    }

    // a method devoted to establishing proper startup of the jaws motors
    // this method sets all of the key settings that will help in motion magic
    private void initializeMotorsSmartMotion() {
      if(motorsInitalizedForSmartMotion == false) { 
        // PID coefficients
        kP = 2e-4; 
        kI = 0;
        kD = 0;
        kIz = 0; 
        kFF = 0.00001; 
        kMaxOutput = 1; 
        kMinOutput = -1;
        maxRPM = Constants.neoMaximumRevolutionsPerMinute;
        int smartMotionSlot = 0;
    
        // Smart Motion Coefficients
        maxVel = maxRPM * neoMotorSpeedReductionFactor; // rpm
        maxAcc = maxVel * 2; // 1/2 second to get up to full speed

        verticalMotor.restoreFactoryDefaults();
        verticalMotor.setIdleMode(IdleMode.kBrake);
        verticalMotor.setInverted(this.isVerticalMotorInverted);
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
        horizontalMotor.setInverted(this.isHorizontalMotorInverted);
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
