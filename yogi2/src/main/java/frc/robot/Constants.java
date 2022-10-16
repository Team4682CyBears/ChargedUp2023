// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rapid React - 2022
// File: Constants.java
// Intent: Forms all constants for the robot.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot;

import frc.robot.*;
import frc.robot.common.Gains;
import frc.robot.common.RoboRioOrientation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
  public static final double DegreesPerRevolution = 360.0;
  // Talon maximum RPM 
  public static final double neoMaximumRevolutionsPerMinute = 5676;
  // Talon maximum RPM 
  public static final double talonMaximumRevolutionsPerMinute = 6380;
  // this uses the on-motor quadratrue encoder
  // see: https://www.vexrobotics.com/pro/falcon-500 where it says "2048 CPR encoder"
  public static final double CtreTalonFx500EncoderTicksPerRevolution = 2048; 
  // this uses the halls effect sensor when plugged into the spark max
  // see: https://www.revrobotics.com/rev-21-1650/ where it says "42 counts per rev."
  public static final double RevNeoEncoderTicksPerRevolution = 42; 

  // shooter 
  public static boolean shooterBottomMotorDefaultDirection = false;
  public static boolean shooterTopMotorDefaultDirection = true;

  // Angle arms magic numbers
  public static final double angleArmsManualMotorStopSpeed = 0.0;
  public static final double angleArmsManualMotorForwardSpeed = 0.8;
  public static final double angleArmsManualMotorReverseSpeed = -0.8;
  public static final double angleArmsMinimumPositionAngle = -5.0;
  public static final double angleArmsReferencePositionAngle = 0.0;
  public static final double angleArmsMaximumPositionAngle = 90.0;
  public static final double angleArmsReadyPositionAngle = 20.0;
  public static final double angleArmsBarPositionAngle = 0.0;
  public static final double angleArmsTiltPositionAngle = 60.0;
  public static final double angleArmsMotorEffectiveGearRatio = 50.0;
  public static final double angleArmsSetpointTolerance = 1.0;
  public static final double angleArmsReferencePositionMotorEncoderUnits = 0.0;

  // Motor magic numbers
  public static final double defaultMotorSpeedToleranceRpm = 20.0;
  public static final double largeMotorSpeedToleranceRpm = 100.0;
  public static final double smallMotorSpeedToleranceRpm = 20.0;
  public static final double bottomMotorForwardLowGoalSpeedRpm = -1300.0;
  public static final double topMotorForwardLowGoalSpeedRpm = -1100.0;
  public static final double bottomMotorForwardHighGoalSpeedRpm = -2650.0;
  public static final double topMotorForwardHighGoalSpeedRpm = -2250.0;
  public static final double bottomMotorForwardMidGoalSpeedRpm = -3048.0;
  public static final double topMotorForwardMidGoalSpeedRpm = -2588.0;
  public static final double bottomMotorReverseHighGoalSpeedRpm = -2600.0;
  public static final double topMotorReverseHighGoalSpeedRpm = -2200.0;
  public static final double bottomMotorReverseLowGoalSpeedRpm = -1700.0;
  public static final double topMotorReverseLowGoalSpeedRpm = -1400.0;
  public static final double bottomMotorIntakeSpeedRpm = 1700.0;
  public static final double topMotorIntakeSpeedRpm = 1400.0;
  
  // Jaws reach points \\
  public static final double jawsIntakePositionAngle = 1.0;
  public static final double jawsLowGoalPositionAngle = 110.0;
  public static final double jawsHighGoalPositionAngle = 130.0; 
  public static final double jawsMidGoalPositionAngle = 117.0;  
  public static final double jawsReverseHighGoalPositionAngle = 147.0;
  public static final double jawsReverseLowGoalPositionAngle = 150.0;
  public static final double jawsPositionAngleTolerance = 1.2;
  public static final double jawsAngleArmsEngagePositionAngle = 125.0;
  public static final double jawsAngleArmsEngagePositionTolerance = 0.9;
  public static final double jawsDefaultPositiveSpeed = 0.6;
  public static final double jawsDefaultNegativeSpeed = -0.6;
  public static final double jawsReferencePositionMotorEncoderUnits = 148200; // 141000, 151972 and 152037

  // telescoping arms reach points \\
  public static final double telescopingArmsRetractHeightInches = 0.50;
  public static final double telescopingArmsMediumExtendHeightInches = 31.0;
  public static final double telescopingArmsHighExtendHeightInches = 20.0;
  public static final double telescopingArmsToleranceInches = 0.10;
  public static final double telescopingArmsDefaultExtendSpeed = 0.7;
  public static final double telescopingArmsDefaultRetractSpeed = -0.7;
  public static final double telescopingArmsStopSpeed = 0.0;

  // Ball storage \\
  public static final double ballStoreSpeed = -0.22;
  public static final double ballRetrieveSpeed = 0.2;
  public static final int maximumStoredBallCount = 2;

  // *********************************************
  // CAN BUS NUMBERS \\
  // *********************************************
  public static final int angleArmsMotorCanId = 4;

  public static final int shooterMotorBottomCanId = 5;
  public static final int shooterMotorTopCanId = 6;

  public static final int driveMotorLeftFrontCanId = 7;
  public static final int driveMotorLeftRearCanId = 8;
  public static final int driveMotorRightFrontCanId = 11;
  public static final int driveMotorRightRearCanId = 12;

  public static final int jawsMotorRightCanId = 9;
  public static final int jawsMotorLeftCanId = 10;

  public static final int telescopingArmsMotorLeftCanId = 13;
  public static final int telescopingArmsMotorRightCanId = 14;

  public static final int ballStorageMotorTopCanId = 15;
  public static final int ballStorageMotorBottomCanId = 16;

  // MAXIMUM COMMAND TIMING SETTINGS
  public static final double maximumClimbTimeOperationSeconds = 5.5;
  public static final double maximumJawsTimeOperationSeconds = 3.2;
  public static final double maximumBallStorageTimeOperationSeconds = 1.2;
  public static final double maximumShooterTimeOperationSeconds = 4.5;
  public static final   double maximumTelescopingArmsTimeOperationSeconds = 3.0;

  // MOTOR SETTINGS \\

  // Drive motor magic numbers
  public static boolean driveMotorLeftFrontDefaultDirection = false;
  public static boolean driveMotorLeftRearDefaultDirection = false;
  public static boolean driveMotorRightFrontDefaultDirection = true;
  public static boolean driveMotorRightRearDefaultDirection = true;

  // jaws arm motors clockwise is elevate
  public static TalonFXInvertType jawsRightMotorDefaultDirection = TalonFXInvertType.Clockwise;
  public static TalonFXInvertType jawsLeftMotorDefaultDirection = TalonFXInvertType.FollowMaster;

  // angle arms motor clockwise is pull backward
  public static TalonFXInvertType angleArmsRightMotorDefaultDirection = TalonFXInvertType.Clockwise;

  // TIMING AND SPEEDS \\
  // AngleArm timing \\
  public static final double angleArmTimingSeconds = 0.3;

  // BallStorage timing \\
  public static final double ballStorageStoreTimingSeconds = 1.0;
  public static final double ballStorageRetrieveTimingSeconds = 2.0;

  // HIDS \\
  // hid ports \\ 
  public static int portDriverController = 0;
  public static int portCoDriverController = 1;
  public static final int highLevelButtonBoardPort = 2;
  public static final int lowLevelButtonBoardPort = 3;

  // util \\ //TODO if needed 
  public static final int kPIDLoopIdx = 0;
  public static final int kTimeoutMs = 30;
  public static final int kSlotIdx = 0;

  // gains \\
  public static final Gains kGains = new Gains(0.02, 0.0, 0.0, 0.02, 0, 1.0);
  public static final int countPerRevHallSensor = 42;

  // orentation
  public static final RoboRioOrientation roboRioOrientation = RoboRioOrientation.RelayForward;

}
