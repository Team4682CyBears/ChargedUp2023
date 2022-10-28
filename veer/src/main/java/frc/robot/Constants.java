// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rapid React - 2022
// File: BallHandler.java
// Intent: Wrapper class standard stub for robot in FRC challange.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.57785; 
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.57785; 

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1; 
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 2; 
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 3; 
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(-353.4-180.0-90.0); 

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 4; 
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 5; 
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 6; 
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(-321.0-180.0-110.0); 

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 7; 
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 8; 
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 9; 
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(-205.5-180.0+10.0); 

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 10; 
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 11; 
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 12; 
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(-305.5-180.0-117.0); 

    // *****************************************************************
    // standard stuff constants - motors rotation, etc.
    public static final double DegreesPerRevolution = 360.0;
    // NEO maximum RPM 
    public static final double neoMaximumRevolutionsPerMinute = 5676;
    // this uses the halls effect sensor when plugged into the spark max
    // see: https://www.revrobotics.com/rev-21-1650/ where it says "42 counts per rev."
    public static final double RevNeoEncoderTicksPerRevolution = 42;

    // *****************************************************************
    // ball handler system constants
    public static final int BallHandlerMotorCanId = 13;
    public static final double BallHandlerMotorDefaultSpeed = 1.0;
    public static final double BallHandlerMotorStorageDirectionMultiplier = 1.0;
    public static final double BallHandlerMotorRetrievalDirectionMultiplier = -1.0;
    public static final double BallHandlerMotorMinimumAbsoluteInputValue = 0.1;
    public static final boolean BallHandlerMotorInvertedDirection = false;
    public static final double BallHandlerPneumaticsDeployCycleTimeSeconds = 0.5;
    public static final double BallHandlerPneumaticsRetractCycleTimeSeconds = 0.5;
    public static final double BallHandlerIntakeBallTimeSeconds = 2.0;
    public static final double BallHandlerLayupBallTimeSeconds = 1.0;

    public static final PneumaticsModuleType BallHandlerPneumaticsControlModuleType = PneumaticsModuleType.CTREPCM;
    public static final int PneumaticsControlModuleNumber = 0;
    public static final int PneumaticsControlModuleForwardChannel = 0;
    public static final int PneumaticsControlModuleReverseChannel = 1;

    // *****************************************************************
    // input device constants
    public static final int portDriverController = 1;
    public static final int portCoDriverController = 2;

    // *****************************************************************
    // telescoping arm system constants
    public static final int TelescopingArmMotorCanId = 14;
    public static final double TelescopingArmDefaultExtendSpeed = 0.7;
    public static final double TelescopingArmDefaultRetractSpeed = -0.7;
    public static final double TelescopingArmStopSpeed = 0.0;    
}
