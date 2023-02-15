// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: Constants.java
// Intent: Forms key constants required for this robot.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

public final class Constants {

    public final static double DriveVoltageScalar = 1.0;
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
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(-353.4-233.0); 

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
    // arm constants
    public static final int HorizontalArmDriveMotorCanId = 5;
    public static final int VerticalArmDriveMotorCanId = 6;
    public static final int VirticalArmMagneticSensor = 0;
    public static final int HorizontalArmMagneticSensor = 1;


    // *****************************************************************
    // standard stuff constants - motors rotation, etc.
    public static final double DegreesPerRevolution = 360.0;
    // NEO maximum RPM 
    public static final double neoMaximumRevolutionsPerMinute = 5676;
    // this uses the halls effect sensor when plugged into the spark max
    // see: https://www.revrobotics.com/rev-21-1650/ where it says "42 counts per rev."
    public static final double RevNeoEncoderTicksPerRevolution = 42;

    // *****************************************************************
    // input device constants
    public static final int portDriverController = 0;
    public static final int portCoDriverController = 1;

    //************************************
    // trajectory constants
    public static final double TrajectoryMaxAcceleration = 1;
    public static final double TrajectoryMaxVelocity = 1;

    // *******************************************************************
    // arm constants - preset positions
    public static final double armPresetPositionStowMetersY = 0.1;
    public static final double armPresetPositionStowMetersZ = 1.23;
    public static final double armPresetPositionGrabMetersY = 1.00;
    public static final double armPresetPositionGrabMetersZ = 1.60;
    public static final double armPresetPositionScoreHighMetersY = 1.10;
    public static final double armPresetPositionScoreHighMetersZ = 1.78;
    public static final double armPresetPositionScoreMediumMetersY = 1.00;
    public static final double armPresetPositionScoreMediumMetersZ = 1.55;
    public static final double armPresetPositionScoreLowMetersY = 0.65;
    public static final double armPresetPositionScoreLowMetersZ = 1.04;

    // *******************************************************************
    // pneumatics constants
    public static final int PneumaticsControlModuleNumber = 1;
    public static final PneumaticsModuleType PneumaticsControlModuleType = PneumaticsModuleType.REVPH;

    // *******************************************************************
    // Stabilizer constants
    public static final int StabilizerPneumaticsControlModuleForwardChannel = 0;
    public static final int StabilizerPneumaticsControlModuleReverseChannel = 1;

    // *******************************************************************
    // grabber constants
    public static final int GrabberHorizontalPneumaticsControlModuleForwardChannel = 2;
    public static final int GrabberHorizontalPneumaticsControlModuleReverseChannel = 3;
    public static final int GrabberVerticalPneumaticsControlModuleForwardChannel = 4;
    public static final int GrabberVerticalPneumaticsControlModuleReverseChannel = 5;  

}