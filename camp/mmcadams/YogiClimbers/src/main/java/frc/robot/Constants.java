// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double DegreesPerRevolution = 360.0;

    // Neo maximum RPM 
    public static final double neoMaximumRevolutionsPerMinute = 5676;

    // see: https://www.revrobotics.com/rev-21-1650/ where it says "42 counts per rev."
    public static final double RevNeoEncoderTicksPerRevolution = 42; 
    public static final int countPerRevHallSensor = 42;

    // telescoping arms reach points
    public static final double telescopingArmsRetractHeightInches = 0.50;
    public static final double telescopingArmsMediumExtendHeightInches = 31.0;
    public static final double telescopingArmsHighExtendHeightInches = 20.0;
    public static final double telescopingArmsToleranceInches = 0.10;
    public static final double telescopingArmsDefaultExtendSpeed = 0.7;
    public static final double telescopingArmsDefaultRetractSpeed = -0.7;
    public static final double telescopingArmsStopSpeed = 0.0;

    // can ids for arms
    public static final int telescopingArmsMotorLeftCanId = 13;
    public static final int telescopingArmsMotorRightCanId = 14;

    // max time for an telescoping arms operation
    public static final double maximumTelescopingArmsTimeOperationSeconds = 3.0;

    // button board input port
    public static final int highLevelButtonBoardPort = 2;
    public static final int lowLevelButtonBoardPort = 3;
}
