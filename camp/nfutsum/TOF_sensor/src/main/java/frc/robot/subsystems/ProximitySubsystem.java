// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ProximitySubsystem extends SubsystemBase {

  // put CAN ID in constants.java
  public static TimeOfFlight tofSensor = new TimeOfFlight(Constants.canID);

  public void ProximitySubysytem(){
    tofSensor.setRangingMode(RangingMode.Short, 200);
  }
  

public double getRange(){
  //gets range and converts from millimeters to inches
    double range = Units.metersToInches(1000 * tofSensor.getRange());
    System.out.println("The range is " + range);
    return range;
}

public static final double getRangeSigma(){
  return  tofSensor.getRangeSigma();
}

public boolean isRangeValid(){
  return tofSensor.isRangeValid();
}
}
