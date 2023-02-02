// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ProximitySubsystem extends SubsystemBase {

  // put CAN ID in constants.java
  public static TimeOfFlight tofSensor = new TimeOfFlight(0);

  public void ProximitySubysytem(){
    tofSensor.setRangingMode(RangingMode.Short, 200);
  }
  

public double getRange(){
  //gets range and converts from millimeters to inches
   return Units.metersToInches(1000 * tofSensor.getRange());
}

public static final double getRangeSigma(){
  return  tofSensor.getRangeSigma();
}

public boolean isRangeValid(){
  return tofSensor.isRangeValid();
}
}
