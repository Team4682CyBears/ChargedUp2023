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

  // Defines Time of Flight sensor 
  public static TimeOfFlight tofSensor = new TimeOfFlight(Constants.canID);

  //Sets the ranging mode to medium (Between 76cm-290cm depending on lighting).
  public void ProximitySubysytem(){
    tofSensor.setRangingMode(RangingMode.Medium, 140);
  }
  

public double getRange(){
  //gets range and converts from millimeters to inches
    double range = Units.metersToInches(tofSensor.getRange()/1000)-1.5;
//-1.5 in offset    
    System.out.println("The range is " + range + " inches");
    return range;
}
//flashes the sensor
public void blinkSensor(){
  tofSensor.identifySensor();
}
//Standard deviation of distance measurment in millimeters
public static final double getRangeSigma(){
  return  tofSensor.getRangeSigma();
}
//Returns true if sensor correctly measured distance from object
public boolean isRangeValid(){
  return tofSensor.isRangeValid();
}
}
