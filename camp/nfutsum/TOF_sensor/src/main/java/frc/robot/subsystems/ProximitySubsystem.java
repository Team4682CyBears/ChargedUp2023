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
    /*EXPERIMENT RESULTS
    Found that TOF sensor max distance ~ 36in/min distance ~ 1in when using 2.5 by 3 inch black and white box.
    Found that TOF sensor max distance ~ 40in/min distance ~ 1in when using 3 by 4.7 block of wood.
    Found that TOF sensor max distance ~ 40in/min distance ~ 0.5in when using 5 by 4 Yellow plastic container.
    
    Found that sensor works best when target is plastic and about 3x4.7 inches. 
    Wood also works nicely and size could probably be sightly adjusted if too large.
*/
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
