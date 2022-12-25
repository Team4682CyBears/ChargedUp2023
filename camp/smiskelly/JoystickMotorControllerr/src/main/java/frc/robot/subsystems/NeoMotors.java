// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class NeoMotors extends SubsystemBase {

  private CANSparkMax theMotor = new CANSparkMax(Constants.myMotorCanID, MotorType.kBrushless);

  /** Creates a new ExampleSubsystem. */
  public NeoMotors() {}

  public void setMotorSpeed(double speed){
    theMotor.set(speed);
  }
  
  
  public void MotorStop(){
    theMotor.set(0.0);
  }
  public CANSparkMax getInstance(){
    return theMotor;
  }

  
}
