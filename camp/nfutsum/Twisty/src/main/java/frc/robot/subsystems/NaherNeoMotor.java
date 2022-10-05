// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class NaherNeoMotor extends SubsystemBase {

  private CANSparkMax myMotor = new CANSparkMax(Constants.myNeoMotorCanBusNumber, MotorType.kBrushless);
    
  /** Creates a new ExampleSubsystem. */
  public NaherNeoMotor() {
  }

  public void setSpeed(double speed){
    myMotor.set(speed);
  }

  public void stopMotor(){
    myMotor.set(0.0);
  }

  /**
   * required for simulation
   * @return CANSparkMax motor
   */
  public CANSparkMax getInstance(){
    return myMotor;
  }
}