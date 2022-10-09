// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This project tests the SPARK MAX simulation. 
 * It contains a signle motor that is set to 0.5 speed in teleopInit.  
 */
public class RobotContainer {
  CANSparkMax myMotor = new CANSparkMax(1, MotorType.kBrushless);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null; 
  }

  /**
   * Method to run in main {@link Robot} class teleopInit
   */
  public void teleopInit(){
    myMotor.set(0.5);
  }

  /**
   * Method to run in main {@link Robot} class simulationInit
   */
  public void simulationInit(){
    REVPhysicsSim.getInstance().addSparkMax(myMotor, DCMotor.getNEO(1));
    
  }

  /**
   * Method to run in main {@link Robot} class simulationPeriodic
   */
  public void simulationPeriodic(){
    REVPhysicsSim.getInstance().run();    
    System.out.println("Motor spped: " + myMotor.get());
  }

}
