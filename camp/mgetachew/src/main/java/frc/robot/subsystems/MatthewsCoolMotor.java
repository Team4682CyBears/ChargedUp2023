// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class MatthewsCoolMotor extends SubsystemBase {

  private WPI_TalonSRX myMotor = new WPI_TalonSRX(Constants.myMotorCanId);

  /** Creates a new RomiDrivetrain. */
  public MatthewsCoolMotor() {
  }

  /**
  * A method exposed to callers to store one ball into storage
  */
  public void footToTheFloorForward()
  {
    myMotor.set(1.0);
  }
  public void footToTheFloorBackward()
  {
    myMotor.set(-1.0);
  }
  public void stopDoingThis()
  {
    myMotor.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
