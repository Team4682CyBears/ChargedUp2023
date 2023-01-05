// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PDPRead extends SubsystemBase {
  PowerDistribution power1 = new PowerDistribution(1, ModuleType.kCTRE);
  private WPI_TalonSRX myMotor = new WPI_TalonSRX(Constants.myMotorCanId);
  /** Creates a new ExampleSubsystem. */
  public PDPRead() {
    double voltage = power1.getVoltage();
    SmartDashboard.putNumber("Voltage", voltage);
        // Get the total current of all channels.
        double totalCurrent = power1.getTotalCurrent();
        SmartDashboard.putNumber("Total Current", totalCurrent);
    
        // Get the total power of all channels.
        // Power is the bus voltage multiplied by the current with the units Watts.
        double totalPower = power1.getTotalPower();
        SmartDashboard.putNumber("Total Power", totalPower);
    
        // Get the total energy of all channels.
        // Energy is the power summed over time with units Joules.
        double totalEnergy = power1.getTotalEnergy();
        SmartDashboard.putNumber("Total Energy", totalEnergy);
  }
  public void Forwards()
  {
    myMotor.set(Constants.motorSpeed);
  }
  public void Panicbot()
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
