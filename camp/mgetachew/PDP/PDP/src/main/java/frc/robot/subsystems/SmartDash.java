// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SmartDash extends SubsystemBase {
  private static final double SpeedMotor = null;

  /** Creates a new ExampleSubsystem. */
  public SmartDash() {}

  /* (non-Javadoc)
   * @see edu.wpi.first.wpilibj2.command.Subsystem#periodic()
   */
  @Override
  public void periodic() {

    TheRevMoter.driveTheMotor();

       // Get the total current of all channels.
       double totalCurrent = TheRevMoter.getTotalCurrent();
       SmartDashboard.putNumber("Total Current", totalCurrent);
   
       // Get the total power of all channels.
       // Power is the bus voltage multiplied by the current with the units Watts.
       double totalPower = TheRevMoter.getTotalPower();
       SmartDashboard.putNumber("Total Power", totalPower);
   
       // Get the total energy of all channels.
       // Energy is the power summed over time with units Joules.
       double totalEnergy = TheRevMoter.getTotalEnergy();
       SmartDashboard.putNumber("Total Energy", totalEnergy);

       // Get the total energy of all channels.
       // Energy is the power summed over time with units Joules.
       SmartDashboard.putData(SpeedMotor);

       double Speedtest = SpeedMotor;
       SmartDashboard.putNumber("Speedtest", SpeedMotor);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
