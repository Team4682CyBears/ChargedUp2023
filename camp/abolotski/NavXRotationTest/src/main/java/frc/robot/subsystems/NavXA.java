// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

//import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NavXA extends SubsystemBase{

  private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP

  /** Creates a new ExampleSubsystem. */
  public NavXA()
  {
    this.doShuffleboardPublish();
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    return runOnce(
      () -> {
        /* one-time action goes here */
      });
  }
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("DriveTrainY", this::getY, null);
    builder.addDoubleProperty("DriveTrainX", this::getX, null);
    builder.addDoubleProperty("DriveTrainYaw", this::getYaw, null);
    builder.addDoubleProperty("DriveTrainPich", this::getPitch, null);
    builder.addDoubleProperty("DriveTrainRoll", this::getRoll, null);
  }

  public void doShuffleboardPublish() {
    SmartDashboard.putNumber("DriveTrainY", this.getY());
    SmartDashboard.putNumber("DriveTrainX", this.getX());
    SmartDashboard.putNumber("DriveTrainZ", this.getZ());
    SmartDashboard.putNumber("DriveTrainYaw", this.getYaw());
    SmartDashboard.putNumber("DriveTrainPich", this.getPitch());
    SmartDashboard.putNumber("DriveTrainRoll", this.getRoll());
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroGyroscope() {
        m_navx.zeroYaw();
  }

  public double getX() {
        return m_navx.getDisplacementX();
  }
  public double getY() {
        return m_navx.getDisplacementY();
  }
  public double getZ() {
        return m_navx.getDisplacementZ();
  }
  public double getYaw() {
        return m_navx.getYaw();
  }
  public double getPitch() {
        return m_navx.getPitch();
  }
  public double getRoll() {
        return m_navx.getRoll();
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.doShuffleboardPublish();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
