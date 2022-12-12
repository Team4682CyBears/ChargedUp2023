// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.NaherNeoMotor;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** This command will stop the motor */
public class StopTheMotor extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final NaherNeoMotor m_subsystem;

  /**
   * This stores NaherNeoMotor into subsystem and makes the command to stop it
   *
   * @param subsystem This subsystem is used by the command.
   */
  public StopTheMotor(NaherNeoMotor subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  // This will call the StopTheMotor command when it's initialized 
  @Override
  public void initialize() {}

  // Calls StopTheMotor command while command is scheduled
  @Override
  public void execute() {
    this.m_subsystem.stopMotor();
  }

  // This will stop the command
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
