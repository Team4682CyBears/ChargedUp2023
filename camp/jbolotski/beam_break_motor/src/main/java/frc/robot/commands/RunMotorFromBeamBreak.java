// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.BeamBreakMotor;
import frc.robot.ManualInputInterfaces;
import frc.robot.common.Types.BeamBreakState;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** A motor driven by a beam break. */
public class RunMotorFromBeamBreak extends CommandBase {
  private final BeamBreakMotor motorSubsystem;
  private ManualInputInterfaces manualInputInterfaces;

  /**
   * Creates a new BeamBreakCommand
   *
   * @param subsystem The subsystem used by this command.
   */
  public RunMotorFromBeamBreak(BeamBreakMotor subsystem, ManualInputInterfaces inputs) {
    motorSubsystem = subsystem;
    manualInputInterfaces = inputs;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(manualInputInterfaces.getBeamBreakState()==BeamBreakState.unbroken){
      motorSubsystem.runMotor();
    } else {
      motorSubsystem.stopMotor();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    motorSubsystem.stopMotor();
  }

  // Returns true when the command should end.
  // This command should always run, so returns false
  @Override
  public boolean isFinished() {
    return false;
  }
}
