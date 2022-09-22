// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.RunMotorFromBeamBreak;
import frc.robot.sim.PhysicsSim;
import frc.robot.subsystems.BeamBreakMotor;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems are defined here...
  private final BeamBreakMotor m_beamBreakMotor = new BeamBreakMotor(Constants.beamBreakMotorSpeed);
  private final ManualInputInterfaces m_manualInputs = new ManualInputInterfaces();

  // The robot's commands are defined here...
  private final RunMotorFromBeamBreak bbAutoCommand = new RunMotorFromBeamBreak(m_beamBreakMotor, m_manualInputs);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }

  public void teleopInit(){
    CommandScheduler.getInstance().schedule(bbAutoCommand);
  }

  public void simulationInit(){
    PhysicsSim.getInstance().addTalonSRX(m_beamBreakMotor.getController(), 0.75, 5100, false);
  }

  public void simulationPeriodic() {
		PhysicsSim.getInstance().run();
	}
}
