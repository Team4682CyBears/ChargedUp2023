// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.MotorGo;
import frc.robot.commands.MotorStop;
import frc.robot.sim.PhysicsSim;
import frc.robot.subsystems.BeamBreakMotor;
import frc.robot.events.BeamBreakTrigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems are defined here...
  private final BeamBreakMotor beamBreakMotor = new BeamBreakMotor(Constants.beamBreakMotorSpeed);
  private BeamBreakTrigger beamBreakTrigger = new BeamBreakTrigger();

  // The robot's commands are defined here...

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
  private void configureButtonBindings() {
    // so triggers supposedly do all the magic necessary to get the command to the scheduler
    // when the beam can be seen lets stop the motor
    beamBreakTrigger.whenActive(new MotorStop(beamBreakMotor));
    // when the beam is broken lets run the motor
    beamBreakTrigger.whenInactive(new MotorGo(beamBreakMotor));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new MotorStop(beamBreakMotor);
  }

  public void teleopInit(){
  }

  public void simulationInit(){
    PhysicsSim.getInstance().addTalonSRX(beamBreakMotor.getController(), 0.75, 5100, false);
  }

  public void simulationPeriodic() {
		PhysicsSim.getInstance().run();
	}
}
