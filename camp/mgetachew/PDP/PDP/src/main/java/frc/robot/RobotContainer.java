// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.control.PortSpy;
import frc.robot.subsystems.PowerDistributionPanelWatcher;
import frc.robot.subsystems.SmartDash;
import frc.robot.subsystems.TheRevMoter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.Constants;

import javax.swing.plaf.synth.Region;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SmartDash m_exampleSubsystem = new SmartDash();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  public TheRevMoter rightMotor = new TheRevMoter();
  private PowerDistributionPanelWatcher pdpWatcher = new PowerDistributionPanelWatcher(ModuleType.kCTRE);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    pdpWatcher.add(new PortSpy(3, Constants.motorcurrentlimit, new InstantCommand(rightMotor::stopMotor, rightMotor)));
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
    return m_autoCommand;
  }
}
