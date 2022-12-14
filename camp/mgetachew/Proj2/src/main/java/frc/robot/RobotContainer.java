// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.Forward;
import frc.robot.commands.Backward;
import frc.robot.commands.Stop;
import frc.robot.subsystems.MatthewsCoolMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final MatthewsCoolMotor m_romiDrivetrain = new MatthewsCoolMotor();

  private final Forward forwardCmd = new Forward(m_romiDrivetrain);
  private final Backward backwardCmd = new Backward(m_romiDrivetrain);
  private final Stop stopCmd = new Stop(m_romiDrivetrain);
  private Joystick highLevelButtonBoard = new Joystick(Constants.highLevelButtonBoardPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton motorForward = new JoystickButton(highLevelButtonBoard, Constants.lowLevelButtonBoardPort);
    JoystickButton motorBackward = new JoystickButton(highLevelButtonBoard, Constants.highLevelButtonBoardPort);

    motorForward.whileHeld(forwardCmd);
    motorForward.whenReleased(stopCmd);
    motorBackward.whileHeld(backwardCmd);
    motorBackward.whenReleased(stopCmd);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // will run in autonomous
    return null;
  }
}
