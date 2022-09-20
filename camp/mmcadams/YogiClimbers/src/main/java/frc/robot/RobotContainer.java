// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.TelescopingArms;
import frc.robot.common.*;
import frc.robot.commands.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final TelescopingArms armsSubsystem = new TelescopingArms();
  private Joystick highLevelButtonBoard = new Joystick(Constants.highLevelButtonBoardPort);
  private Joystick lowLevelButtonBoard = new Joystick(Constants.lowLevelButtonBoardPort);

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


    JoystickButton telescopingArmLeftManualExtend = new JoystickButton(highLevelButtonBoard, 1);
    JoystickButton telescopingArmLeftManualRetract = new JoystickButton(highLevelButtonBoard, 2);
    JoystickButton telescopingArmRightManualExtend = new JoystickButton(highLevelButtonBoard, 3);
    JoystickButton telescopingArmRightManualRetract = new JoystickButton(highLevelButtonBoard, 4);

    telescopingArmLeftManualExtend.whileHeld(
      new ParallelCommandGroup(
        new TelescopingArmsSingleManual(
          armsSubsystem,
          TelescopingArmSide.Left,
          Constants.telescopingArmsDefaultExtendSpeed),
        new ButtonPress("buttonBoardHigh.1", "telescopingArmLeftManualExtend.whileHeld")));
    telescopingArmLeftManualExtend.whenReleased(
      new ParallelCommandGroup(
        new TelescopingArmsSingleManual(
          armsSubsystem,
          TelescopingArmSide.Left,
          Constants.telescopingArmsStopSpeed),
        new ButtonPress("buttonBoardHigh.1", "telescopingArmLeftManualExtend.whenReleased")));
    telescopingArmLeftManualRetract.whileHeld(
      new ParallelCommandGroup(
        new TelescopingArmsSingleManual(
          armsSubsystem,
          TelescopingArmSide.Left,
          Constants.telescopingArmsDefaultRetractSpeed),
        new ButtonPress("buttonBoardHigh.2", "telescopingArmLeftManualRetract.whileHeld")));
    telescopingArmLeftManualRetract.whenReleased(
      new ParallelCommandGroup(
        new TelescopingArmsSingleManual(
          armsSubsystem,
          TelescopingArmSide.Left,
          Constants.telescopingArmsStopSpeed),
        new ButtonPress("buttonBoardHigh.2", "telescopingArmLeftManualRetract.whenReleased")));
    telescopingArmRightManualExtend.whileHeld(
      new ParallelCommandGroup(
        new TelescopingArmsSingleManual(
          armsSubsystem,
          TelescopingArmSide.Right,
          Constants.telescopingArmsDefaultExtendSpeed),
        new ButtonPress("buttonBoardHigh.3", "telescopingArmRightManualExtend.whileHeld")));
    telescopingArmRightManualExtend.whenReleased(
      new ParallelCommandGroup(
        new TelescopingArmsSingleManual(
          armsSubsystem,
          TelescopingArmSide.Right,
          Constants.telescopingArmsStopSpeed),
        new ButtonPress("buttonBoardHigh.3", "telescopingArmRightManualExtend.whenReleased")));
    telescopingArmRightManualRetract.whileHeld(
      new ParallelCommandGroup(
        new TelescopingArmsSingleManual(
          armsSubsystem,
          TelescopingArmSide.Right,
          Constants.telescopingArmsDefaultRetractSpeed),
        new ButtonPress("buttonBoardHigh.4", "telescopingArmRightManualRetract.whileHeld")));
    telescopingArmRightManualRetract.whenReleased(
      new ParallelCommandGroup(
        new TelescopingArmsSingleManual(
          armsSubsystem,
          TelescopingArmSide.Right,
          Constants.telescopingArmsStopSpeed),
        new ButtonPress("buttonBoardHigh.4", "telescopingArmRightManualRetract.whenReleased")));


    JoystickButton telescopingArmsUp = new JoystickButton(lowLevelButtonBoard, 3);
    JoystickButton telescopingArmsDown = new JoystickButton(lowLevelButtonBoard, 4);

    telescopingArmsUp.whileHeld(
        new ParallelCommandGroup(
          new TelescopingArmsManual(armsSubsystem, Constants.telescopingArmsDefaultExtendSpeed),
          new ButtonPress("buttonBoardLow.3", "3.whileHeld")));
      telescopingArmsDown.whileHeld(
        new ParallelCommandGroup(
          new TelescopingArmsManual(armsSubsystem, Constants.telescopingArmsDefaultRetractSpeed),
          new ButtonPress("buttonBoardLow.4", "4.whileHeld")));
      telescopingArmsUp.whenReleased(
        new ParallelCommandGroup(
          new TelescopingArmsManual(armsSubsystem, Constants.telescopingArmsStopSpeed),
          new ButtonPress("buttonBoardLow.3", "3.whenReleased")));
      telescopingArmsDown.whenReleased(
        new ParallelCommandGroup(
          new TelescopingArmsManual(armsSubsystem, Constants.telescopingArmsStopSpeed),
          new ButtonPress("buttonBoardLow.4", "4.whenReleased")));
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    TelescopingArmExtendHigh high = new TelescopingArmExtendHigh(armsSubsystem);
    TelescopingArmsRetract retract = new TelescopingArmsRetract(armsSubsystem);
    SequentialCommandGroup group = new SequentialCommandGroup(
      high.withTimeout(Constants.maximumTelescopingArmsExtendTimeOperationSeconds),
      retract.withTimeout(Constants.maximumTelescopingArmsRetractTimeOperationSeconds));
    return group;
  }
}
