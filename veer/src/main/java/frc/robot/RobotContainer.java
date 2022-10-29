// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DriveTimeCommand;
import frc.robot.control.InstalledHardware;
import frc.robot.control.ManualInputInterfaces;
import frc.robot.control.SubsystemCollection;
import frc.robot.subsystems.BallHandler;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.TelescopingArm;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private SubsystemCollection subsystems = new SubsystemCollection();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // init the input system 
    this.initializeManualInputInterfaces();

    // init the various subsystems
    this.initializeBallHandler();
    this.initializeDrivetrainSubsystem();
    this.initializeTelescopingArm();

    // Configure the button bindings
    this.subsystems.getManualInputInterfaces().initializeButtonCommandBindings();
  }

  private void initializeManualInputInterfaces()
  {
    if(InstalledHardware.coDriverXboxControllerInstalled &&
      InstalledHardware.driverXboxControllerInstalled)
    {
      subsystems.setManualInputInterfaces(new ManualInputInterfaces(subsystems));
      System.out.println("SUCCESS: initializeManualInputInterfaces");
    }
    else
    {
      System.out.println("FAIL: initializeManualInputInterfaces");
    }
  }

  private void initializeBallHandler()
  {
    if(InstalledHardware.ballHandlingMotorInstalled && 
      InstalledHardware.ballHandlingPneumaticsInstalled && 
      InstalledHardware.compressorInstalled)
    {
      subsystems.setBallHandlerSubsystem(new BallHandler());
      System.out.println("SUCCESS: initializeBallHandler");

      subsystems.getBallHandlerSubsystem().setDefaultCommand(
        new RunCommand(
          () ->
          subsystems.getBallHandlerSubsystem().setBallMotor(
            subsystems.getManualInputInterfaces().getBallMotorSpeed()
            ),
            subsystems.getBallHandlerSubsystem()));
    }
    else
    {
      System.out.println("FAIL: initializeBallHandler");
    }
  }

  private void initializeDrivetrainSubsystem()
  {
    if(InstalledHardware.leftFrontDriveInstalled && 
      InstalledHardware.leftRearDriveInstalled && 
      InstalledHardware.rightFrontDriveInstalled &&
      InstalledHardware.rightRearDriveInstalled)
    {
      // The robot's subsystems and commands are defined here...
      subsystems.setDriveTrainSubsystem(new DrivetrainSubsystem());
      System.out.println("SUCCESS: initializeDrivetrain");

      // Set up the default command for the drivetrain.
      // The controls are for field-oriented driving:
      // Left stick Y axis -> forward and backwards movement
      // Left stick X axis -> left and right movement
      // Right stick X axis -> rotation
      subsystems.getDriveTrainSubsystem().setDefaultCommand(new DefaultDriveCommand(
        subsystems.getDriveTrainSubsystem(),
        () -> -modifyAxis(subsystems.getManualInputInterfaces().getInputArcadeDriveY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(subsystems.getManualInputInterfaces().getInputArcadeDriveX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(subsystems.getManualInputInterfaces().getInputSpinDriveX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
      ));
    }
    else
    {
      System.out.println("FAIL: initializeDrivetrain");
    }
  }

  private void initializeTelescopingArm()
  {
    if(InstalledHardware.telescopingArmsDriveMotorInstalled)
    {
      subsystems.setTelescopingArmSubsystem(new TelescopingArm());
      System.out.println("SUCCESS: initializeTelescopingArm");
    }
    else
    {
      System.out.println("FAIL: initializeTelescopingArm");
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return new InstantCommand();
    // TODO robot-centric drive command.  
    return new DriveTimeCommand(subsystems.getDriveTrainSubsystem(), 0.4, 0.0, 0.0, 10.0);
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}