// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SetMotorSpeed;
import frc.robot.commands.StopMotor;
import frc.robot.subsystems.EpicTalonMotor;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private XboxController mainController = new XboxController(Constants.PortMainController); 

  // The robot's subsystems and commands are defined here...
  private final EpicTalonMotor talonMotorSub = new EpicTalonMotor();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }


    /** This function is called periodically during operator control. */
    public void teleopPeriodic() {
      //if(mainController.getLeftY() * -1.0 >= Constants.XBoxStickTolerence * -1 && mainController.getLeftY() * -1.0 <= Constants.XBoxStickTolerence){
        //new ScheduleCommand(new StopMotor(talonMotorSub));
      //}
      CommandScheduler.getInstance().schedule(new SetMotorSpeed(talonMotorSub, mainController.getLeftY() * -1.0));
    }
  
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //talonMotorSub.setDefaultCommand(new SetMotorSpeed(talonMotorSub, mainController.getLeftY() * -1.0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}
