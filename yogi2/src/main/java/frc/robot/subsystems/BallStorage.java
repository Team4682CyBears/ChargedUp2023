// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rapid React - 2022
// File: ClimberS1.java
// Intent: Forms a subsystem that controls movements by the Jaws.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.common.*;

public class BallStorage extends SubsystemBase implements Sendable
{
  private WPI_TalonSRX bottomMotor = new WPI_TalonSRX(Constants.ballStorageMotorBottomCanId);
  private WPI_TalonSRX topMotor = new WPI_TalonSRX(Constants.ballStorageMotorTopCanId);
 
  /**
  * No argument constructor for the BallStorage subsystem.
  */
  public BallStorage()
  {  
    topMotor.configFactoryDefault();
    topMotor.setNeutralMode(NeutralMode.Brake);
    topMotor.setInverted(true);

    bottomMotor.configFactoryDefault();
    bottomMotor.setNeutralMode(NeutralMode.Brake);
    bottomMotor.setInverted(false);
    CommandScheduler.getInstance().registerSubsystem(this);
  }

  @Override
  public void periodic()
  {
  }

  @Override
  public void initSendable(SendableBuilder builder)
  {
    builder.addDoubleProperty("TopMotorSpeedSetting", this::getTopMotorSpeed, null);
    builder.addDoubleProperty("BottomMotorSpeedSetting", this::getBottomMotorSpeed, null);
    builder.addStringProperty("BallStorageDirection", this::getBallStorageDirection, null);
  }

  /**
  * A method exposed to callers to retrieve one ball from storage
  */
  public void retrieve()
  {
    bottomMotor.set(ControlMode.PercentOutput, Constants.ballRetrieveSpeed);
    topMotor.set(ControlMode.PercentOutput, Constants.ballRetrieveSpeed);
  }

  /**
  * A method exposed to callers to store one ball into storage
  */
  public void store()
  {
    // since motors are followers ok to just set one
    bottomMotor.set(ControlMode.PercentOutput, Constants.ballStoreSpeed);
    topMotor.set(ControlMode.PercentOutput, Constants.ballStoreSpeed);
  }

  @Override
  public void setDefaultCommand(Command myCommand)
  {
      // TODO Auto-generated method stub
      super.setDefaultCommand(myCommand);
  }

  /**
  * A method exposed to callers to cause motors to drive belts in storage direction
  */
  public void storeBallManual()
  {
      // since motors are followers ok to just set one
      bottomMotor.set(ControlMode.PercentOutput, Constants.ballStoreSpeed);
      topMotor.set(ControlMode.PercentOutput, Constants.ballStoreSpeed);
  }

  /**
  * A method exposed to callers to cause motors to stop
  */
  public void stopBallManual()
  {
      // since motors are followers ok to just set one
      bottomMotor.set(ControlMode.PercentOutput, 0.0);
      topMotor.set(ControlMode.PercentOutput, 0.0);
  }

  /**
  * A method exposed to callers to cause motors to drive belts in retrieve direction
  */
  public void retrieveBallManual()
  {
      bottomMotor.set(ControlMode.PercentOutput, Constants.ballRetrieveSpeed);
      topMotor.set(ControlMode.PercentOutput, Constants.ballRetrieveSpeed);
  }

  /**
   * Gets the top motor speed setting
   * @return the top motor controller output as decmil fraction
   */
  private double getTopMotorSpeed()
  {
    return topMotor.getMotorOutputPercent() / 100.0;
  }

  /**
   * Gets the bottom motor speed setting
   * @return the bottom motor controller output as decmil fraction
   */
  private double getBottomMotorSpeed()
  {
    return bottomMotor.getMotorOutputPercent() / 100.0;
  }

  /**
   * To get a string that represents what direction the ball is moving in
   * @return a string representing the ball storage movement direction
   */
  private String getBallStorageDirection()
  {
    double topMotorSpeed = this.getTopMotorSpeed();
    double bottomMotorSpeed = this.getBottomMotorSpeed();
    if(topMotorSpeed == 0.0 && bottomMotorSpeed == 0.0)
    {
      return "Stopped";
    }
    else if((Constants.ballRetrieveSpeed > 0.0 && topMotorSpeed > 0.0) || 
      (Constants.ballRetrieveSpeed < 0.0 && topMotorSpeed < 0.0))
    {
      return "Retrieving";
    }
    else if((Constants.ballStoreSpeed > 0.0 && topMotorSpeed > 0.0) || 
      (Constants.ballStoreSpeed < 0.0 && topMotorSpeed < 0.0))
    {
      return "Storing";
    }
    else
    {
      if(topMotorSpeed > 0.1 && bottomMotorSpeed > 0.1 ||
        topMotorSpeed < -0.1 && bottomMotorSpeed < -0.1)
      {
        return "VERY INVALID";
      }
      else
      {
        return "INVALID";
      }
    }
  }
}
