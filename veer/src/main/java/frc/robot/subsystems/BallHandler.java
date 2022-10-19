// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rapid React - 2022
// File: BallHandler.java
// Intent: Wrapper class standard stub for robot in FRC challange.
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

import static frc.robot.Constants.*;

public class BallHandler extends SubsystemBase implements Sendable {

    private WPI_TalonSRX ballMotor = new WPI_TalonSRX(Constants.BallHandlerMotorCanId);
    private boolean currentBallArmDeployed = false;

    /**
     * No argument constructor for the BallHandler subsystem.
    */
    public BallHandler() {
        // TODO - Naher do stuff here
    }

    /**
     * 
     */
    public void deployPosition(){
        // TODO - Naher do stuff here
    }

    public void retractPosition(){
        // TODO - Naher do stuff here
    }

    public void togglePosition(){
        // TODO - Naher do stuff here
    }

    public void storeBall(){
        // TODO - Naher do stuff here
    }

    public void retrieveBall(){
        // TODO - Naher do stuff here
    }

    public void stopStoringBall(){
        // TODO - Naher do stuff here
    }

    @Override
    public void initSendable(SendableBuilder builder)
    {
      builder.addDoubleProperty("BallMotorSpeed", this::getBallMotorSpeed, null);
      builder.addStringProperty("BallArmState", this::getBallArmState, null);
    }
 
    private String getBallArmState()
    {
        return (this.currentBallArmDeployed ? "Arm Deployed" : "Arm Retracted");
    }

    private double getBallMotorSpeed()
    {
        return this.ballMotor.get();
    }

    private void intitalizeBallArmState()
    {
        // TODO - Naher do stuff here
    }

}