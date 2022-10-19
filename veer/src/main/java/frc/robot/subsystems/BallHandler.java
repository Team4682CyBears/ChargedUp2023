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
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;
import frc.robot.common.*;

import static frc.robot.Constants.*;

public class BallHandler extends SubsystemBase implements Sendable {

    private WPI_TalonSRX ballMotor = new WPI_TalonSRX(Constants.BallHandlerMotorCanId);
    private boolean currentBallArmDeployed = false;
    Compressor compressor = new Compressor(1, Constants.BallHandlerPneumaticsControlModuleType);
    DoubleSolenoid solenoid = new DoubleSolenoid(
        Constants.PneumaticsControlModuleNumber,
        Constants.BallHandlerPneumaticsControlModuleType,
        Constants.PneumaticsControlModuleForwardChannel,
        Constants.PneumaticsControlModuleReverseChannel);

    /**
     * No argument constructor for the BallHandler subsystem.
    */
    public BallHandler() {
        this.intitalizeBallHandlingState();
    }

    /**
     * Method to obtain if the ball arm is in the retracted position.
     * @return boolean true if ball arm is retracted, otherwise false.
     */
    public boolean isDeployed()
    {
        return currentBallArmDeployed;
    }

    /**
     * Method to obtain if the ball arm is in the retracted position.
     * @return boolean true if ball arm is retracted, otherwise false.
     */
    public boolean isRetracted()
    {
        return !currentBallArmDeployed;
    }

    /**
     * TODO - Naher - put comments in here correctly
     */
    public void deployPosition(){
        this.solenoid.set(DoubleSolenoid.Value.kForward);
        currentBallArmDeployed = true;
    }

    /**
     * TODO - Naher - put comments in here correctly
     */
    public void retractPosition(){
        this.solenoid.set(DoubleSolenoid.Value.kReverse);
        currentBallArmDeployed = false;
    }

    /**
     * TODO - Naher - put comments in here correctly
     */
    public void togglePosition(){
        if(currentBallArmDeployed)
        {
            this.retractPosition();
        }
        else
        {
            this.deployPosition();
        }
    }

    /**
     * TODO - Naher - put comments in here correctly
     */
    public void storeBall(){
        // TODO - Naher do stuff here
    }

    /**
     * TODO - Naher - put comments in here correctly
     */
    public void retrieveBall(){
        // TODO - Naher do stuff here
    }

    /**
     * TODO - Naher - put comments in here correctly
     */
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

    private void intitalizeBallHandlingState()
    {
        ballMotor.configFactoryDefault();
        ballMotor.setNeutralMode(NeutralMode.Coast);
        ballMotor.setInverted(Constants.BallHandlerMotorInvertedDirection);

        // confirm that the double solenoid has retracted the arm
        this.retractPosition();
    }

}