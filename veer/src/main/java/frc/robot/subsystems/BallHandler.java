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
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * TODO - Naher - put comments in here correctly
 */
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
     * This deploys the arm into a deployed (downward) position
     */
    public void deployPosition(){
        this.solenoid.set(DoubleSolenoid.Value.kForward);
        currentBallArmDeployed = true;
    }

    /**
     * // TODO - Naher - put comments in here correctly
     * This lifts the arm into a retracted (upward) position
     */
    public void retractPosition(){
        this.solenoid.set(DoubleSolenoid.Value.kReverse);
        currentBallArmDeployed = false;
    }

    /**
     * // TODO - Naher - put comments in here correctly
     * This toggles the arm position
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
     * // TODO - Naher - put comments in here correctly
     * This stores balls
     */
    public void storeBall(){
        // TODO - Naher do stuff here
        this.ballMotor.set(ControlMode.PercentOutput, Constants.BallHandlerMotorDefaultSpeed);
    
    }

    /**
     * // TODO - Naher - put comments in here correctly
     * This sucks in a ball
     */
    public void retrieveBall(){
        // TODO - Naher do stuff here
        this.ballMotor.set(ControlMode.PercentOutput, Constants.BallHandlerMotorDefaultSpeed * -1.0);
    }

    /**
     * // TODO - Naher - put comments in here correctly
     * This stops the motor after storing balls
     */
    public void stopStoringBall(){
        // TODO - Naher do stuff here
        this.ballMotor.set(0.0);
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