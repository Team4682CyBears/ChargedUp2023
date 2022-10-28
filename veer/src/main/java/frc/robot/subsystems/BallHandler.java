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
import frc.robot.Constants;
import frc.robot.common.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A class intended to model the BallHandler infrastructure on Veer.
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
     * Method to move the arm into a deployed (upward) position
     */
    public void deployPosition(){
        this.solenoid.set(DoubleSolenoid.Value.kForward);
        currentBallArmDeployed = true;
    }

    /**
     * Method to move the arm into a retracted (downward) position
     */
    public void retractPosition(){
        this.solenoid.set(DoubleSolenoid.Value.kReverse);
        currentBallArmDeployed = false;
    }

    /**
     * Method to toggle the arm position from its current location.
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
     * Method to store a ball.
     */
    public void storeBall(){
        this.ballMotor.set(
            ControlMode.PercentOutput,
            Constants.BallHandlerMotorDefaultSpeed * Constants.BallHandlerMotorStorageDirectionMultiplier);
    }

    /**
     * Method to retrieve the ball.
     */
    public void retrieveBall(){
        this.ballMotor.set(
            ControlMode.PercentOutput,
            Constants.BallHandlerMotorDefaultSpeed * Constants.BallHandlerMotorRetrievalDirectionMultiplier);
    }

    /**
     * Method to stop the ball handling motor.
     */
    public void stopStoringBall(){
        this.ballMotor.set(0.0);
    }

    /**
     * Method to set the ball motor speed to a certain value.
     * @param speed - Values between -1.0 and 1.0, all others will be truncated to be within these bounds. 
     */
    public void setBallMotor(double speed){
        double trimmedSpeed = MotorUtils.truncateValue(speed, -1.0, 1.0);
        this.ballMotor.set(
            ControlMode.PercentOutput,
            trimmedSpeed);
    }

    /**
     * Used to send telemetry to shuffleboard or similar
     * @param builder - the sendable builder to insert telemetry content into
     */
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
        this.stopStoringBall();

        // confirm that the double solenoid has retracted the arm
        this.retractPosition();
    }

}