package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EpicTalonMotor;

/**
 * This class is a command for setting the motor speed
 */
public class SetMotorSpeed extends CommandBase{
    private EpicTalonMotor epicTalonMotorSubsytem;
    private double mySpeed = 0.0;
    private boolean done = false;

    /**
     * This constructor takes in a motor subsystem and an incomming speed value
     * @param epicTalonMotorSubsystem The motor subsystem
     * @param speed the speed input between -1 and 1
     */
    public SetMotorSpeed(EpicTalonMotor epicTalonMotorSubsystem, double speed) {
        this.epicTalonMotorSubsytem = epicTalonMotorSubsystem;
        this.mySpeed = speed;
        addRequirements(epicTalonMotorSubsystem);}
    
    /**
     * Initialization method
     */
    public void initialize(){
        done = false;}

    /**
     * sets speed on motor
     */
    public void execute() {
        epicTalonMotorSubsytem.SetSpeed(mySpeed);
        done = true;
    }

    /**
     * end method
     */
    public void end(){
        done = true;}
        
    /**
     * isFinished method: always returns done
     */
    public boolean isFinished(){
        return done;
    }

    
}
