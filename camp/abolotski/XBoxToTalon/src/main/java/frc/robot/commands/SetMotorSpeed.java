package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EpicTalonMotor;


public class SetMotorSpeed extends CommandBase{
    private EpicTalonMotor epicTalonMotorSubsytem;
    private double mySpeed = 0.0;
    private boolean done = false;

    public SetMotorSpeed(EpicTalonMotor epicTalonMotorSubsystem, double speed) {
        this.epicTalonMotorSubsytem = epicTalonMotorSubsystem;
        this.mySpeed = speed;
        addRequirements(epicTalonMotorSubsystem);}

    public void initialize(){
        System.out.println("in command initialization");
        done = false;}

    public void execute() {
        System.out.println("in command execution" + mySpeed);
        epicTalonMotorSubsytem.SetSpeed(mySpeed);
        done = true;
    }

    public void end(){
        done = true;}
        
    public boolean isFinished(){
        return done;
    }

    
}
