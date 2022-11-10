package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EpicTalonMotor;

public class StopMotor extends CommandBase{
    private EpicTalonMotor epicTalonMotorSubsytem;
    private boolean done = false;

    public StopMotor(EpicTalonMotor epicTalonMotorSubsystem) {
        this.epicTalonMotorSubsytem = epicTalonMotorSubsystem;
        addRequirements(epicTalonMotorSubsystem);
    }

    public void initialize(){
        done = false;
    }

    public void execute(){
        epicTalonMotorSubsytem.StopMotor();
        done = true;
    }
    
    public void end(){
        done = true;
    }

    public boolean isFinished(){
        return done;
    }
    


    
}
