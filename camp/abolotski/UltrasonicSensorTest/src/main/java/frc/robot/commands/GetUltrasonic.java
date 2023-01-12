
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.UltrasonicSensor;
import edu.wpi.first.wpilibj.AnalogInput;

public class GetUltrasonic extends CommandBase{
    private UltrasonicSensor UltrasonicSensorSubsystem;
    private boolean done = false;

    public void initialize(){
        done = false;
    }

    public void execute(){
        done = true;
    }
    
    public void end(){
        done = true;
    }

    public boolean isFinished(){
        return done;
    }
    


    
}
