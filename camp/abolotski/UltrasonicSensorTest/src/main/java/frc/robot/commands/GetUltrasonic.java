
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.UltrasonicSensor;

public class GetUltrasonic extends CommandBase{
    private UltrasonicSensor ultrasonicSensorSubsystem;
    private boolean done = false;
    private double UltrasonicRawValue;

    double voltage_scale_factor = 1; //should be: double voltage_scale_factor = 5/RobotController.getVoltage5V();

    public GetUltrasonic(UltrasonicSensor ultrasonicSensorSubsystem) {
        this.ultrasonicSensorSubsystem = ultrasonicSensorSubsystem;
        addRequirements(ultrasonicSensorSubsystem);
    }

    public void initialize(){
        done = false;
    }

    public void execute(){
        UltrasonicRawValue = ultrasonicSensorSubsystem.getRawValue();
        double currentDistanceInches = UltrasonicRawValue * voltage_scale_factor * 0.125;
        System.out.println(currentDistanceInches);
        done = true;
    }
    
    public void end(){
        done = true;
    }

    public boolean isFinished(){
        return done;
    }
    


    
}
