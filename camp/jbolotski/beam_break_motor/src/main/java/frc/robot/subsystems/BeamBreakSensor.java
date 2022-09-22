package frc.robot.subsystems;

import frc.robot.common.Types.BeamBreakState;

import edu.wpi.first.wpilibj.DigitalInput;

public class BeamBreakSensor extends DigitalInput{

    /**
     * Constructor
     * uses default DigitlInput behavior
     */
    public BeamBreakSensor(int channel){
        // pass the channel to the superclass constructor
        super(channel);
    }

    /**
     * a method exposed to callers to return the beam break state as broken or unbroken
     * use this instead of underlying DigitalInput.get method which returns true/false
     * @return BeamBreakState 
     */
    public BeamBreakState getState(){
        //TODO not sure about the polarity of the beam break
        if(super.get()){
            return BeamBreakState.unbroken;
        } else{
            return BeamBreakState.broken;
        }
    }
}
