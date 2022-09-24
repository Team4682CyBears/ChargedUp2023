package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.common.Types.BeamBreakState;

/**
 * Creates a Trigger from a DigitalInput for a Beam Break sensor
 * returns true when the beam is unbroken
 */
public class BeamBreakSensor extends Trigger{
    private DigitalInput beamBreakInput = null;

    /**
     * Constructor
     * @param channel Digital IO channel
     */
    public BeamBreakSensor(int channel){
        // get a new DigitalInput on the channel. 
        beamBreakInput = new DigitalInput(channel); 
    }

    /**
     * @return returns true when beam is unbroken
     */
    @Override
    public boolean get(){
        return this.getState()==BeamBreakState.unbroken;
    }

    /**
     * a method exposed to callers to return the beam break state as broken or unbroken
     * use this instead of underlying DigitalInput.get method which returns true/false
     * @return BeamBreakState 
     */
    public BeamBreakState getState(){
        //TODO not sure about the polarity of the beam break
        if(beamBreakInput.get()){
            return BeamBreakState.unbroken;
        } else{
            return BeamBreakState.broken;
        }
    }

    /**
     * Runs the given runnable whenever the beam just becomes broken.
     * @param toRun - the runnable to run
     * @param requirements - the required subsystems
     */
    public void whenBroken(Runnable toRun, Subsystem... requirements){
        this.debounce(Constants.beamBreakDebounceTimeSeconds,DebounceType.kBoth).whenInactive(toRun, requirements);
    }

    /**
     * Runs the given runnable whenever the beam just becomes unbroken.
     * @param toRun - the runnable to run
     * @param requirements - the required subsystems
     */
    public void whenUnbroken(Runnable toRun, Subsystem... requirements){
        this.debounce(Constants.beamBreakDebounceTimeSeconds,DebounceType.kBoth).whenActive(toRun, requirements);
    }
}
