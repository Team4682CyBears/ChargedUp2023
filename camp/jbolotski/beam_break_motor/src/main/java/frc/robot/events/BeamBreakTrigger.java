package frc.robot.events;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

/**
 * Creates a Trigger from a DigitalInput for a Beam Break sensor
 * returns true when the beam is unbroken
 */
public class BeamBreakTrigger extends Trigger{
    private DigitalInput beamBreakInput = new DigitalInput(Constants.beamBreakSensorPort); 

    /**
     * Constructor
     * @param channel Digital IO channel
     */
    public BeamBreakTrigger(){
        this.debounce(Constants.beamBreakDebounceTimeSeconds);
    }

    /**
     * @return returns true when beam is unbroken
     */
    @Override
    public boolean get(){
        return beamBreakInput.get();
    }

}
