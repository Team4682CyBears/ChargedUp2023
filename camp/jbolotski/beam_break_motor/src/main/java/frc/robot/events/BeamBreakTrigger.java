package frc.robot.events;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

/**
 * Creates a Trigger from a DigitalInput for a Beam Break sensor
 * returns true when the beam is unbroken
 */
public class BeamBreakTrigger extends Trigger{

    private static Trigger localInstance = (new BeamBreakTrigger()).debounce(Constants.beamBreakDebounceTimeSeconds, DebounceType.kBoth);
    private DigitalInput beamBreakInput = new DigitalInput(Constants.beamBreakSensorPort); 

    public static Trigger getInstance()
    {
        return localInstance;
    }

    /**
     * default ctor - private to ensure singelton pattern
     */
    private BeamBreakTrigger(){
    }

    /**
     * @return returns true when beam is unbroken
     */
    @Override
    public boolean get(){
        return beamBreakInput.get();
    }

}
