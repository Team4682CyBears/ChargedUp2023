// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: AbsoluteEncoderCollection.java
// Intent: Singleton pattern for encoder collection permitting access from anywhere in codebase.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.swerveHelpers;

import java.util.HashMap;
import java.util.Map;

import javax.management.InvalidAttributeValueException;

public class AbsoluteEncoderCollection {

    private static AbsoluteEncoderCollection instance = new AbsoluteEncoderCollection();
    private Map <Integer, AbsoluteEncoder> encoderMap = new HashMap<Integer, AbsoluteEncoder>();

    /** Private CTOR forms basis of singleton pattern */
    private AbsoluteEncoderCollection(){
    }

    /**
     * Get at instance of this singleton
     * @return the single instance of the AbsoluteEncoderCollection for the whole robot
     */
    public static AbsoluteEncoderCollection getInstance() { return instance; }
    
    /**
     * Method to add an encoder
     * @param canId - the id of the can bus for the absolute encoder
     * @param encoder - the actual encoder
     */
    public void addAbsoluteEncoder(int canId, AbsoluteEncoder encoder) {
        if(encoder != null) {
            Integer encoderId = canId;
            if(encoderMap.containsKey(encoderId)){
                encoderMap.remove(encoderId);
                System.out.println("WARNING: overwriting absolute encoder at can id == " + canId);
            }
            encoderMap.put(encoderId, encoder);
        }
    }

    /**
     * How many encoders have materalized in the code
     * @return count of absolute encoders
     */
    public int size(){
        return encoderMap.size();
    }

    /**
     * A method that will determine if all encoders are synced
     * @return true if all encoders have been synced, else false
     */
    public boolean getAllEncodersEverSynched() {
        boolean allEncodersEverSet = (this.size() > 0);
        for(Map.Entry<Integer, AbsoluteEncoder> set: encoderMap.entrySet()){
            allEncodersEverSet &= set.getValue().getHasEncoderEverSynched();
        }
        return allEncodersEverSet;
    }

    /**
     * A method to obtain if the can id based encoder was actually set
     * @param canId the canid to confirm was set
     * @return true if the encoder is set, else false
     * @throws InvalidAttributeValueException
     */
    public boolean getEncoderAtCanIdEverSynched(int canId) throws InvalidAttributeValueException {
        boolean encoderEverSet = false;
        Integer encoderId = canId;
        if(encoderMap.containsKey(encoderId)){
            encoderEverSet = encoderMap.get(encoderId).getHasEncoderEverSynched();
        }
        else {
            throw new InvalidAttributeValueException("invalid canid requested of " + canId);
        }
        return encoderEverSet;
    }


}
