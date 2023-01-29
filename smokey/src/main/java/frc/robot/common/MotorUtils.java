// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: MotorUtils.java
// Intent: Forms util class of methods that are commonly used on motor input/output.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.common;

import java.util.*;

public class MotorUtils
{

    /**
    * a method to validate speed input and throw if the speed value is invalid   
    *
    * @param  speed - target speed of the motor
    * @param  prependMessage - when speed is invalid, a string added to the front of the exception message
    * @param  appendMessage - when speed is invalid, a string added to the end of the exception message
    */
    public static void validateMotorSpeedInput(
        double speed,
        String prependMessage,
        String appendMessage)
    {
        if(speed > 1.0 || speed < -1.0)
        {
            throw new IllegalArgumentException(
                prependMessage == null ? "" : prependMessage +
                "input outside of acceptable motor speed range (valid range from -1.0 to 1.0)" +
                appendMessage == null ? "" : appendMessage);
        }        
    }
    
    /**
    * A method to make sure that values are retained within the boundaries 
    *
    * @param  value - target value 
    * @param  minBoundary - when value is below the minimum boundary the minBoundary is used as the returned value
    * @param  maxBoundary - when value is above the maximum boundary the maxBoundary is used as the returned value
    * @return the target or trimmed value
    */
    public static double truncateValue(
        double value,
        double minBoundary,
        double maxBoundary)
    {
        double trimmedValue = value;
        if(value < minBoundary)
        {
            trimmedValue = minBoundary;
        }
        else if (value > maxBoundary)
        {
            trimmedValue = maxBoundary;
        }
        return trimmedValue;
    }


    /**
     * Method to help provide debug info for measurement discontinuities
     * @param measurements array of measurements - must be minimul length of 1, should ideally give 4+ measurements
     * @param doDescriptivePrint
     * @return true when a measurement discontinuity was found, else false
     */
    public static boolean hasMeasurementDiscontinuity(
        ArrayList<Double> measurements,
        boolean doDescriptivePrint)
    {
        // dupe the inbound list - make sure we do deep copy
        ArrayList<Double> sortableList = new ArrayList<Double>();
        for(int inx = 0; inx < measurements.size(); ++inx)
        {
            sortableList.add((double)measurements.get(inx));
        }

        // sort the dup'd list
        Collections.sort(sortableList);

        // get the interquartile range measurements
        int firstQuartileIndex = measurements.size() / 4;
        int thirdQuartileIndex = measurements.size() * 3 / 4;
        double firstQuartile = measurements.get(firstQuartileIndex);
        double thirdQuartile = measurements.get(thirdQuartileIndex);
        double innerQuartileRange = thirdQuartile - firstQuartile;
        double lowerFence = firstQuartile - (1.5 * innerQuartileRange);
        double upperFence = thirdQuartile + (1.5 * innerQuartileRange);

        boolean foundDiscontinuity = false;

        // flow through original array looking for outliers
        for(int inx = 0; inx < measurements.size(); ++inx)
        {
            double nextMeasurement = measurements.get(inx);
            foundDiscontinuity &= (nextMeasurement < lowerFence || nextMeasurement > upperFence);
        }

        if(doDescriptivePrint)
        {
            System.out.println("DISCONTINUITY FOUND:");
            // flow through original array looking for outliers
            for(int inx = 0; inx < measurements.size(); ++inx)
            {
                double nextMeasurement = measurements.get(inx);
                if(nextMeasurement < lowerFence)
                {
                    System.out.println(nextMeasurement + " <- LOW OUTLIER");
                }
                else if(nextMeasurement > upperFence)
                {
                    System.out.println(nextMeasurement + " <- HIGH OUTLIER");
                }
                else
                {
                    System.out.println(nextMeasurement);
                }
            }
        }

        return foundDiscontinuity;
    }
}
