// this is a subsystem for a ultrasonic sensor

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;

public class UltrasonicSensor extends SubsystemBase {

    private static final AnalogInput ultrasonic = new AnalogInput(0);

    public UltrasonicSensor(){}

    public static double getRawValue() {
        double rawValue = ultrasonic.getValue();
        return rawValue;
    }

    public static double getValueInInches(){
        double UltrasonicRawValue = getRawValue();
        double currentDistanceCentimeters = UltrasonicRawValue * 0.125 * 0.39;
        return currentDistanceCentimeters;
    }
}