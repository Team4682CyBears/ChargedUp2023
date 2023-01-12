// this is a subsystem for a ultrasonic sensor

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;

public class UltrasonicSensor extends SubsystemBase {

    private final AnalogInput ultrasonic = new AnalogInput(0);

    public UltrasonicSensor(){}

    public double getRawValue() {
        double rawValue = ultrasonic.getValue();
        return rawValue;
    }
}