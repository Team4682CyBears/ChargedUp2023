package frc.robot.subsystems;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

public class SmartDash implements Sendable {
    private double input;

    public SmartDash() {
        input = 0.0;
        SendableRegistry.add(this, "SmartDash");
    }

    public void setInput(double input) {
        this.input = input;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Input", () -> input, this::setInput);
    }
}
