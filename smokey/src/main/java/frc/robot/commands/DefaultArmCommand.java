// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: DefaultArmCommand.java
// Intent: Forms a command to move the dual part arm with y and z inputs.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultArmCommand extends CommandBase {
    private final ArmSubsystem armSubsystem;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier zSupplier;

    public DefaultArmCommand(ArmSubsystem theArmSubsystem,
                             DoubleSupplier yMotionSupplier,
                             DoubleSupplier zMotionSupplier) {
        this.armSubsystem = theArmSubsystem;
        this.ySupplier = yMotionSupplier;
        this.zSupplier = zMotionSupplier;
        addRequirements(this.armSubsystem);
    }

    @Override
    public void execute() {
        this.armSubsystem.setArmSpeeds(ySupplier.getAsDouble(), zSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        this.armSubsystem.setArmSpeeds(0.0, 0.0);
    }
}