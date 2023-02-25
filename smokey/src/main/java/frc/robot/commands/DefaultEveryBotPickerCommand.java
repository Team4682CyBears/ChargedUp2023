// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: DefaultEveryBotPickerCommand.java
// Intent: Forms a command to uptake/expell the cargo from the every bot picker.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.EveryBotPickerSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultEveryBotPickerCommand extends CommandBase {

    private EveryBotPickerSubsystem everyBotPickerSub;
    private DoubleSupplier uptakeSupplier;
    private DoubleSupplier expellSupplier;
    private final double inputThreshold = 0.1;

    /**
     * Ccnstructor for DefaultEveryBotPickerCommand
     * @param everyBotPickerSubsystem - the subsystem for the everybot picker
     * @param uptakeSupplier - trigger uptake value
     * @param expellSupplier - trigger expell value
     */
    public DefaultEveryBotPickerCommand(EveryBotPickerSubsystem everyBotPickerSubsystem,
                               DoubleSupplier uptakeInputSupplier,
                               DoubleSupplier expellInputSupplier) {
        this.everyBotPickerSub = everyBotPickerSubsystem;
        this.uptakeSupplier = uptakeInputSupplier;
        this.expellSupplier = expellInputSupplier;

        addRequirements(this.everyBotPickerSub);
    }

    @Override
    public void execute() {
        double uptakeValue = this.uptakeSupplier.getAsDouble();
        double expellValue = this.expellSupplier.getAsDouble();
        double uptakeAbsValue = Math.abs(uptakeValue);
        double expellAbsValue = Math.abs(expellValue);
        double inputValue = (uptakeAbsValue > expellAbsValue ? uptakeValue : expellValue);
        if(uptakeAbsValue <= this.inputThreshold && uptakeAbsValue <= this.inputThreshold){
            inputValue = 0.0;
        }
        this.everyBotPickerSub.setPickerSpeed(inputValue);
    }

    @Override
    public void end(boolean interrupted) {
        this.everyBotPickerSub.setPickerSpeed(0.0);
    }
}