// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: EveryBotPickerAutoUptakeCommand.java
// Intent: Forms a command to automatically uptake the cargo into the every bot picker.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EveryBotPickerSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class EveryBotPickerAutoUptakeCommand extends CommandBase {

    private EveryBotPickerSubsystem everyBotPickerSub;
    private Timer timer = new Timer();
    private boolean done = false;
    private final double uptakeDurationSeconds = 1.7;
    private final double uptakeRelativeSpeed = 1.0;
    private final double stoppedRelativeSpeed = 0.0;

    /**
     * Ccnstructor for EveryBotPickerAutoUptakeCommand
     * @param everyBotPickerSubsystem - the subsystem for the everybot picker
     * @param uptakeSupplier - trigger uptake value
     * @param expellSupplier - trigger expell value
     */
    public EveryBotPickerAutoUptakeCommand(EveryBotPickerSubsystem everyBotPickerSubsystem) {
        this.everyBotPickerSub = everyBotPickerSubsystem;
        addRequirements(this.everyBotPickerSub);
    }

    /**
     * Override of the initialize method on CommandBase - called on start of command
     */
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        done = false;
    }

    /**
     * Override of the execute method on CommandBase - Called every time the scheduler runs while the command is scheduled.
     */
    @Override
    public void execute() {
        if (timer.hasElapsed(this.uptakeDurationSeconds)) {
            everyBotPickerSub.setPickerRelativeSpeed(this.stoppedRelativeSpeed);
            done = true;
        }
        else {
            everyBotPickerSub.setPickerRelativeSpeed(this.uptakeRelativeSpeed);
        }
    }   

    /**
     * Override of the end method on CommandBase - Called once the command ends or is interrupted.
     */
    @Override
    public void end(boolean interrupted) {
        everyBotPickerSub.setPickerRelativeSpeed(this.stoppedRelativeSpeed);
        if(interrupted) {
          done = true;      
        }
      }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return done;
    }
}