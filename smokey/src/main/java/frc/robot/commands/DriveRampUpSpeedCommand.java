// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: DriveRampUpSpeedCommand.java
// Intent: Forms a command to ramp up the drivetrain subsystem speed until it reaches maximum.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class DriveRampUpSpeedCommand extends CommandBase {

    private DrivetrainSubsystem drivetrainSub;
    private Timer timer = new Timer();
    private final double powerIncrementDelaySeconds = 0.5;


    public DriveRampUpSpeedCommand(DrivetrainSubsystem drivetrainSub) {
        this.drivetrainSub = drivetrainSub;
        // DO NOT declare drivetrainSub as requirement.  
        // We need default drive commands to keep working while this command runs
    }

    /**
     * Override of the initialize method on CommandBase - called on start of command
     */
    @Override
    public void initialize() {
        drivetrainSub.incrementPowerReductionFactor();
        timer.reset();
        timer.start();
    }

    /**
     * Override of the execute method on CommandBase - Called every time the scheduler runs while the command is scheduled.
     */
    @Override
    public void execute() {
        if (timer.hasElapsed(this.powerIncrementDelaySeconds)) {
            drivetrainSub.incrementPowerReductionFactor();
            timer.reset();
            timer.start();
        }
    }   

    /**
     * Override of the end method on CommandBase - Called once the command ends or is interrupted.
     */
    @Override
    public void end(boolean interrupted) {
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return drivetrainSub.isMaxPowerReductionFactor();
    }
}