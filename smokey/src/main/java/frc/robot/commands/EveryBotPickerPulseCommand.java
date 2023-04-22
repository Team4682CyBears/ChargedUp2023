// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: EveryBotPickerPulseCommand.java
// Intent: Forms a command to uptake/expell the cargo from the every bot picker.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.common.EveryBotPickerAction;
import frc.robot.common.MotorUtils;
import frc.robot.subsystems.EveryBotPickerSubsystem;

public class EveryBotPickerPulseCommand extends CommandBase {

    private EveryBotPickerSubsystem everyBotPickerSub;
    private double pickerPulseOnWidthSeconds;
    private double pickerPulseOffWidthSeconds;
    private double pickerPulseAmplitude;
    private double pickerPulseDurationSeconds;
    private EveryBotPickerAction pickerPulseAction = EveryBotPickerAction.ConeUptake;
    private boolean pulseOn = true;
    private Timer pulseTimer = new Timer();
    private Timer totalTimer = new Timer();
    private boolean done = false;

    /**
     * Ccnstructor for DefaultEveryBotPickerCommand
     * @param everyBotPickerSubsystem - the subsystem for the everybot picker
     * @param pulseWidth - the pulse width value 0.0 to 1.0
     * @param pulseAmplitude - the pulse amplitude 0.0 to 1.0
     * @param pulseDurationSeconds - the duration of the command in seconds
     * @param pulseAction - the target action
     */
    public EveryBotPickerPulseCommand(
        EveryBotPickerSubsystem everyBotPickerSubsystem,
        double pulseWidthOnSeconds,
        double pulseWidthOffSeconds,
        double pulseAmplitude,
        double pulseDurationSeconds,
        EveryBotPickerAction pulseAction) {
        this.everyBotPickerSub = everyBotPickerSubsystem;
        this.pickerPulseOnWidthSeconds = MotorUtils.truncateValue(pulseWidthOnSeconds, 0.0, 5.0);
        this.pickerPulseOffWidthSeconds = MotorUtils.truncateValue(pulseWidthOffSeconds, 0.0, 10.0);
        this.pickerPulseAmplitude = MotorUtils.truncateValue(pulseAmplitude, 0.0, 1.0);
        this.pickerPulseDurationSeconds = MotorUtils.truncateValue(pulseDurationSeconds, 0.0, 3600.0);
        this.pickerPulseAction = pulseAction;
        addRequirements(this.everyBotPickerSub);
    }

    /**
     * Init to clean state on cmd start
     */
    @Override
    public void initialize() {
        totalTimer.reset();
        totalTimer.start();

        pulseTimer.reset();
        pulseTimer.start();

        pulseOn = true;

        done = false;
    }

    /**
     * Execute the action
     */
    @Override
    public void execute() {
        if (this.totalTimer.hasElapsed(this.pickerPulseDurationSeconds)) {
            if(this.pulseOn) {
                if(this.pulseTimer.hasElapsed(this.pickerPulseOnWidthSeconds)) {
                    this.everyBotPickerSub.setPickerRelativeSpeed(0.0);
                    this.pulseOn = false;
                }
                else {
                    this.doPickerPusle();
                }
            }
            else {
                this.everyBotPickerSub.setPickerRelativeSpeed(0.0);
                if(this.pulseTimer.hasElapsed(this.pickerPulseOffWidthSeconds)) {
                    this.pulseOn = true;
                    this.doPickerPusle();
                }
            }
        }
        else {
            done = true;
        }
    }

     // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.everyBotPickerSub.setPickerRelativeSpeed(0.0);
        if(interrupted) {
            done = true;      
        }
    }
     
     // Returns true when the command should end.
     @Override
     public boolean isFinished() {
         return done;
     }

     private void doPickerPusle() {
        if(this.pickerPulseAction == EveryBotPickerAction.ConeExpel) {
            this.everyBotPickerSub.setPickerRelativeSpeed(this.pickerPulseAmplitude * -1.0);
        }
        else if(this.pickerPulseAction == EveryBotPickerAction.ConeUptake) {
            this.everyBotPickerSub.setPickerRelativeSpeed(this.pickerPulseAmplitude);
        }
        else if(this.pickerPulseAction == EveryBotPickerAction.CubeExpel) {
            this.everyBotPickerSub.setPickerRelativeSpeed(this.pickerPulseAmplitude);
        }
        else if(this.pickerPulseAction == EveryBotPickerAction.CubeUptake) {
            this.everyBotPickerSub.setPickerRelativeSpeed(this.pickerPulseAmplitude * -1.0);
        }
    }
}