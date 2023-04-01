// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: ArmToReferencePositionCommand.java
// Intent: Forms a command to move the dual part arm with y and z inputs.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

/*
 * Forms a command to move the dual part arm with y and z motor inputs.  
 * Y motor input for the extension/horizontal arm.
 * Z motor input for the angle/vertical arm.
 */
public class ArmToReferencePositionCommand extends CommandBase {
    private final ArmSubsystem armSubsystem;
    private Timer timer = new Timer();
    private boolean done = false;
    private final double retractMaximumTime = 3.0;
    private final double horizontalRetractSpeed = -1.0;
    private final double verticalRetractSpeed = -1.0;

    /**
     * The constructor to create a default command for the arm subsystem.
     * @param theArmSubsystem - the arm subsystem
     */
    public ArmToReferencePositionCommand(ArmSubsystem theArmSubsystem) {
        this.armSubsystem = theArmSubsystem;
        addRequirements(this.armSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        timer.reset();
        timer.start();
        done = false;
    }

    @Override
    public void execute() {

        if (timer.hasElapsed(this.retractMaximumTime) || this.done == true) {
          this.done = true;
          this.armSubsystem.setArmSpeeds(0.0, 0.0);
          System.out.println("Arm to reference: Timer has elapsed!");
        }
        else {
            double verticalSpeed = 0.0;
            double horizontalSpeed = 0.0;
            boolean horizontalEncoderResetViaSensor = this.armSubsystem.hasHorizontalArmEncoderBeenResetViaSensor();
            boolean verticalEncoderResetViaSensor = this.armSubsystem.hasVerticalArmEncoderBeenResetViaSensor();
            if(horizontalEncoderResetViaSensor == false) {
                horizontalSpeed = horizontalRetractSpeed;
            }
            if(verticalEncoderResetViaSensor == false) {
                verticalSpeed = verticalRetractSpeed;
            }
            this.armSubsystem.setArmSpeeds(horizontalSpeed, verticalSpeed);
            this.done = horizontalEncoderResetViaSensor && verticalEncoderResetViaSensor;
            if(this.done) {
                System.out.println("ArmToReferencePositionCommand is DONE! at timer " + timer.get());
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.armSubsystem.setArmSpeeds(0.0, 0.0);
        if(interrupted) {
          done = true;
        }
    }
    
    @Override
    public boolean isFinished() {
        return this.done;
    }
}