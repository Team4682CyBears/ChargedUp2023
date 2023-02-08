// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: ArmToPointCommand.java
// Intent: Forms a command to move the dual part arm to a y and z point in space.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmToPointCommand extends CommandBase {
    private final ArmSubsystem armSubsystem;
    private final double yValue;
    private final double zValue;
    private boolean done = false;

    public ArmToPointCommand(ArmSubsystem theArmSubsystem,
                             double yPointMeters,
                             double zPointMeters) {
        this.armSubsystem = theArmSubsystem;
        this.yValue = yPointMeters;
        this.zValue = zPointMeters;
        addRequirements(this.armSubsystem);
    }

    @Override
    public void initialize() {
        // when the point in space is invalid just be done
        done = (this.armSubsystem.setArmToPointInSpace(yValue, zValue) == false);
        if(done)
        {
            System.out.println("The requested arm point is invalid for the current arms!!! Y = " + yValue + " Z = " + zValue);
        }
    }

    @Override
    public void execute() {
        done = this.armSubsystem.isRequestedArmMovementComplete();
    }

    @Override
    public void end(boolean interrupted) {
        if(interrupted){
            done = true;
        }
        this.armSubsystem.setArmSpeeds(0.0, 0.0);
    }

    @Override
    public boolean isFinished(){
        return done;
    }
}