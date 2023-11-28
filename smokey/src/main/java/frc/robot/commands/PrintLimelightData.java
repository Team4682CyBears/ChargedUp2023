// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: RumbleCommand.java
// Intent: Forms a command to make the XBox controller rumble/vibrate.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class PrintLimelightData extends CommandBase {

    private boolean done = false;
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    public PrintLimelightData(){

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        done = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // read values periodically
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);
    
        // post to smart dashboard periodically
        System.out.println("HERE HERE LOOK HERE LIMELIGHT VALUES (x y area) <------------");
        System.out.println(x);
        System.out.println(y);
        System.out.println(area);
        done = true;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            done = true;
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return done;
    }

}