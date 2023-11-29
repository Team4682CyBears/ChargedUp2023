// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: AllignWithTag.java
// Intent: Forms a command to allign itself with a designated april tag.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.common.MotorUtils;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AllignWithTag extends CommandBase{
    private boolean done = false;
    private double tagID;
    private double velocityValue = 0.3;
    private PIDController yPID = new PIDController(1.0, 0.0, 0.0);
    private DrivetrainSubsystem drivetrainsubsystem = null;
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    
    public AllignWithTag(double tagID, DrivetrainSubsystem drivetrainSubsystem){
        this.tagID = tagID;

        this.drivetrainsubsystem = drivetrainSubsystem;

        addRequirements(drivetrainsubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        done = false;
        drivetrainsubsystem.drive(new ChassisSpeeds(0,0,0));
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double tid = table.getEntry("tid").getDouble(0.0);
        double[] relativeBotpos = table.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
        double relativeBotY = relativeBotpos[0];

        System.out.println(relativeBotY);


        if (tid != tagID){
            done = true;
        }
        else {
            double velocity = yPID.calculate(relativeBotY, 0.0);
            velocity = -1 * MotorUtils.clamp(velocity, -velocityValue, velocityValue);
            drivetrainsubsystem.drive(new ChassisSpeeds(0, velocity, 0));
        }

        }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            done = true;
            drivetrainsubsystem.drive(new ChassisSpeeds(0,0,0));
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        double[] relativeBotpos = table.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
        double relativeBotY = relativeBotpos[0];
        // close enough is abs(robot pose y) < 0.05
        if (Math.abs(relativeBotY) < 0.05){
            done = true;
        }
        return done;
    }
    
}
