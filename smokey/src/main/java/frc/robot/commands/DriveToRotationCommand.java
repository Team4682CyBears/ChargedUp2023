// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rapid React - 2023
// File: DriveToRotationCommand.java
// Intent: Forms a command to drive the wheels .
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;
import static java.lang.Math.abs;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveToRotationCommand extends CommandBase{
    private static int rangeOfMotionDegreesFromZero = 5;
    private static int rangeOfMotionBandSize = 100/rangeOfMotionDegreesFromZero;

    private DrivetrainSubsystem drivetrain;
    private boolean done = false;
    private DoubleSupplier commandedInputSupplier = null;
    private Pose2d initialPosition = null;

    /** 
     * Creates a new driveCommand. 
    * 
    * @param drivetrainSubsystem - the drive train subsystem
    * @param plan - the Trajectory that should be followed by the robot
    */
    public DriveToRotationCommand(
    DrivetrainSubsystem drivetrainSubsystem,
    DoubleSupplier commandedStickInput) {
        this.drivetrain = drivetrainSubsystem;
        this.commandedInputSupplier = commandedStickInput;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrainSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
        initialPosition = this.drivetrain.getRobotPosition();
        done = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double requestedOffsetAngleRadians = 
            MathUtil.angleModulus(delta.getRotation().getRadians()))
            ((int)(commandedInputSupplier.getAsDouble() * 100.0)) / DriveToRotationCommand.rangeOfMotionBandSize;
        Pose2d currentPosition = drivetrain.getRobotPosition();
        double updatedRotationDelta = 
            MathUtil.angleModulus(initialPosition.getRotation().getRadians()) -
            MathUtil.angleModulus(currentPosition.getRotation().getRadians()) - 
            Math;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
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
