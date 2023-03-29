// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: DriveFinePlacementCommand.java
// Intent: Forms a command to drive the robot in fine placement mode.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.common.SwerveDriveRotationMode;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveFinePlacementCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;

    private ChassisSpeeds commandedChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    private double rotationalVelocity = 0.0;

    public DriveFinePlacementCommand(DrivetrainSubsystem drivetrainSubsystem,
        double rotationalVelocity) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.rotationalVelocity = rotationalVelocity;
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        drivetrainSubsystem.setSwerveDriveRotationMode(SwerveDriveRotationMode.FinePlacement);
    }

    @Override
    public void execute() {       
        // field-oriented rotational-only movement
        // around center of rotation at front of snout for fine placement
        commandedChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            0.0,
            0.0,
            rotationalVelocity,
            drivetrainSubsystem.getGyroscopeRotation()
            );   

        drivetrainSubsystem.drive(commandedChassisSpeeds);        
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
        drivetrainSubsystem.setSwerveDriveRotationMode(SwerveDriveRotationMode.Normal);
        System.out.println("Fine Placement Ended");
    }

}