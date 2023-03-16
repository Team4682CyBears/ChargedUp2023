// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: DefaultDriveCommand.java
// Intent: Forms a command to drive the robot.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    // true if field oriented drive, false for robot oriented drive
    private final Boolean fieldOrientedDrive = true;
    private ChassisSpeeds commandedChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    private ChassisSpeeds previousChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    private double maxAccelerationMPerS2 = 6.0;
    private double maxAccelerationRadPerS2 = 14.0;
    // TODO move this to Constants
    private double deltaTimeSeconds = 0.02; // 20ms scheduler time tick

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        // NOTE: For now we will NOT register the NavxSubsystem, this is safe to do here because
        // all access to the class is read-only.
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {       
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        if(fieldOrientedDrive) {
            commandedChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                m_translationXSupplier.getAsDouble(),
                m_translationYSupplier.getAsDouble(),
                m_rotationSupplier.getAsDouble(),
                m_drivetrainSubsystem.getGyroscopeRotation()
                );   
        } else {
            commandedChassisSpeeds = new ChassisSpeeds(
                m_translationXSupplier.getAsDouble(),
                m_translationYSupplier.getAsDouble(),
                m_rotationSupplier.getAsDouble());
        }
   
        commandedChassisSpeeds = limitChassisSpeedsAccel(commandedChassisSpeeds);
        m_drivetrainSubsystem.drive(commandedChassisSpeeds);        
        previousChassisSpeeds = commandedChassisSpeeds;
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    private ChassisSpeeds limitChassisSpeedsAccel(ChassisSpeeds speeds) {
        double xAccel = (speeds.vxMetersPerSecond - previousChassisSpeeds.vxMetersPerSecond)/deltaTimeSeconds;
        double yAccel = (speeds.vyMetersPerSecond - previousChassisSpeeds.vyMetersPerSecond)/deltaTimeSeconds;
        double omegaAccel = (speeds.omegaRadiansPerSecond - previousChassisSpeeds.omegaRadiansPerSecond)/deltaTimeSeconds;
        double xVelocityLimited = speeds.vxMetersPerSecond;
        double yVelocityLimited = speeds.vyMetersPerSecond;
        double omegaVelocityLimited = speeds.omegaRadiansPerSecond;
        // if accelerations over limit
        if (Math.abs(xAccel) > maxAccelerationMPerS2){
            // new velocity is the old velocity + the maximum allowed change toward the new direction
            xVelocityLimited = 
                previousChassisSpeeds.vxMetersPerSecond 
                + Math.copySign(maxAccelerationMPerS2 * deltaTimeSeconds, xAccel);
        }
        if (Math.abs(yAccel) > maxAccelerationMPerS2){
            yVelocityLimited = 
                previousChassisSpeeds.vyMetersPerSecond 
                + Math.copySign(maxAccelerationMPerS2 * deltaTimeSeconds, yAccel);
        }
        if (Math.abs(omegaAccel) > maxAccelerationRadPerS2){
            omegaVelocityLimited = 
                previousChassisSpeeds.omegaRadiansPerSecond 
                + Math.copySign(maxAccelerationRadPerS2 * deltaTimeSeconds, omegaAccel);
        }
        return new ChassisSpeeds(xVelocityLimited, yVelocityLimited, omegaVelocityLimited);
    }
}