// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: AutonomousChooser.java
// Intent: Allows auto mode routine to be selected from shuffleboard
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.control;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.DriveTimeCommand;
import frc.robot.commands.DriveToPointCommand;

/**
 * A class for choosing different auto mode routines from shuffleboard
 */
public class AutonomousChooser {
    private SubsystemCollection subsystems;
    private final SendableChooser<AutonomousMode> autonomousModeChooser = new SendableChooser<>();

    /**
     * Constructor for AutonomousChooser
     * @param subsystems - the SubsystemCollection
     */
    public AutonomousChooser(SubsystemCollection subsystems) {
        this.subsystems = subsystems;

        autonomousModeChooser.setDefaultOption("Test Auto Forward", AutonomousMode.TEST_AUTO_FORWARD);
        autonomousModeChooser.addOption("Test Auto Backward", AutonomousMode.TEST_AUTO_BACKWARD);
        autonomousModeChooser.addOption("Test Auto To A Position", AutonomousMode.TEST_AUTO_DRIVE_TO_POSITION);

        SmartDashboard.putData(autonomousModeChooser);
    }
    
    /**
     * A method to return the autonomousModeChooser
     * @return autonomousModeChooser
     */
    public SendableChooser<AutonomousMode> getModeChooser() {
        return autonomousModeChooser;
    }

    /**
     * A method to get the TestAutoForward command
     * @return command
     */
    public Command getTestAutoForward() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command);

        // drive forward and then turn clockwise
        command.addCommands(driveSegment(1., 0, 0, 1.0));
        command.addCommands(driveSegment(0, 0, 0.5, 2.0));

        return command;
    }

    /**
     * A method to get the TestAutoBackward command
     * @return command
     */
    public Command getTestAutoBackward() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command);

        // drive backward and then turn counterclockwise
        command.addCommands(driveSegment(-1., 0, 0, 1.0));
        command.addCommands(driveSegment(0, 0, -0.5, 2.0));

        return command;
    }

    /**
     * A method to get the create a DriveToPointCommand using the current position as a starting point
     * @return command
     */
    public Command getTestAutoToPosition() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command);

        Pose2d currentPosition = subsystems.getDriveTrainSubsystem().getRobotPosition();
        Pose2d destinationPosition = currentPosition.plus(new Transform2d(new Translation2d(1.5, 0.5), Rotation2d.fromDegrees(180.0)));
        command.addCommands(new DriveToPointCommand(subsystems.getDriveTrainSubsystem(), destinationPosition));

        return command;
    }    

    private Command driveSegment(double x, double y, double rot, double durationSeconds) {
        return new DriveTimeCommand(subsystems.getDriveTrainSubsystem(), x, y, rot, durationSeconds);
    }
    
    private void resetRobotPose(SequentialCommandGroup command) {
        // TODO this is where we would set the starting robot position. 
        // just zeroing the gyro for now
        command.addCommands(new InstantCommand(() -> subsystems.getNavxSubsystem().zeroGyroscope()));
    }

    /**
     * A method to return the chosen auto command
     * @param subsystems - the SubsystemCollection
     * @return command
     */
    public Command getCommand() {
        switch (autonomousModeChooser.getSelected()) {
            case TEST_AUTO_FORWARD :
                return this.getTestAutoForward();
            case TEST_AUTO_BACKWARD :
                return this.getTestAutoBackward();
            case TEST_AUTO_DRIVE_TO_POSITION :
                return this.getTestAutoToPosition();
        }
        return new InstantCommand();
    }

    private enum AutonomousMode {
        TEST_AUTO_FORWARD,
        TEST_AUTO_BACKWARD,
        TEST_AUTO_DRIVE_TO_POSITION, 
    }
}
