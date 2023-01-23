// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: AutonomousChooser.java
// Intent: Allows auto mode routine to be selected from shuffleboard
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.control;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.DriveTimeCommand;

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
        autonomousModeChooser.addOption("Distance Test 1", AutonomousMode.DISTANCE_TEST_1);
        autonomousModeChooser.addOption("Distance Test 2", AutonomousMode.DISTANCE_TEST_2);
        autonomousModeChooser.addOption("Distance Test 2", AutonomousMode.DISTANCE_TEST_3);


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

    public Command getDistanceTest1() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command);

        // drive backward and then turn counterclockwise
        command.addCommands(driveSegment(0.2, 0, 0, 1.0));

        return command;
    }

    public Command getDistanceTest2() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command);

        // drive backward and then turn counterclockwise
        command.addCommands(driveSegment(0.1, 0, 0, 1.0));

        return command;
    }

    public Command getDistanceTest3() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command);

        // drive backward and then turn counterclockwise
        command.addCommands(driveSegment(0.2, 0, 0, 0.5));

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
                return getTestAutoForward();
            case TEST_AUTO_BACKWARD :
                return getTestAutoBackward();
            case DISTANCE_TEST_1 :
                return getDistanceTest1();
            case DISTANCE_TEST_2 :
                return getDistanceTest2();
            case DISTANCE_TEST_3 :
                return getDistanceTest3();
        }
        return new InstantCommand();
    }

    private enum AutonomousMode {
        TEST_AUTO_FORWARD, TEST_AUTO_BACKWARD, DISTANCE_TEST_1, DISTANCE_TEST_2, DISTANCE_TEST_3;
    }
}
