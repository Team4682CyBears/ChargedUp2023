// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: AutonomousChooser.java
// Intent: Allows auto mode routine to be selected from shuffleboard
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.control;

import javax.print.attribute.standard.RequestingUserName;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.constraint.RectangularRegionConstraint;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.DriveToPointCommand;
import frc.robot.commands.DriveTrajectoryCommand;
import frc.robot.commands.ManipulatePickerCommand;
import frc.robot.commands.MoveArmToScoring;
import frc.robot.commands.MoveArmToStowed;
import frc.robot.control.Trajectories;
import edu.wpi.first.math.util.Units;

/**
 * A class for choosing different auto mode routines from shuffleboard
 */
public class AutonomousChooser {
    private SubsystemCollection subsystems;
    private final SendableChooser<AutonomousMode> autonomousModeChooser = new SendableChooser<>();
    private final SendableChooser<AutonomousMode> balanceChooser = new SendableChooser<>();
    private final SendableChooser<AutonomousMode> scoreHeight = new SendableChooser<>();
    private Trajectories trajectories;

    /**
     * Constructor for AutonomousChooser
     * @param subsystems - the SubsystemCollection
     */
    public AutonomousChooser(SubsystemCollection subsystems) {
        this.subsystems = subsystems;
        System.out.println(">>>> creating auto trajectories");
        this.trajectories = new Trajectories(subsystems.getDriveTrainSubsystem()); 
        System.out.println(">>>> finished creating auto trajectories");
        
        autonomousModeChooser.addOption("Node 1 Routine", AutonomousMode.LEFT_PATH);
        autonomousModeChooser.addOption("Node 5 Routine", AutonomousMode.MIDDLE_PATH);
        autonomousModeChooser.addOption("Node 9 Routine", AutonomousMode.RIGHT_PATH);

        balanceChooser.setDefaultOption("Do Balance", AutonomousMode.DO_BALANCE);
        balanceChooser.addOption("Do NOT Balance", AutonomousMode.DO_NOT_BALANCE);

        scoreHeight.addOption("Score High", AutonomousMode.SCORE_HIGH);
        scoreHeight.addOption("Score Middle", AutonomousMode.SCORE_MIDDLE);
        scoreHeight.addOption("Score Low", AutonomousMode.SCORE_LOW);

        SmartDashboard.putData(autonomousModeChooser);
        SmartDashboard.putData(balanceChooser);
        SmartDashboard.putData(scoreHeight);
    }
    
    /**
     * A method to return the autonomousModeChooser
     * @return autonomousModeChooser
     */
    public SendableChooser<AutonomousMode> getModeChooser() {
        return autonomousModeChooser;
    }

    public Command getLeftRoutine(){
        SequentialCommandGroup command = new SequentialCommandGroup();
        resetRobotPose(command);
        command.addCommands(new InstantCommand(() -> subsystems.getDriveTrainSubsystem().setRobotPosition(trajectories.Node1Position)));
        command.addCommands(new DriveToPointCommand(subsystems.getDriveTrainSubsystem(), subsystems.getDriveTrainSubsystem().getRobotPosition().plus(new Transform2d(new Translation2d(Units.inchesToMeters(-5.25), 0), new Rotation2d(0.0)))));
        command.addCommands(new MoveArmToScoring(getScoringHeight()));
        command.addCommands(new ManipulatePickerCommand(false));
        command.addCommands(new DriveToPointCommand(subsystems.getDriveTrainSubsystem(), subsystems.getDriveTrainSubsystem().getRobotPosition().plus(new Transform2d(new Translation2d(Units.inchesToMeters(5.25), 0), new Rotation2d(0.0)))));
        command.addCommands(new MoveArmToStowed());
        command.addCommands(new DriveTrajectoryCommand(subsystems.getDriveTrainSubsystem(), trajectories.LeftTrajectory));
        command.addCommands(new InstantCommand(() -> subsystems.getDriveTrainSubsystem().setRobotPosition(trajectories.End)));
        if (getDoBalance()) {
        command.addCommands(new DriveTrajectoryCommand(subsystems.getDriveTrainSubsystem(), trajectories.OntoRampTrajectory));
        //Autobalance Command HERE <----------------------------- LOOK DO THIS NOW NOW NOW NOW NOW NOW
        }
        return command;
    }

    public Command getRightRoutine(){
        SequentialCommandGroup command = new SequentialCommandGroup();
        resetRobotPose(command);
        command.addCommands(new InstantCommand(() -> subsystems.getDriveTrainSubsystem().setRobotPosition(trajectories.Node9Position)));
        command.addCommands(new DriveToPointCommand(subsystems.getDriveTrainSubsystem(), subsystems.getDriveTrainSubsystem().getRobotPosition().plus(new Transform2d(new Translation2d(Units.inchesToMeters(-5.25), 0), new Rotation2d(0.0)))));
        command.addCommands(new MoveArmToScoring(getScoringHeight()));
        command.addCommands(new ManipulatePickerCommand(false));
        command.addCommands(new DriveToPointCommand(subsystems.getDriveTrainSubsystem(), subsystems.getDriveTrainSubsystem().getRobotPosition().plus(new Transform2d(new Translation2d(Units.inchesToMeters(5.25), 0), new Rotation2d(0.0)))));
        command.addCommands(new MoveArmToStowed());
        command.addCommands(new DriveTrajectoryCommand(subsystems.getDriveTrainSubsystem(), trajectories.RightTrajectory));
        command.addCommands(new InstantCommand(() -> subsystems.getDriveTrainSubsystem().setRobotPosition(trajectories.End)));
        if (getDoBalance()) {
        command.addCommands(new DriveTrajectoryCommand(subsystems.getDriveTrainSubsystem(), trajectories.OntoRampTrajectory));
        //Autobalance Command HERE <----------------------------- LOOK DO THIS NOW NOW NOW NOW NOW NOW
        }
        return command;
    }

    public Command getMiddleRoutine(){
        SequentialCommandGroup command = new SequentialCommandGroup();
        resetRobotPose(command);
        command.addCommands(new InstantCommand(() -> subsystems.getDriveTrainSubsystem().setRobotPosition(trajectories.Node5Position)));
        command.addCommands(new DriveToPointCommand(subsystems.getDriveTrainSubsystem(), subsystems.getDriveTrainSubsystem().getRobotPosition().plus(new Transform2d(new Translation2d(Units.inchesToMeters(-5.25), 0), new Rotation2d(0.0)))));
        command.addCommands(new MoveArmToScoring(getScoringHeight()));
        command.addCommands(new ManipulatePickerCommand(false));
        command.addCommands(new DriveToPointCommand(subsystems.getDriveTrainSubsystem(), subsystems.getDriveTrainSubsystem().getRobotPosition().plus(new Transform2d(new Translation2d(Units.inchesToMeters(5.25), 0), new Rotation2d(0.0)))));
        command.addCommands(new MoveArmToStowed());
        command.addCommands(new DriveTrajectoryCommand(subsystems.getDriveTrainSubsystem(), trajectories.MiddleTrajectory));
        command.addCommands(new InstantCommand(() -> subsystems.getDriveTrainSubsystem().setRobotPosition(trajectories.End)));
        if (getDoBalance()) {
        command.addCommands(new DriveTrajectoryCommand(subsystems.getDriveTrainSubsystem(), trajectories.OntoRampTrajectory));
        //Autobalance Command HERE <----------------------------- LOOK DO THIS NOW NOW NOW NOW NOW NOW
        }
        return command;
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
            case LEFT_PATH :
                return this.getLeftRoutine();
            case RIGHT_PATH :
                return this.getRightRoutine();
            case MIDDLE_PATH :
                return this.getMiddleRoutine();
        }
        return new InstantCommand();
    }

    public boolean getDoBalance(){
        Boolean DoTheBalance;
        switch (balanceChooser.getSelected()) {
            default : case DO_BALANCE :
                DoTheBalance = true;
            case DO_NOT_BALANCE :
                DoTheBalance = false;
        }
        return DoTheBalance;
    }

    public int getScoringHeight(){
        switch (scoreHeight.getSelected()) {
            default : case SCORE_HIGH :
                return 3;
            case SCORE_MIDDLE :
                return 2;
            case SCORE_LOW :
                return 1;
        }
    }

    private enum AutonomousMode {
        LEFT_PATH,
        RIGHT_PATH,
        MIDDLE_PATH,
        DO_BALANCE,
        DO_NOT_BALANCE,
        SCORE_HIGH,
        SCORE_MIDDLE,
        SCORE_LOW
    }
}
