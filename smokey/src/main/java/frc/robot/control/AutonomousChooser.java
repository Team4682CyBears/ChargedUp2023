// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: AutonomousChooser.java
// Intent: Allows auto mode routine to be selected from shuffleboard
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.control;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.commands.ArmToLocationCommand;
import frc.robot.commands.AutoBalanceStepCommand;
import frc.robot.commands.DriveToPointCommand;
import frc.robot.commands.DriveTrajectoryCommand;
import frc.robot.commands.EveryBotPickerAutoExpellCommand;
import frc.robot.commands.ManipulatePickerCommand;
import frc.robot.commands.ArmToLocationCommand.ArmLocation;
import frc.robot.common.SwerveTrajectoryGenerator;
import frc.robot.common.VectorUtils;

/**
 * A class for choosing different auto mode routines from shuffleboard
 */
public class AutonomousChooser {
    private SubsystemCollection subsystems;
    private final SendableChooser<AutonomousPath> AutonomousPathChooser = new SendableChooser<>();
    private final SendableChooser<AutonomousBalance> balanceChooser = new SendableChooser<>();
    private final SendableChooser<ScoringPosition> scoreHeight = new SendableChooser<>();
    private Trajectories trajectories;
    
    //Robot to travel in the negative x direction
    //want to make sure snout is deelply engaged in the node, so overdrive by tolerence amount
    private Translation2d intoNodeTranslation = new Translation2d(
        -1 * (Constants.snoutDepth + Constants.TrajectoryPoseTol.getX()), 0);

    /**
     * Constructor for AutonomousChooser
     * @param subsystems - the SubsystemCollection
     */
    public AutonomousChooser(SubsystemCollection subsystems) {
        this.subsystems = subsystems;
        if(this.subsystems.getDriveTrainSubsystem() != null){
            System.out.println(">>>> creating auto trajectories");
            this.trajectories = new Trajectories(subsystems.getDriveTrainSubsystem()); 
            System.out.println(">>>> finished creating auto trajectories");
            
            AutonomousPathChooser.setDefaultOption("Node 1 Routine", AutonomousPath.LEFT_PATH);
            AutonomousPathChooser.addOption("Node 5 Routine", AutonomousPath.MIDDLE_PATH);
            AutonomousPathChooser.addOption("Node 9 Routine", AutonomousPath.RIGHT_PATH);
            AutonomousPathChooser.addOption("Test Node5 Score Routine", AutonomousPath.TEST_NODE5_SCORE_ROUTINE);
            AutonomousPathChooser.addOption("Test Setting Robot Position", AutonomousPath.TEST_SET_ROBOT_POSITION);
    
            balanceChooser.setDefaultOption("Do Balance", AutonomousBalance.DO_BALANCE);
            balanceChooser.addOption("Do NOT Balance", AutonomousBalance.DO_NOT_BALANCE);
    
            scoreHeight.setDefaultOption("Score High", ScoringPosition.SCORE_HIGH);
            scoreHeight.addOption("Score Middle", ScoringPosition.SCORE_MIDDLE);
            scoreHeight.addOption("Score Low", ScoringPosition.SCORE_LOW);
    
            SmartDashboard.putData(AutonomousPathChooser);
            SmartDashboard.putData(balanceChooser);
            SmartDashboard.putData(scoreHeight);
        }
        else {
            System.out.println(">>>> NO auto trajectories because no drive train subsystem");
        }
    }
    
    /**
     * A method to return the AutonomousPathChooser
     * @return AutonomousPathChooser
     */
    public SendableChooser<AutonomousPath> getModeChooser() {
        return AutonomousPathChooser;
    }

    /**
     * Builds a command list for use in auto routines.  This is the first part of the routine that scores the game piece. 
     * @param NodePosition
     * @return
     */
    public Command getScoreRoutine(Pose2d NodePosition){
        ArrayList<Pose2d> IntoNodeWaypoints = new ArrayList<Pose2d>();
        IntoNodeWaypoints.add(NodePosition);
        IntoNodeWaypoints.add(VectorUtils.translatePose(NodePosition, intoNodeTranslation));
        Trajectory IntoNodeTrajectory = SwerveTrajectoryGenerator.generateTrajectory(
            IntoNodeWaypoints, 
            subsystems.getDriveTrainSubsystem().getTrajectoryConfig());

        ArrayList<Pose2d> OutOfNodeWaypoints = new ArrayList<Pose2d>();
        OutOfNodeWaypoints.add(VectorUtils.translatePose(NodePosition, intoNodeTranslation));
        OutOfNodeWaypoints.add(NodePosition);
        Trajectory OutOfNodeTrajectory = SwerveTrajectoryGenerator.generateTrajectory(
            OutOfNodeWaypoints, 
            subsystems.getDriveTrainSubsystem().getTrajectoryConfig());

        SequentialCommandGroup command = new SequentialCommandGroup();
        setRobotPose(command, NodePosition);
        command.addCommands(new InstantCommand(
            () -> System.out.println("Begin Driving Trajectory from: " + subsystems.getDriveTrainSubsystem().getRobotPosition())));

        // drive into node
        ParallelCommandGroup intoNodeAndHighScore = new ParallelCommandGroup(
            new DriveTrajectoryCommand(subsystems.getDriveTrainSubsystem(), IntoNodeTrajectory));

        // move arm to high score
        if(this.subsystems.getArmSubsystem() != null) {
            intoNodeAndHighScore.addCommands(new ArmToLocationCommand(subsystems.getArmSubsystem(), ArmLocation.ARM_HIGH_SCORE));
        }

        command.addCommands(intoNodeAndHighScore);

        // expel the game piece by either opening the claw or running the motors to expell
        if(this.subsystems.getPickerSubsystem() != null) {
            command.addCommands(new ManipulatePickerCommand(subsystems.getPickerSubsystem(), true));
        }
        else if (this.subsystems.getEveryBotPickerSubsystem() != null) {
            command.addCommands(new EveryBotPickerAutoExpellCommand(subsystems.getEveryBotPickerSubsystem()));
        }

        // drive out of the score position
        ParallelCommandGroup outOfNodeAndStow = new ParallelCommandGroup(
            new DriveTrajectoryCommand(subsystems.getDriveTrainSubsystem(), OutOfNodeTrajectory));

        // stow the arm
        if(this.subsystems.getArmSubsystem() != null) {
            outOfNodeAndStow.addCommands(new ArmToLocationCommand(subsystems.getArmSubsystem(), ArmLocation.ARM_STOW));
        }

        command.addCommands(outOfNodeAndStow);

        // close the claw
        if(this.subsystems.getPickerSubsystem() != null) {
            command.addCommands(new ManipulatePickerCommand(subsystems.getPickerSubsystem(), false));
        }

        return command;
    }
    
    /**
     * Builds a command list for use in auto routines
     * @param NodePosition starting position of robot corrosponding to the node. Nodes are numbered from left to right 1- 9 from the drivers perspective
     * @param Trajectory trajectory to follow out of the community
     * @return command
     */
    public Command getAutoRoutine (Pose2d NodePosition, Trajectory Trajectory){
        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(getScoreRoutine(NodePosition));
        command.addCommands(new DriveTrajectoryCommand(subsystems.getDriveTrainSubsystem(), Trajectory));
        return command;
    }

    /**
     * Builds a command list for the balance routine, or will not if we toggle it so
     * @param DoBalance the enum for wether the balance routine should be run
     * @return
     */
    public Command getBalanceRoutine (AutonomousBalance DoBalance){
        SequentialCommandGroup command = new SequentialCommandGroup();
        if (DoBalance == AutonomousBalance.DO_BALANCE){
            command.addCommands(new DriveTrajectoryCommand(subsystems.getDriveTrainSubsystem(), trajectories.OntoRampTrajectory));
            command.addCommands(new AutoBalanceStepCommand(subsystems.getDriveTrainSubsystem())
            .repeatedly().until(subsystems.getDriveTrainSubsystem()::isLevel));
        }
        return command;
    }

    public Command getLeftRoutine(){
        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(getAutoRoutine(trajectories.Node1Position, trajectories.LeftTrajectory));
        command.addCommands(getBalanceRoutine(balanceChooser.getSelected()));
        return command;
    }

    public Command getRightRoutine(){
        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(getAutoRoutine(trajectories.Node9Position, trajectories.RightTrajectory));
        command.addCommands(getBalanceRoutine(balanceChooser.getSelected()));
        return command;
    }

    public Command getMiddleRoutine(){
        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(getAutoRoutine(trajectories.Node5Position, trajectories.MiddleTrajectory));
        command.addCommands(getBalanceRoutine(balanceChooser.getSelected()));
        return command;
    }

    private void setRobotPose(SequentialCommandGroup command, Pose2d pose){
        command.addCommands(
            // set yaw to the starting rotation here so that field orientation ends up correct after auto
            new InstantCommand(() -> subsystems.getDriveTrainSubsystem().setYaw(pose.getRotation().getDegrees()),
            subsystems.getDriveTrainSubsystem()));
        command.addCommands(
            new InstantCommand(() -> subsystems.getDriveTrainSubsystem().setRobotPosition(pose),
            subsystems.getDriveTrainSubsystem()));
        command.addCommands(new InstantCommand(
            () -> System.out.println("Setting Robot Position to : " + pose)));
    }

    private Command getTestRobotPositionRoutine(){
        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(new InstantCommand(
            () -> System.out.println("Setting Robot Position to: " + trajectories.Node1Position)));
        command.addCommands(new InstantCommand(
            () -> subsystems.getDriveTrainSubsystem().setRobotPosition(trajectories.Node1Position),
            subsystems.getDriveTrainSubsystem()));
        command.addCommands(new InstantCommand(
            () -> System.out.println("Robot Position: " + subsystems.getDriveTrainSubsystem().getRobotPosition())));
        command.addCommands(new InstantCommand(
            () -> System.out.println("Setting Robot Position to: " + trajectories.Node5Position)));
        command.addCommands(new InstantCommand(
            () -> subsystems.getDriveTrainSubsystem().setRobotPosition(trajectories.Node5Position),
            subsystems.getDriveTrainSubsystem()));
        command.addCommands(new InstantCommand(
            () -> System.out.println("Robot Position: " + subsystems.getDriveTrainSubsystem().getRobotPosition())));
        command.addCommands(new InstantCommand(
            () -> System.out.println("Setting Robot Position to: " + trajectories.Node9Position)));
        command.addCommands(new InstantCommand(
            () -> subsystems.getDriveTrainSubsystem().setRobotPosition(trajectories.Node9Position),
            subsystems.getDriveTrainSubsystem()));
        command.addCommands(new InstantCommand(
            () -> System.out.println("Robot Position: " + subsystems.getDriveTrainSubsystem().getRobotPosition())));
        return command;
    }

    /**
     * A method to return the chosen auto command
     * @param subsystems - the SubsystemCollection
     * @return command
     */
    public Command getCommand() {
        switch (AutonomousPathChooser.getSelected()) {
            case LEFT_PATH :
                return this.getLeftRoutine();
            case RIGHT_PATH :
                return this.getRightRoutine();
            case MIDDLE_PATH :
                return this.getMiddleRoutine();
            case TEST_NODE5_SCORE_ROUTINE:
                return this.getScoreRoutine(trajectories.Node5Position);
            case TEST_SET_ROBOT_POSITION:
                return this.getTestRobotPositionRoutine();
        }
        return new InstantCommand();
    }

    private enum AutonomousPath {
        LEFT_PATH,
        RIGHT_PATH,
        MIDDLE_PATH,
        TEST_NODE5_SCORE_ROUTINE,
        TEST_SET_ROBOT_POSITION
    }

    private enum AutonomousBalance {
        DO_BALANCE,
        DO_NOT_BALANCE
    }

    private enum ScoringPosition {
        SCORE_HIGH,
        SCORE_MIDDLE,
        SCORE_LOW
    }
}
