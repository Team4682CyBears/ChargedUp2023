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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.commands.ArmToLocationCommand;
import frc.robot.commands.ArmToReferencePositionCommand;
import frc.robot.commands.AutoBalanceStepCommand;
import frc.robot.commands.DriveTrajectoryCommand;
import frc.robot.commands.EveryBotPickerAutoCommand;
import frc.robot.commands.ManipulatePickerCommand;
import frc.robot.commands.ArmToLocationCommand.ArmLocation;
import frc.robot.common.SwerveTrajectoryConfig;
import frc.robot.common.EveryBotPickerAction;
import frc.robot.common.SwerveTrajectoryGenerator;
import frc.robot.common.VectorUtils;

/**
 * A class for choosing different auto mode routines from shuffleboard
 */
public class AutonomousChooser {
    private SubsystemCollection subsystems;
    private final SendableChooser<AutonomousPath> autonomousPathChooser = new SendableChooser<>();
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
            
            autonomousPathChooser.setDefaultOption("Direct Onto Ramp Routine", AutonomousPath.DIRECT_PATH);
            autonomousPathChooser.addOption("Node 1 (Left) Routine", AutonomousPath.LEFT_PATH);
            autonomousPathChooser.addOption("Node 2 (Left) Routine", AutonomousPath.NODE2_ROUTINE);
            autonomousPathChooser.addOption("Node 5 Routine", AutonomousPath.MIDDLE_PATH);
            autonomousPathChooser.addOption("Node 8 (Right) Routine", AutonomousPath.NODE8_ROUTINE);
            autonomousPathChooser.addOption("Node 9 (Right) Routine", AutonomousPath.RIGHT_PATH);
            autonomousPathChooser.addOption("Test Node5 Score Routine", AutonomousPath.TEST_NODE5_SCORE_ROUTINE);
    
            balanceChooser.setDefaultOption("Do Balance", AutonomousBalance.DO_BALANCE);
            balanceChooser.addOption("Do NOT Balance", AutonomousBalance.DO_NOT_BALANCE);
    
            scoreHeight.setDefaultOption("Score High", ScoringPosition.SCORE_HIGH);
            scoreHeight.addOption("Score Middle", ScoringPosition.SCORE_MIDDLE);
    
            SmartDashboard.putData(autonomousPathChooser);
            SmartDashboard.putData(balanceChooser);
            SmartDashboard.putData(scoreHeight);
        }
        else {
            System.out.println(">>>> NO auto trajectories because no drive train subsystem");
        }
    }

    /**
     * A method to return the chosen auto command
     * @param subsystems - the SubsystemCollection
     * @return command
     */
    public Command getCommand() {
        switch (autonomousPathChooser.getSelected()) {
            case LEFT_PATH :
                return this.getLeftRoutine();
            case RIGHT_PATH :
                return this.getRightRoutine();
            case MIDDLE_PATH :
                return this.getMiddleRoutine();
            case DIRECT_PATH :
                return this.getDirectRoutine();
            case NODE2_ROUTINE:
                return this.getNode2Routine();
            case NODE8_ROUTINE:
                return this.getNode8Routine();
            case TEST_NODE5_SCORE_ROUTINE:
                return this.getScoreRoutine(trajectories.getNode5Position(), trajectories.getConfig());
        }
        return new InstantCommand();
    }
    
    /**
     * Builds a command list for use in auto routines.  This is the first part of the routine that scores the game piece. 
     * @param NodePosition
     * @param config  - when the auto will drive another trajectory after scoring, supply a config with a higher ending velocity.   
     * @return
     */
    private Command getScoreRoutine(Pose2d NodePosition, SwerveTrajectoryConfig config){
        // for now we will always assume that we are attempting to score the cube
        subsystems.getManualInputInterfaces().setTargetGamePieceAsCube();

        // Build into/out of node trajectories in real time because they depend on the starting position
        ArrayList<Pose2d> IntoNodeWaypoints = new ArrayList<Pose2d>();
        IntoNodeWaypoints.add(NodePosition);
        IntoNodeWaypoints.add(VectorUtils.translatePose(NodePosition, intoNodeTranslation));
        // use the default config for IntoNodeTrajectory
        Trajectory IntoNodeTrajectory = SwerveTrajectoryGenerator.generateTrajectory(
            IntoNodeWaypoints, 
            subsystems.getDriveTrainSubsystem().getTrajectoryConfig());

        ArrayList<Pose2d> OutOfNodeWaypoints = new ArrayList<Pose2d>();
        OutOfNodeWaypoints.add(VectorUtils.translatePose(NodePosition, intoNodeTranslation));
        OutOfNodeWaypoints.add(NodePosition);
        // use the supplied config for OutOfNodeTrajectory
        Trajectory OutOfNodeTrajectory = SwerveTrajectoryGenerator.generateTrajectory(
            OutOfNodeWaypoints, 
            config);

        SequentialCommandGroup command = new SequentialCommandGroup();
        setRobotPose(command, NodePosition);
        command.addCommands(new InstantCommand(
            () -> System.out.println("Begin Driving Trajectory from: " + subsystems.getDriveTrainSubsystem().getRobotPosition())));

        // drive into node
        ParallelCommandGroup intoNodeAndHighScore = new ParallelCommandGroup(
            new DriveTrajectoryCommand(subsystems.getDriveTrainSubsystem(), IntoNodeTrajectory));

        // move arm score into selected position
        if(this.subsystems.getArmSubsystem() != null) {
            SequentialCommandGroup armSequence = new SequentialCommandGroup();
            armSequence.addCommands(new ArmToReferencePositionCommand(subsystems.getArmSubsystem()));
            armSequence.addCommands(this.getArmPositionRoutine(scoreHeight.getSelected()));
            intoNodeAndHighScore.addCommands(armSequence);
        }

        command.addCommands(intoNodeAndHighScore);

        // expel the game piece by either opening the claw or running the motors to expell
        if(this.subsystems.getPickerSubsystem() != null) {
            command.addCommands(new ManipulatePickerCommand(subsystems.getPickerSubsystem(), true));
        }
        else if (this.subsystems.getEveryBotPickerSubsystem() != null) {// cube uses uptake command to expell
            command.addCommands(new EveryBotPickerAutoCommand(EveryBotPickerAction.CubeExpel, subsystems.getEveryBotPickerSubsystem()));
        }

        // drive out of the score position
        ParallelCommandGroup outOfNodeAndStow = new ParallelCommandGroup(
            new DriveTrajectoryCommand(subsystems.getDriveTrainSubsystem(), OutOfNodeTrajectory));

        // stow the arm
        if(this.subsystems.getArmSubsystem() != null) {
            outOfNodeAndStow.addCommands(
                new ArmToLocationCommand(
                    subsystems.getArmSubsystem(),
                    ArmLocation.ARM_STOW,
                    subsystems.getManualInputInterfaces()));
        }

        command.addCommands(outOfNodeAndStow);

        // close the claw
        if(this.subsystems.getPickerSubsystem() != null) {
            command.addCommands(new ManipulatePickerCommand(subsystems.getPickerSubsystem(), false));
        }

        return command;
    }
    
    /**
     * A method to move the arm to the specified scoring position. 
     * Caller is responsible for checking that the Arm Subsystem is installed. 
     * @param height
     * @return command
     */
    private Command getArmPositionRoutine (ScoringPosition height){
        if (height == ScoringPosition.SCORE_HIGH){
            return new ArmToLocationCommand(
                subsystems.getArmSubsystem(),
                ArmToLocationCommand.ArmLocation.ARM_HIGH_SCORE,
                subsystems.getManualInputInterfaces());
        }
        else if (height == ScoringPosition.SCORE_MIDDLE){
            return new ArmToLocationCommand(
                subsystems.getArmSubsystem(),
                ArmToLocationCommand.ArmLocation.ARM_MED_SCORE,
                subsystems.getManualInputInterfaces());
        }
        return new InstantCommand();
    }

    /**
     * Builds a command list for the balance routine, or will not if we toggle it so
     * @param DoBalance the enum for wether the balance routine should be run
     * @return command
     */
    private Command getBalanceRoutine (AutonomousBalance DoBalance, Trajectory toRampTrajectory){
        SequentialCommandGroup command = new SequentialCommandGroup();
        if (DoBalance == AutonomousBalance.DO_BALANCE){
            command.addCommands(new DriveTrajectoryCommand(subsystems.getDriveTrainSubsystem(), toRampTrajectory));
            command.addCommands(new AutoBalanceStepCommand(subsystems.getDriveTrainSubsystem()));
        }
        return command;
    }

        private Command getDirectRoutine(){
        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(getScoreAndDriveRoutine(trajectories.getNode5Position(), trajectories.getDirectToRampTrajectory()));
        command.addCommands(new AutoBalanceStepCommand(subsystems.getDriveTrainSubsystem()));
        return command;
    }

    private Command getLeftRoutine(){
        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(getScoreAndDriveRoutine(trajectories.getNode1Position(), trajectories.getLeftTrajectory()));
        command.addCommands(getBalanceRoutine(balanceChooser.getSelected(), trajectories.getLeftToOntoRampTrajectory()));
        return command;
    }

    private Command getMiddleRoutine(){
        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(getScoreAndDriveRoutine(trajectories.getNode5Position(), trajectories.getMiddleTrajectoryPart1()));
        command.addCommands(new DriveTrajectoryCommand(subsystems.getDriveTrainSubsystem(), trajectories.getMiddleTrajectoryPart2()));
        command.addCommands(getBalanceRoutine(balanceChooser.getSelected(), trajectories.getBehindToOntoRampTrajectory()));
        return command;
    }

    private Command getNode2Routine(){
        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(getScoreAndDriveRoutine(trajectories.getNode2Position(), trajectories.getNode2Trajectory()));
        command.addCommands(getBalanceRoutine(balanceChooser.getSelected(), trajectories.getLeftToOntoRampTrajectory()));
        return command;
    }

    private Command getNode8Routine(){
        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(getScoreAndDriveRoutine(trajectories.getNode8Position(), trajectories.getNode8Trajectory()));
        command.addCommands(getBalanceRoutine(balanceChooser.getSelected(), trajectories.getRightToOntoRampTrajectory()));
        return command;
    }

    private Command getRightRoutine(){
        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(getScoreAndDriveRoutine(trajectories.getNode9Position(), trajectories.getRightTrajectory()));
        command.addCommands(getBalanceRoutine(balanceChooser.getSelected(), trajectories.getRightToOntoRampTrajectory()));
        return command;
    }

    /**
     * Builds a command list for use in auto routines
     * @param NodePosition starting position of robot corrosponding to the node. Nodes are numbered from left to right 1- 9 from the drivers perspective
     * @param Trajectory trajectory to follow out of the community
     * @return command
     */
    private Command getScoreAndDriveRoutine (Pose2d NodePosition, Trajectory Trajectory){
        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(getScoreRoutine(NodePosition, trajectories.getFirstSegmentConfig()));
        command.addCommands(new DriveTrajectoryCommand(subsystems.getDriveTrainSubsystem(), Trajectory));
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

    private enum AutonomousPath {
        LEFT_PATH,
        RIGHT_PATH,
        MIDDLE_PATH,
        DIRECT_PATH,
        TEST_NODE5_SCORE_ROUTINE,
        NODE2_ROUTINE,
        NODE8_ROUTINE
    }

    private enum AutonomousBalance {
        DO_BALANCE,
        DO_NOT_BALANCE
    }

    private enum ScoringPosition {
        SCORE_HIGH,
        SCORE_MIDDLE,
    }
}