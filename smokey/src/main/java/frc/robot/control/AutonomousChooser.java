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
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.commands.ArmToLocationCommand;
import frc.robot.commands.AutoBalanceStepCommand;
import frc.robot.commands.DriveToPointCommand;
import frc.robot.commands.DriveTrajectoryCommand;
import frc.robot.commands.ManipulatePickerCommand;
import frc.robot.commands.ArmToLocationCommand.ArmLocation;

/**
 * A class for choosing different auto mode routines from shuffleboard
 */
public class AutonomousChooser {
    private SubsystemCollection subsystems;
    private final SendableChooser<AutonomousPath> AutonomousPathChooser = new SendableChooser<>();
    private final SendableChooser<AutonomousBalance> balanceChooser = new SendableChooser<>();
    private final SendableChooser<ScoringPosition> scoreHeight = new SendableChooser<>();
    private Trajectories trajectories;
    
    private Transform2d intoNodeTransform = new Transform2d(new Translation2d(Constants.snoutDepth * -1, 0), new Rotation2d(0.0));
    //private Transform2d outOfNodeTransform = new Transform2d(new Translation2d(Constants.snoutDepth, 0), new Rotation2d(0.0));

    /**
     * Constructor for AutonomousChooser
     * @param subsystems - the SubsystemCollection
     */
    public AutonomousChooser(SubsystemCollection subsystems) {
        this.subsystems = subsystems;
        System.out.println(">>>> creating auto trajectories");
        this.trajectories = new Trajectories(subsystems.getDriveTrainSubsystem()); 
        System.out.println(">>>> finished creating auto trajectories");
        
        AutonomousPathChooser.addOption("Node 1 Routine", AutonomousPath.LEFT_PATH);
        AutonomousPathChooser.addOption("Node 5 Routine", AutonomousPath.MIDDLE_PATH);
        AutonomousPathChooser.addOption("Node 9 Routine", AutonomousPath.RIGHT_PATH);

        balanceChooser.setDefaultOption("Do Balance", AutonomousBalance.DO_BALANCE);
        balanceChooser.addOption("Do NOT Balance", AutonomousBalance.DO_NOT_BALANCE);

        scoreHeight.addOption("Score High", ScoringPosition.SCORE_HIGH);
        scoreHeight.addOption("Score Middle", ScoringPosition.SCORE_MIDDLE);
        scoreHeight.addOption("Score Low", ScoringPosition.SCORE_LOW);

        SmartDashboard.putData(AutonomousPathChooser);
        SmartDashboard.putData(balanceChooser);
        SmartDashboard.putData(scoreHeight);
    }
    
    /**
     * A method to return the AutonomousPathChooser
     * @return AutonomousPathChooser
     */
    public SendableChooser<AutonomousPath> getModeChooser() {
        return AutonomousPathChooser;
    }

    /**
     * Builds a command list for use in auto routines
     * @param NodePosition starting position of robot corrosponding to the node. Nodes are numbered from left to right 1- 9 from the drivers perspective
     * @param Trajectory trajectory to follow out of the community
     * @return command
     */
    public Command getAutoRoutine (Pose2d NodePosition, Trajectory Trajectory){
        SequentialCommandGroup command = new SequentialCommandGroup();
        resetRobotPose(command);
        command.addCommands(new InstantCommand(() -> subsystems.getDriveTrainSubsystem().setRobotPosition(NodePosition)));

        command.addCommands(new ParallelCommandGroup(
            new DriveToPointCommand(subsystems.getDriveTrainSubsystem(), NodePosition.plus(intoNodeTransform)),
            new ArmToLocationCommand(subsystems.getArmSubsystem(), ArmLocation.ARM_HIGH_SCORE)));

        command.addCommands(new ManipulatePickerCommand(subsystems.getPickerSubsystem(), true));

        command.addCommands(new ParallelCommandGroup(
            new DriveToPointCommand(subsystems.getDriveTrainSubsystem(), NodePosition),
            new ArmToLocationCommand(subsystems.getArmSubsystem(), ArmLocation.ARM_STOW)));

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
            command.addCommands(new AutoBalanceStepCommand(subsystems.getDriveTrainSubsystem(), subsystems.getNavxSubsystem())
            .repeatedly().until(subsystems.getNavxSubsystem()::isLevel));
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
        switch (AutonomousPathChooser.getSelected()) {
            case LEFT_PATH :
                return this.getLeftRoutine();
            case RIGHT_PATH :
                return this.getRightRoutine();
            case MIDDLE_PATH :
                return this.getMiddleRoutine();
        }
        return new InstantCommand();
    }

    private enum AutonomousPath {
        LEFT_PATH,
        RIGHT_PATH,
        MIDDLE_PATH,
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
