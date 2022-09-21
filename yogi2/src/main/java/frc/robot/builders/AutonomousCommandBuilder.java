// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rapid React - 2022
// File: AutonomousCommandBuilder.java
// Intent: Forms the autonomus command initiation logic for various autonomous seutps.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 
package frc.robot.builders;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class AutonomousCommandBuilder
{

    /**
     * A method to build an initial set of automated steps for first 15 seconds
     * @param collection - The grouping of subystems and input content necessary to control various operations in the robot
     * @return The command that represents a succession of commands/steps that form the action associated with this method  
     */
    public static Command buildTestCollectAndShoot(SubsystemCollection collection)
    {
        // TODO - much work needed in this command group here!!!
        SequentialCommandGroup commandGroup = new SequentialCommandGroup();

        if(collection.getBallStorageSubsystem() != null && 
           collection.getDriveTrainSubsystem() != null &&
           collection.getJawsSubsystem() != null &&
           collection.getShooterSubsystem() != null)
        {
            // example automation to attempt to
            // 1. shoot first ball
            JawsReverseHighGoal jawsToReverseHighGoal = new JawsReverseHighGoal(collection.getJawsSubsystem());
            ShooterAutomatic shootReverseHighGoal = new ShooterAutomatic(
                collection.getShooterSubsystem(),
                collection.getBallStorageSubsystem(),
                collection.getJawsSubsystem(),
                true);

            // 2. move toward the second ball 
            DriveCommand driveForwardToBall = new DriveCommand(collection.getDriveTrainSubsystem(), 49.7, 10.0, 1.9);
            JawsIntake jawsIntake = new JawsIntake(collection.getJawsSubsystem());
            ParallelCommandGroup forwardAndIntake = new ParallelCommandGroup(driveForwardToBall, jawsIntake);

            // 3. pickup another ball by intake/storage turned on and move forward to scoop the ball
            ShooterAutomatic shooterIntake = new ShooterAutomatic(
                collection.getShooterSubsystem(),
                collection.getBallStorageSubsystem(),
                collection.getJawsSubsystem(),
                false);
            DriveCommand driveForwardBallScoop = new DriveCommand(collection.getDriveTrainSubsystem(), 5.0, 0.0, 0.5);
            ParallelCommandGroup ballIntakeScoop = new ParallelCommandGroup(driveForwardBallScoop, shooterIntake);

            // 4. move back to the shooting position and move the arms up to 
            DriveCommand driveReverseToHighGoal = new DriveCommand(collection.getDriveTrainSubsystem(), 54.7, -10.0, 2.1);
            ParallelCommandGroup returnToShooterSpot = new ParallelCommandGroup(driveReverseToHighGoal, jawsToReverseHighGoal);

            // 5. shoot second ball
            ShooterAutomatic shootSecondReverseHighGoal = new ShooterAutomatic(
                collection.getShooterSubsystem(),
                collection.getBallStorageSubsystem(),
                collection.getJawsSubsystem(),
                true);

            // 6. get out of the box
            DriveCommand driveForwardOutOfBox = new DriveCommand(collection.getDriveTrainSubsystem(), 48.0, 0.0, 1);

            // 6. build the command group
            commandGroup.addCommands(
                jawsToReverseHighGoal,
                shootReverseHighGoal,
                forwardAndIntake,
                ballIntakeScoop,
                returnToShooterSpot,
                shootSecondReverseHighGoal,
                driveForwardOutOfBox
            );
        }
        
        return commandGroup;
    }
    
    /**
     * A method to build an initial set of automated steps for first 15 seconds - very simple auto
     * @param collection - The grouping of subystems and input content necessary to control various operations in the robot
     * @return The command that represents a succession of commands/steps that form the action associated with this method  
     */
    public static Command buildSimpleShootAndDrive(SubsystemCollection collection)
    {
        SequentialCommandGroup commandGroup = new SequentialCommandGroup();

        if(collection.getBallStorageSubsystem() != null && 
           collection.getDriveTrainSubsystem() != null &&
           collection.getJawsSubsystem() != null &&
           collection.getShooterSubsystem() != null)
        {
            // 1. shoot first ball
            JawsReverseLowGoal jawsToReverseLowGoal = new JawsReverseLowGoal(collection.getJawsSubsystem());
            ShooterAutomatic shootReverseLowGoal = new ShooterAutomatic(
                collection.getShooterSubsystem(),
                collection.getBallStorageSubsystem(),
                collection.getJawsSubsystem(),
                true);

            // 2. move toward the second ball 
            DriveCommand driveForwardToBall = new DriveCommand(collection.getDriveTrainSubsystem(), 38.0, 0.0, 1.5);
            JawsIntake jawsIntake = new JawsIntake(collection.getJawsSubsystem());
            ParallelCommandGroup forwardAndIntake = new ParallelCommandGroup(driveForwardToBall, jawsIntake);

            // 3. pickup another ball by intake/storage turned on and move forward to scoop the ball
            ShooterAutomatic shooterIntake = new ShooterAutomatic(
                collection.getShooterSubsystem(),
                collection.getBallStorageSubsystem(),
                collection.getJawsSubsystem(),
                false);
            DriveCommand driveForwardBallScoop = new DriveCommand(collection.getDriveTrainSubsystem(), 3.0, 0.0, 0.333);
            ParallelCommandGroup ballIntakeScoop = new ParallelCommandGroup(driveForwardBallScoop, shooterIntake);

            // 4. move back to the shooting position and move the arms up to 
            ShooterAutomatic shooterIntakeAgain = new ShooterAutomatic(
                collection.getShooterSubsystem(),
                collection.getBallStorageSubsystem(),
                collection.getJawsSubsystem(),
                false);
            DriveCommand driveForwardBallScoopAgain = new DriveCommand(collection.getDriveTrainSubsystem(), 3.0, 0.0, 0.333);
            ParallelCommandGroup ballIntakeScoopAgain = new ParallelCommandGroup(driveForwardBallScoopAgain, shooterIntakeAgain);

            // 5. shoot second ball
            ShooterAutomatic shootReverseHighGoalAgain = new ShooterAutomatic(
                collection.getShooterSubsystem(),
                collection.getBallStorageSubsystem(),
                collection.getJawsSubsystem(),
                true);

            // 6. build the command group
            commandGroup.addCommands(
                jawsToReverseLowGoal,
                shootReverseLowGoal,
                forwardAndIntake,
                ballIntakeScoop,
                ballIntakeScoopAgain,
                shootReverseHighGoalAgain
            );
        }
        
        return commandGroup;
    }
    
    /**
     * A method to build an initial set of automated steps for first 15 seconds - very simple auto
     * @param collection - The grouping of subystems and input content necessary to control various operations in the robot
     * @return The command that represents a succession of commands/steps that form the action associated with this method  
     */
    public static Command buildSimpleForwardDriveDistance(SubsystemCollection collection)
    {
        SequentialCommandGroup commandGroup = new SequentialCommandGroup();

        if(collection.getDriveTrainSubsystem() != null)
        {
            // move forward 
            DriveCommand driveCommand1 = new DriveCommand(
                collection.getDriveTrainSubsystem(),
                36.0,
                0.0,
                1.0);
        
            // 3. build the command group
            commandGroup.addCommands(driveCommand1.withTimeout(2.0));
        }
        
        return commandGroup;
    }

    /**
     * A method to build an initial set of automated steps for first 15 seconds - very simple auto
     * @param collection - The grouping of subystems and input content necessary to control various operations in the robot
     * @return The command that represents a succession of commands/steps that form the action associated with this method  
     */
    public static Command buildSimpleShootHighAndReverseDrive(SubsystemCollection collection)
    {
        SequentialCommandGroup commandGroup = new SequentialCommandGroup();

        if(collection.getBallStorageSubsystem() != null && 
           collection.getDriveTrainSubsystem() != null &&
           collection.getJawsSubsystem() != null &&
           collection.getShooterSubsystem() != null)
        {
            // 1. shoot first ball
            ButtonPress startButton = new ButtonPress("autonomous", "start");
//            JawsForwardHighGoal jawsToForwardHighGoal = new JawsForwardHighGoal(collection.getJawsSubsystem());
            ShooterAutomatic shootForwardHighGoal = new ShooterAutomatic(
                collection.getShooterSubsystem(),
                collection.getBallStorageSubsystem(),
                collection.getJawsSubsystem(),
                true);

            // 2. move toward the second ball 
            DriveTimeCommand driveTimeCommand = new DriveTimeCommand(
                collection.getDriveTrainSubsystem(),
                -1.0,
                0.0,
                1.0);

            ButtonPress endButton = new ButtonPress("autonomous", "end");

            // 3. build the command group
            commandGroup.addCommands(
                startButton.withTimeout(0.5),
//                jawsToForwardHighGoal.withTimeout(2.0),
                shootForwardHighGoal.withTimeout(5.0),
                driveTimeCommand.withTimeout(5.0),
                endButton.withTimeout(0.5));
        }
        
        return commandGroup;
    }

    /**
     * A method to build an initial set of automated steps for first 15 seconds - very simple auto
     * @param collection - The grouping of subystems and input content necessary to control various operations in the robot
     * @return The command that represents a succession of commands/steps that form the action associated with this method  
     */
    public static Command buildSimpleReverseDrive(SubsystemCollection collection)
    {
        SequentialCommandGroup commandGroup = new SequentialCommandGroup();

        if(collection.getDriveTrainSubsystem() != null)
        {
            ButtonPress startButton = new ButtonPress("autonomous", "start");

            // 2. move toward the second ball 
            DriveTimeCommand driveTimeCommand = new DriveTimeCommand(
                collection.getDriveTrainSubsystem(),
                -1.0,
                0.0,
                1.0);

            ButtonPress endButton = new ButtonPress("autonomous", "end");

            // 3. build the command group
            commandGroup.addCommands(
                startButton.withTimeout(0.5),
                driveTimeCommand.withTimeout(5.0),
                endButton.withTimeout(0.5));
        }
        
        return commandGroup;
    }
    
    /**
     * A method to build all of the stop commands and run them in parallel
     * @param collection - The grouping of subystems and input content necessary to control various operations in the robot
     * @return The command that represents a succession of commands/steps that form the action associated with this method  
     */
    public static Command buildAllStop(SubsystemCollection collection)
    {
        ParallelCommandGroup stopCommands = new ParallelCommandGroup();

        if(collection.getBallStorageSubsystem() != null)
        {
            stopCommands.addCommands(new BallStorageAllStopManual(collection.getBallStorageSubsystem()));
        }

        if(collection.getDriveTrainSubsystem() != null)
        {
            stopCommands.addCommands(new RunCommand(
                () ->
                collection.getDriveTrainSubsystem().arcadeDrive(0.0, 0.0),
                collection.getDriveTrainSubsystem()));
        }

        if(collection.getJawsSubsystem() != null)
        {
            stopCommands.addCommands(new JawsAllStop(collection.getJawsSubsystem()));
        }

        if(collection.getShooterSubsystem() != null)
        {
            stopCommands.addCommands(new ShooterAllStop(collection.getShooterSubsystem()));
        }

        if(collection.getAngleArmsSubsystem() != null)
        {
            stopCommands.addCommands(new AngleArmsAllStop(collection.getAngleArmsSubsystem()));
        }

        if(collection.getTelescopingArmsSubsystem() != null)
        {
            stopCommands.addCommands(new TelescopingArmsAllStop(collection.getTelescopingArmsSubsystem()));
        }
        
        return stopCommands;
    }
}
